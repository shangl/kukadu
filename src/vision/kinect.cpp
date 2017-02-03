#include <pcl/io/pcd_io.h>
#include <tf2_msgs/TFMessage.h>
#include <pcl_ros/transforms.h>
#include <kukadu/utils/utils.hpp>
#include <tf/transform_listener.h>
#include <kukadu/vision/kinect.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <kukadu/storage/moduleusagesingleton.hpp>

using namespace std;
using namespace pcl;

namespace kukadu {

    Kinect::Kinect(ros::NodeHandle node, bool doTransform) {
        construct("camera/depth_registered/points", "world_link", node, doTransform);
    }

    Kinect::Kinect(std::string kinectTopic, ros::NodeHandle node, bool doTransform) {
        construct(kinectTopic, "world_link", node, doTransform);
    }

    Kinect::Kinect(std::string kinectTopic, std::string targetFrame, ros::NodeHandle node, bool doTransform) {
        construct(kinectTopic, targetFrame, node, doTransform);
    }

    void Kinect::construct(std::string kinectTopic, std::string targetFrame, ros::NodeHandle node, bool doTransform) {

        kinectFrameSet = false;
        this->doTransform = doTransform;

        stdVisPubTopic = "/kukadu/rviz";

        isInit = false;
        keepRunning = false;
        pcRequested = false;
        firstCloudSet = false;

        this->node = node;

        subKinect = node.subscribe(kinectTopic, 1, &Kinect::callbackKinectPointCloud, this);

        this->visPubTopic = stdVisPubTopic;
        visPublisher = node.advertise<sensor_msgs::PointCloud2>(visPubTopic, 1);

        if(doTransform)
            transformListener = KUKADU_SHARED_PTR<tf::TransformListener>(new tf::TransformListener());

        ros::Rate r(10);
        while(!firstCloudSet) {
            r.sleep();
            ros::spinOnce();
        }

        if(doTransform)
            transformListener->waitForTransform(targetFrame, currentPc->header.frame_id, ros::Time::now(), ros::Duration(5.0));

        this->targetFrame = targetFrame;

    }

    std::string Kinect::getKinectFrame() {
        if(kinectFrameSet)
            return kinectFrame;
        throw KukaduException("(Kinect) no point cloud retrieved yet");
    }

    KUKADU_SHARED_PTR<kukadu_thread> Kinect::startSensing() {

        KUKADU_MODULE_START_USAGE();

        keepRunning = true;
        thr = KUKADU_SHARED_PTR<kukadu_thread>(new kukadu_thread(&Kinect::runThread, this));
        while(!this->isInitialized());

        KUKADU_MODULE_END_USAGE();

        return thr;
    }

    void Kinect::stopSensing() {

        KUKADU_MODULE_START_USAGE();

        keepRunning = false;
        if(thr && thr->joinable())
            thr->join();

        KUKADU_MODULE_END_USAGE();

    }

    // question to myself: why is this here???
    void Kinect::runThread() {

        isInit = true;
        ros::Rate r(10);
        while(keepRunning)
            r.sleep();

    }

    bool Kinect::isInitialized() {
        return isInit;
    }

    void Kinect::callbackKinectPointCloud(const sensor_msgs::PointCloud2& pc) {

        pcMutex.lock();

            if(pcRequested || !firstCloudSet)
                currentPc = boost::make_shared<sensor_msgs::PointCloud2>(pc);

            pcRequested = false;
            firstCloudSet = true;

            kinectFrame = currentPc->header.frame_id;
            kinectFrameSet = true;

        pcMutex.unlock();


    }

    sensor_msgs::PointCloud2::Ptr Kinect::getCurrentPointCloud() {

        KUKADU_MODULE_START_USAGE();

        pcRequested = true;

        ros::Rate kinectRate(10);
        while(pcRequested)
            kinectRate.sleep();

        sensor_msgs::PointCloud2::Ptr retCloud;

        pcMutex.lock();

            bool tfWorked = false;

            while(!tfWorked) {
                retCloud = currentPc;
                try {
                    tf::StampedTransform trans;
                    retCloud->header.stamp = ros::Time(0);
                    if(doTransform) {
                        transformListener->lookupTransform(targetFrame, retCloud->header.frame_id, ros::Time(0), trans);
                        pcl_ros::transformPointCloud(targetFrame, *retCloud, *retCloud, *transformListener);
                    }
                    tfWorked = true;
                } catch(tf::TransformException ex) {
                    cerr << "transformation not found" << endl;
                }
            }

            firstCloudSet = false;

        pcMutex.unlock();

        KUKADU_MODULE_END_USAGE();

        return retCloud;

    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Kinect::getCurrentColorPointCloud() {
        KUKADU_MODULE_START_USAGE();
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr retCloud = sensorMsgsPcToPclPc<pcl::PointCloud<PointXYZRGB>::Ptr, pcl::PointXYZRGB>(getCurrentPointCloud());
        KUKADU_MODULE_END_USAGE();
        return retCloud;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr Kinect::getCurrentIntensityPointCloud() {
        KUKADU_MODULE_START_USAGE();
        pcl::PointCloud<pcl::PointXYZI>::Ptr retCloud = sensorMsgsPcToPclPc<pcl::PointCloud<PointXYZI>::Ptr, pcl::PointXYZI>(getCurrentPointCloud());
        KUKADU_MODULE_END_USAGE();
        return retCloud;
    }

    std::string Kinect::getVisPubTopic() {
        return visPubTopic;
    }

    void Kinect::setVisPubTopic(std::string visPubTopic) {
        this->visPubTopic = visPubTopic;
        visPublisher = node.advertise<sensor_msgs::PointCloud2>(this->visPubTopic, 1);
    }

    void Kinect::visualizeCurrentPc() {
        KUKADU_MODULE_START_USAGE();
        visPublisher.publish(getCurrentPointCloud());
        KUKADU_MODULE_END_USAGE();
    }

    void Kinect::visualizeCurrentTransformedPc(KUKADU_SHARED_PTR<PCTransformator> transformator) {

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr currentPc = sensorMsgsPcToPclPc<pcl::PointCloud<PointXYZRGB>::Ptr, pcl::PointXYZRGB>(getCurrentPointCloud());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed = transformator->transformPc(currentPc);
        visPublisher.publish(pclPcToSensorMsgsPc(transformed));

    }

    void Kinect::storeCurrentPc(std::string fileName) {
        KUKADU_MODULE_START_USAGE();
        pcl::io::savePCDFile(fileName, sensorMsgsPcToPclPc2(*getCurrentPointCloud()), Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), true);
        KUKADU_MODULE_END_USAGE();
    }

}
