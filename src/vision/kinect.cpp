#include <pcl/io/pcd_io.h>
#include <tf2_msgs/TFMessage.h>
#include <pcl_ros/transforms.h>
#include <kukadu/utils/utils.hpp>
#include <tf/transform_listener.h>
#include <kukadu/vision/kinect.hpp>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

namespace kukadu {

    Kinect::Kinect(ros::NodeHandle node, bool doTransform) {
        string kinectPrefix = "kinect";
        construct(kinectPrefix, kinectPrefix + "_depth_frame", node, doTransform);
    }

    Kinect::Kinect(std::string kinectPrefix, ros::NodeHandle node, bool doTransform) {
        construct(kinectPrefix, kinectPrefix + "_depth_frame", node, doTransform);
    }

    Kinect::Kinect(std::string kinectPrefix, std::string targetFrame, ros::NodeHandle node, bool doTransform) {
        construct(kinectPrefix, targetFrame, node, doTransform);
    }

    void Kinect::construct(std::string kinectPrefix, std::string targetFrame, ros::NodeHandle node, bool doTransform) {

        this->doTransform = doTransform;

        stdVisPubTopic = "/kukadu/rviz";

        isInit = false;
        keepRunning = false;
        pcRequested = false;
        firstCloudSet = false;

        this->node = node;

        this->kinectPrefix = "/" + kinectPrefix;
        subKinect = node.subscribe(kinectPrefix + "/depth_registered/points", 1, &Kinect::callbackKinectPointCloud, this);

        this->visPubTopic = stdVisPubTopic;
        visPublisher = node.advertise<sensor_msgs::PointCloud2>(visPubTopic, 1);

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

    KUKADU_SHARED_PTR<kukadu_thread> Kinect::startSensing() {
        keepRunning = true;
        thr = KUKADU_SHARED_PTR<kukadu_thread>(new kukadu_thread(&Kinect::runThread, this));
        while(!this->isInitialized());
        return thr;
    }

    void Kinect::stopSensing() {
        keepRunning = false;
    }

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

        pcMutex.unlock();


    }

    sensor_msgs::PointCloud2::Ptr Kinect::getCurrentPointCloud() {

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

        return retCloud;

    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Kinect::getCurrentPclPointCloud() {
        return sensorMsgsPcToPclPc(getCurrentPointCloud());
    }

    std::string Kinect::getVisPubTopic() {
        return visPubTopic;
    }

    void Kinect::setVisPubTopic(std::string visPubTopic) {
        this->visPubTopic = visPubTopic;
        visPublisher = node.advertise<sensor_msgs::PointCloud2>(this->visPubTopic, 1);
    }

    void Kinect::visualizeCurrentPc() {
        visPublisher.publish(getCurrentPointCloud());
    }

    void Kinect::visualizeCurrentTransformedPc(KUKADU_SHARED_PTR<PCTransformator> transformator) {

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr currentPc = sensorMsgsPcToPclPc(getCurrentPointCloud());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed = transformator->transformPc(currentPc);
        visPublisher.publish(pclPcToSensorMsgsPc(transformed));

    }

    void Kinect::storeCurrentPc(std::string fileName) {
        pcl::io::savePCDFile(fileName, sensorMsgsPcToPclPc2(*getCurrentPointCloud()), Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), true);
    }

}
