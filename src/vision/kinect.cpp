#include <pcl/io/pcd_io.h>
#include <tf2_msgs/TFMessage.h>
#include <pcl_ros/transforms.h>
#include <kukadu/utils/utils.hpp>
#include <tf/transform_listener.h>
#include <kukadu/vision/kinect.hpp>
#include <kukadu/utils/kukadutokenizer.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <kukadu/storage/moduleusagesingleton.hpp>

using namespace std;
using namespace pcl;

namespace kukadu {

    pcl::PointCloud<pcl::PointXYZ>::Ptr loadPointCloudFromFile(std::string file) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ> (file.c_str(), *cloud) == -1)
            throw KukaduException("(Kinect) could not load point cloud from file");
        return cloud;
    }

    std::string extractCameraName(std::string kinectTopic) {
        KukaduTokenizer tok(kinectTopic, "/");
        if(!kinectTopic.length())
            throw KukaduException("(Kinect) emtpy topic string");
        std::string nextToken;
        while(!(nextToken = tok.next()).length())
            return nextToken;
    }

    Kinect::Kinect(StorageSingleton& dbStorage, std::string hardwareName)
        : Hardware(dbStorage, HARDWARE_DEPTH_CAMERA, Hardware::loadTypeIdFromName("Kinect"), "Kinect", Hardware::loadInstanceIdFromName(hardwareName), hardwareName) {

        this->node = ros::NodeHandle(); sleep(1);
        construct(hardwareName + "/depth_registered/points", "origin", node, true);

    }

    Kinect::Kinect(StorageSingleton& dbStorage, std::string hardwareName, bool simulate)
        : Hardware(dbStorage, HARDWARE_DEPTH_CAMERA, Hardware::loadTypeIdFromName("Kinect"), "Kinect", Hardware::loadInstanceIdFromName(hardwareName), hardwareName) {

        this->simulate = simulate;
        if(!simulate) {
            this->node = ros::NodeHandle(); sleep(1);
            construct(hardwareName + "/depth_registered/points", "origin", node, true);
        } else {
            simulatedPc = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2());
            pcl::toROSMsg(*loadPointCloudFromFile(resolvePath("$KUKADU_HOME/misc/simulatedPc.pcd")), *simulatedPc);
        }

    }

    Kinect::Kinect(StorageSingleton& dbStorage, ros::NodeHandle node, bool doTransform)
        : Hardware(dbStorage, HARDWARE_DEPTH_CAMERA, Hardware::loadOrCreateTypeIdFromName("Kinect"), "Kinect", Hardware::loadOrCreateInstanceIdFromName("camera"), "camera") {
        construct("camera/depth_registered/points", "origin", node, doTransform);
    }

    Kinect::Kinect(StorageSingleton& dbStorage, std::string kinectTopic, ros::NodeHandle node, bool doTransform)
        : Hardware(dbStorage, HARDWARE_DEPTH_CAMERA, Hardware::loadOrCreateTypeIdFromName("Kinect"), "Kinect",
                   Hardware::loadOrCreateInstanceIdFromName(extractCameraName(kinectTopic)), extractCameraName(kinectTopic)) {
        construct(kinectTopic, "origin", node, doTransform);
    }

    Kinect::Kinect(StorageSingleton& dbStorage, std::string kinectTopic, std::string targetFrame, ros::NodeHandle node, bool doTransform)
        : Hardware(dbStorage, HARDWARE_DEPTH_CAMERA, Hardware::loadOrCreateTypeIdFromName("Kinect"), "Kinect",
                   Hardware::loadOrCreateInstanceIdFromName(extractCameraName(kinectTopic)), extractCameraName(kinectTopic)) {
        construct(kinectTopic, targetFrame, node, doTransform);
    }

    Kinect::~Kinect() {
        stopSensing();
    }

    void Kinect::construct(std::string kinectTopic, std::string targetFrame, ros::NodeHandle node, bool doTransform) {

        isStarted = false;
        kinectFrameSet = false;
        this->doTransform = doTransform;

        stdVisPubTopic = "/kukadu/rviz";

        isInit = false;
        keepRunning = false;
        pcRequested = false;
        firstCloudSet = false;

        this->node = node;
        this->kinectTopic = kinectTopic;
        this->targetFrame = targetFrame;

        // kinect is started automatically
        Hardware::start();

    }

    void Kinect::installHardwareTypeInternal() {
        // nothing to do
    }

    void Kinect::installHardwareInstanceInternal() {
        // nothing to do
    }

    void Kinect::stopInternal() {
        stopSensing();
    }

    void Kinect::storeCurrentSensorDataToDatabase() {
        // throw KukaduException("(Kinect) todo");
    }

    double Kinect::getPreferredPollingFrequency() {
        return 1.0;
    }

    std::vector<std::pair<long long int, arma::vec> > Kinect::loadData(long long int startTime, long long int endTime,
                                                                       long long int maxTotalDuration,
                                                                       long long int maxTimeStepDifference) {
        throw KukaduException("(Kinect) todo");
    }

    void Kinect::startInternal() {

        if(!simulate) {
            if(!isStarted) {

                subKinect = node.subscribe(kinectTopic, 1, &Kinect::callbackKinectPointCloud, this);

                this->visPubTopic = stdVisPubTopic;
                visPublisher = node.advertise<sensor_msgs::PointCloud2>(visPubTopic, 1);

                if(doTransform)
                    transformListener = make_shared<tf::TransformListener>();

                ros::Rate r(10);
                while(!firstCloudSet)
                    r.sleep();

                if(doTransform)
                    transformListener->waitForTransform(targetFrame, currentPc->header.frame_id, ros::Time(0), ros::Duration(5.0));

                startSensing();

                isStarted = true;

            }
        }

    }

    std::string Kinect::getKinectFrame() {
        if(kinectFrameSet)
            return kinectFrame;
        throw KukaduException("(Kinect) no point cloud retrieved yet");
    }

    KUKADU_SHARED_PTR<kukadu_thread> Kinect::startSensing() {

        KUKADU_MODULE_START_USAGE();

        if(!keepRunning) {

            keepRunning = true;
            if(!thr)
                thr = KUKADU_SHARED_PTR<kukadu_thread>(new kukadu_thread(&Kinect::runThread, this));
            while(!this->isInitialized());

        }

        KUKADU_MODULE_END_USAGE();

        return thr;
    }

    std::string Kinect::getTargetFrame() {
        return this->targetFrame;
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

        if(!simulate) {

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
                        retCloud->header.stamp = ros::Time::now();
                        if(doTransform) {
                            transformListener->waitForTransform(targetFrame, retCloud->header.frame_id, retCloud->header.stamp, ros::Duration(5.0));
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

        } else {

            pcMutex.lock();

            currentPc = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2(*simulatedPc));

            pcMutex.unlock();

            return currentPc;

        }

        KUKADU_MODULE_END_USAGE();

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
