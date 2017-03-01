#ifndef KUKADU_KINECT_H
#define KUKADU_KINECT_H

#include <vector>
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <kukadu/types/kukadutypes.hpp>
#include <kukadu/vision/pcstdtrans.hpp>
#include <pcl_conversions/pcl_conversions.h>

namespace kukadu {

    class Kinect {

    private:

        std::string stdVisPubTopic;

        bool isInit;
        bool keepRunning;
        bool doTransform;
        bool firstCloudSet;

        bool pcRequested;

        KUKADU_SHARED_PTR<kukadu_thread> thr;

        kukadu_mutex pcMutex;

        std::string targetFrame;
        std::string visPubTopic;
        std::string kinectTopic;

        bool kinectFrameSet;
        std::string kinectFrame;

        KUKADU_SHARED_PTR<tf::TransformListener> transformListener;

        sensor_msgs::PointCloud2::Ptr currentPc;

        ros::NodeHandle node;

        ros::Subscriber subKinect;
        ros::Subscriber subTransformation;

        ros::Publisher visPublisher;

        void runThread();
        void callbackKinectPointCloud(const sensor_msgs::PointCloud2& pc);
        void construct(std::string kinectTopic, std::string targetFrame, ros::NodeHandle node, bool doTransform = true);

    public:

        Kinect(ros::NodeHandle node, bool doTransorm = true);
        Kinect(std::string kinectTopic, std::string targetFrame, ros::NodeHandle node, bool doTransform = true);
        Kinect(std::string kinectTopic, ros::NodeHandle node, bool doTransform = true);

        void stopSensing();
        void visualizeCurrentPc();

        void storeCurrentPc(std::string fileName);

        void setVisPubTopic(std::string visPubTopic);
        void visualizeCurrentTransformedPc(KUKADU_SHARED_PTR<PCTransformator> transformator);

        bool isInitialized();

        std::string getVisPubTopic();
        std::string getKinectFrame();

        sensor_msgs::PointCloud2::Ptr getCurrentPointCloud();
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCurrentColorPointCloud();
        pcl::PointCloud<pcl::PointXYZI>::Ptr getCurrentIntensityPointCloud();

        KUKADU_SHARED_PTR<kukadu_thread> startSensing();

    };

    template <typename PointCloudPtr, typename Point> PointCloudPtr sensorMsgsPcToPclPc(sensor_msgs::PointCloud2::Ptr pc) {

        pcl::PCLPointCloud2 intermediate;
        pcl::PointCloud<Point> output;
        PointCloudPtr outputPtr;
        pcl_conversions::toPCL(*pc, intermediate);
        pcl::fromPCLPointCloud2(intermediate, output);
        outputPtr = output.makeShared();
        return outputPtr;

    }

}

#endif
