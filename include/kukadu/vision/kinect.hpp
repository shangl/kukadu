#ifndef KUKADU_KINECT_H
#define KUKADU_KINECT_H

#include <vector>
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <kukadu/robot/hardware.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <kukadu/types/kukadutypes.hpp>
#include <kukadu/vision/pcstdtrans.hpp>
#include <pcl_conversions/pcl_conversions.h>

namespace kukadu {

    class Kinect : public Hardware {

    private:

        std::string stdVisPubTopic;

        bool isInit;
        bool simulate;
        bool keepRunning;
        bool doTransform;
        bool firstCloudSet;

        bool isStarted;

        bool pcRequested;

        KUKADU_SHARED_PTR<kukadu_thread> thr;

        kukadu_mutex pcMutex;

        std::string targetFrame;
        std::string visPubTopic;
        std::string kinectTopic;

        bool kinectFrameSet;
        std::string kinectFrame;
        KUKADU_SHARED_PTR<kukadu_thread> listenerThread;

        KUKADU_SHARED_PTR<tf::TransformListener> transformListener;

        sensor_msgs::PointCloud2::Ptr currentPc;
        sensor_msgs::PointCloud2::Ptr simulatedPc;

        ros::NodeHandle node;

        ros::Subscriber subKinect;
        ros::Subscriber subTransformation;

        ros::Publisher visPublisher;

        void runThread();
        void callbackKinectPointCloud(const sensor_msgs::PointCloud2& pc);
        void construct(std::string kinectTopic, std::string targetFrame, ros::NodeHandle node, bool doTransform = true);

    protected:

        virtual void installHardwareTypeInternal();
        virtual void installHardwareInstanceInternal();

        virtual void startInternal();
        virtual void stopInternal();

    public:

        Kinect(StorageSingleton& dbStorage, std::string hardwareName);
        Kinect(StorageSingleton& dbStorage, std::string hardwareName, bool simulate);
        Kinect(StorageSingleton& dbStorage, ros::NodeHandle node, bool doTransorm = true);
        Kinect(StorageSingleton& dbStorage, std::string kinectTopic, std::string targetFrame, ros::NodeHandle node, bool doTransform = true);
        Kinect(StorageSingleton& dbStorage, std::string kinectTopic, ros::NodeHandle node, bool doTransform = true);

        ~Kinect();

        std::string getTargetFrame();

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

        virtual void storeCurrentSensorDataToDatabase();
        virtual double getPreferredPollingFrequency();

        /* returns a vector of sensor data of the given hardware and the corresponding time */
        /* the maximum duration of the whole exported series is 1 hour (3600000 ms) */
        virtual std::vector<std::pair<long long int, arma::vec> > loadData(long long int startTime, long long int endTime,
                                                                           long long int maxTotalDuration = 3600000,
                                                                           long long int maxTimeStepDifference = 5000);

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
