#ifndef KUKADU_CALIBRATION_H
#define KUKADU_CALIBRATION_H

#include <istream>
#include <utility>
#include <armadillo>
#include <geometry_msgs/Pose.h>
#include <kukadu/robot/queue.hpp>
#include <kukadu/types/kukadutypes.hpp>
#include <kukadu/vision/arlocalizer.hpp>

namespace kukadu {

    class Calibrator {

        std::string originalFrame;
        std::string targetFrame;

    public:

        Calibrator(std::string originalFrame, std::string targetFrame);

        virtual tf::Transform calibrateTfTransform();
        virtual arma::mat calibrateAffineTransMatrix();
        virtual std::pair<arma::mat, arma::vec> calibrate() = 0;

        std::string getOriginalFrame();
        std::string getTargetFrame();

    };

    class CameraCalibrator : public Calibrator {

    private:

        bool runDataCollectionThread;

        bool storeDataToFile;
        std::string storePath;
        std::string attachedObjectId;

        bool readFromFile;
        std::ifstream inStream;

        double minCartDistance;
        double minQuatDistance;

        std::ofstream outFile;

        kukadu_thread collectionThread;

        geometry_msgs::Pose lastObjectPoseInCamFrame;
        geometry_msgs::Pose lastObjectPoseInRobotFrame;

        int dataPointCount;
        arma::vec centroidRobot;
        arma::vec centroidCamera;
        std::vector<arma::vec> samplesRobot;
        std::vector<arma::vec> samplesCamera;
        kukadu_mutex dataMutex;

        KUKADU_SHARED_PTR<ControlQueue> queue;
        KUKADU_SHARED_PTR<Localizer> localizer;

        geometry_msgs::Pose getPoseFromLine(const std::string& line);

        geometry_msgs::Pose getCurrentPoseRobot();
        std::pair<bool, geometry_msgs::Pose> getCurrentPoseCamera();

        void initializeForNewRun();
        void dataCollectionRunner();

        bool isSignificantlyDifferenct(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2);

    public:

        CameraCalibrator(KUKADU_SHARED_PTR<ControlQueue> queue, KUKADU_SHARED_PTR<Localizer> localizer, std::string attachedObjectId, bool storeDataToFile = false, std::string storePath = "",
                         double minCartDistance = 0.03, double minQuatDistance = 0.03);
        ~CameraCalibrator();

        virtual void startDataCollection();
        virtual void endDataCollection();

        virtual std::pair<arma::mat, arma::vec> calibrate();

        void setReadDataFromRobot();
        void setReadDataFromFile(std::string file);

    };

}

#endif
