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

    public:

        virtual arma::mat calibrateRotation() = 0;
        virtual arma::vec calibrateTranslation() = 0;

    };

    class KinectCalibrator : public Calibrator {

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

        KUKADU_SHARED_PTR<ControlQueue> queue;
        KUKADU_SHARED_PTR<Localizer> localizer;

        geometry_msgs::Pose getPoseFromLine(const std::string& line);

        geometry_msgs::Pose getCurrentPoseRobot();
        std::pair<bool, geometry_msgs::Pose> getCurrentPoseCamera();

        void dataCollectionRunner();

        bool isSignificantlyDifferenct(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2);

    public:

        KinectCalibrator(KUKADU_SHARED_PTR<ControlQueue> queue, KUKADU_SHARED_PTR<Localizer> localizer, std::string attachedObjectId, bool storeDataToFile = false, std::string storePath = "",
                         double minCartDistance = 0.03, double minQuatDistance = 0.03);
        ~KinectCalibrator();

        virtual void startDataCollection();
        virtual void endDataCollection();

        virtual arma::mat calibrateRotation();
        virtual arma::vec calibrateTranslation();

        void setReadDataFromRobot();
        void setReadDataFromFile(std::string file);

    };

}

#endif
