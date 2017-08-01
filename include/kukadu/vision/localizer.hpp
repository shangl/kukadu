#ifndef KUKADU_LOCALIZER_H
#define KUKADU_LOCALIZER_H

#include <memory>
#include <vector>
#include <string>
#include <utility>
#include <armadillo>
#include <geometry_msgs/Pose.h>
#include <kukadu/utils/utils.hpp>
#include <kukadu/vision/kinect.hpp>
#include <kukadu/types/kukadutypes.hpp>

namespace kukadu {

    class Localizer {

    private:

    public:

        virtual std::string getLocalizerFrame() = 0;

        virtual geometry_msgs::Pose localizeObject(std::string id) = 0;
        virtual std::map<std::string, geometry_msgs::Pose> localizeObjects() = 0;
        virtual std::vector<geometry_msgs::Pose> localizeObjects(std::vector<std::string> ids) = 0;

    };

    class PoseEstimator : public TimedObject {

    private:
        StorageSingleton& storage;
        std::string poseEstimatorName;

    protected:

        virtual std::vector<std::string> getAvailableObjects() = 0;

        virtual void installInternal() = 0;

    public:

        PoseEstimator(StorageSingleton& dbStorage, std::string poseEstimatorName);

        virtual std::pair<geometry_msgs::Pose, arma::vec> estimatePose(std::string id) = 0;

        void install();

    };

    class PCBlobDetector : public kukadu::Localizer, public kukadu::PoseEstimator {

    private:

        std::string targetFrame;

        bool visualizeResult;

        double xOffset;
        double yOffset;

        arma::vec center;

        std::shared_ptr<Kinect> kinect;

    protected:

        virtual std::vector<std::string> getAvailableObjects();

        virtual void installInternal();

    public:

        PCBlobDetector(std::shared_ptr<Kinect> kinect, StorageSingleton& dbStorage, std::string targetFrame = "origin", arma::vec center = stdToArmadilloVec({0.0, 0.0, 0.0}), double xOffset = 2.0, double yOffset = 2.0, bool visualizeResult = false);

        virtual std::string getLocalizerFrame();

        virtual geometry_msgs::Pose localizeObject(std::string id);

        virtual std::map<std::string, geometry_msgs::Pose> localizeObjects();

        virtual std::vector<geometry_msgs::Pose> localizeObjects(std::vector<std::string> ids);

        virtual std::pair<geometry_msgs::Pose, arma::vec> estimatePose(std::string id);

    };

}

#endif
