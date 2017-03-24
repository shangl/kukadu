#ifndef KUKADU_LOCALIZER_H
#define KUKADU_LOCALIZER_H

#include <memory>
#include <vector>
#include <string>
#include <utility>
#include <armadillo>
#include <geometry_msgs/Pose.h>
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

    class PoseEstimator {

    public:

        virtual std::pair<geometry_msgs::Pose, arma::vec> estimatePose(std::string id) = 0;

    };

    class PCBlobDetector : public kukadu::Localizer, public kukadu::PoseEstimator {

    private:

        std::string targetFrame;

        double xOffset;
        double yOffset;

        arma::vec center;

        std::shared_ptr<Kinect> kinect;

    public:

        PCBlobDetector(std::shared_ptr<Kinect> kinect, std::string targetFrame, arma::vec center, double xOffset, double yOffset);

        virtual std::string getLocalizerFrame();

        virtual geometry_msgs::Pose localizeObject(std::string id);

        virtual std::map<std::string, geometry_msgs::Pose> localizeObjects();

        virtual std::vector<geometry_msgs::Pose> localizeObjects(std::vector<std::string> ids);

        virtual std::pair<geometry_msgs::Pose, arma::vec> estimatePose(std::string id);

    };

}

#endif
