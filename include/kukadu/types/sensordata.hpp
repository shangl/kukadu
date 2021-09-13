#ifndef KUKDADU_SENSORDATA_H
#define KUKDADU_SENSORDATA_H

#include <vector>
#include <string>
#include <utility>
#include <armadillo>
#include <geometry_msgs/Pose.h>
#include <kukadu/utils/utils.hpp>

namespace kukadu {

    class SensorData {

    private:

        std::vector<std::string> labels;
        std::vector<std::string> jointPosLabels;
        std::vector<std::string> jointFrcLabels;
        std::vector<std::string> cartPosLabels;
        std::vector<std::string> cartFrcTrqLabels;
        std::vector<std::string> cartForceAbsLabel;

        std::vector<long long int> time;

        arma::mat values;

    public:

        SensorData(std::string timeLabel, std::vector<std::string> jointPosLabels, std::vector<std::string> jointFrcLabels, std::vector<std::string> cartPosLabels,
                   std::vector<std::string> cartForceAbsLabel, std::vector<std::string> cartFrcTrqLabels,
                   std::vector<long long int> time, arma::mat jointPos, arma::mat jointFrc, arma::mat cartPos, arma::mat cartForceAbs, arma::mat cartFrcTrq);

        int labelExists(std::string label);
        arma::vec getDataByIdx(int idx);
        arma::vec getDataByLabel(std::string label);
        arma::mat getRange(std::vector<std::string> indexes);
        arma::mat getRange(std::vector<int> indexes);
        arma::mat getRange(int startIdx, int endIdx);

        // this function does not make sense to be public because the resolution of double is not good enough for the values
        //arma::vec getTimeInSeconds();
        std::vector<long long int> getTimeInMilliSeconds();

        arma::vec getNormalizedTimeInSeconds();
        std::vector<long long int> getNormalizedTimeInMilliSeconds();

        arma::mat getJointPos();
        arma::mat getJointForces();
        arma::mat getCartPos();
        arma::mat getCartFrcTrqs();

        geometry_msgs::Pose getCartPose(int rowIdx);

        double getTimeNormalizedTimeInSeconds(int rowIdx);
        arma::vec getJointPosRow(int rowIdx);
        arma::vec getJointForcesRow(int rowIdx);
        arma::vec getCartPosRow(int rowIdx);
        arma::vec getCartFrcTrqsRow(int rowIdx);

        void removeDuplicateTimes();

    };

}

#endif
