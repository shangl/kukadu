#include <kukadu/types/sensordata.hpp>
#include <kukadu/types/kukadutypes.hpp>

using namespace std;
using namespace arma;

namespace kukadu {

    SensorData::SensorData(std::string timeLabel, std::vector<std::string> jointPosLabels, std::vector<std::string> jointFrcLabels, std::vector<std::string> cartPosLabels,
               std::vector<std::string> cartForceAbsLabel, std::vector<std::string> cartFrcTrqLabels, std::vector<long long int> time, arma::mat jointPos, arma::mat jointFrc, arma::mat cartPos, arma::mat cartForceAbs, arma::mat cartFrcTrq) {

        this->jointPosLabels = jointPosLabels;
        this->jointFrcLabels = jointPosLabels;
        this->cartPosLabels = cartPosLabels;
        this->cartFrcTrqLabels = cartFrcTrqLabels;
        this->cartForceAbsLabel = cartForceAbsLabel;

        labels.push_back(timeLabel);
        if(jointPos.n_cols > 1) {
            for(int i = 0; i < jointPosLabels.size(); ++i)
                labels.push_back(jointPosLabels.at(i));
        } else
            this->jointPosLabels.clear();

        if(jointFrc.n_cols > 1) {
            for(int i = 0; i < jointFrcLabels.size(); ++i)
                labels.push_back(jointFrcLabels.at(i));
        } else
            this->jointFrcLabels.clear();

        if(cartPos.n_cols > 1) {
            for(int i = 0; i < cartPosLabels.size(); ++i)
                labels.push_back(cartPosLabels.at(i));
        } else
            this->cartPosLabels.clear();

        if(cartFrcTrq.n_cols > 1) {
            for(int i = 0; i < cartFrcTrqLabels.size(); ++i)
                labels.push_back(cartFrcTrqLabels.at(i));
        } else
            this->cartFrcTrqLabels.clear();

        if(cartForceAbs.n_cols > 1) {
            for(int i = 0; i < cartForceAbsLabel.size(); ++i)
                labels.push_back(cartForceAbsLabel.at(i));
        } else
            this->cartForceAbsLabel.clear();

        this->time = time;
        removeDuplicateTimes();

        if(jointPos.n_cols > 1)
            if(values.n_rows > 1) values = armaJoinRows(values, jointPos);
            else values = jointPos;

        if(jointFrc.n_cols > 1)
            if(values.n_rows > 1) values = armaJoinRows(values, jointFrc);
            else values = jointFrc;

        if(cartPos.n_cols > 1)
            if(values.n_rows > 1) values = armaJoinRows(values, cartPos);
            else values = cartPos;

        if(cartFrcTrq.n_cols > 1)
            if(values.n_rows > 1) values = armaJoinRows(values, cartFrcTrq);
            else values = cartFrcTrq;

        if(cartForceAbs.n_cols > 1)
            if(values.n_rows > 1) values = armaJoinRows(values, cartForceAbs);
            else values = cartForceAbs;

        auto labelCount = jointPosLabels.size() + jointFrcLabels.size() + cartPosLabels.size() + cartFrcTrqLabels.size() + cartForceAbsLabel.size();
        if(labelCount != values.n_cols)
            throw KukaduException("(SensorData) the number of labels does not match the number of data columns");

    }

    geometry_msgs::Pose SensorData::getCartPose(int rowIdx) {
        auto poseRow = getCartPosRow(rowIdx);
        geometry_msgs::Pose retPose;
        retPose.position.x = poseRow(0);
        retPose.position.y = poseRow(1);
        retPose.position.z = poseRow(2);
        retPose.orientation.x = poseRow(3);
        retPose.orientation.y = poseRow(4);
        retPose.orientation.z = poseRow(5);
        retPose.orientation.w = poseRow(6);
        return retPose;
    }

    void SensorData::removeDuplicateTimes() {

        double currentTime = DBL_MAX;
        int vecSize = values.n_rows;
        for(int i = 0; i < vecSize; ++i) {
            double nextTime = getTimeNormalizedTimeInSeconds(i);
            if(currentTime == nextTime) {
                values.shed_row(i);
                time.erase(time.begin() + i);
            }
        }

    }

    int SensorData::labelExists(std::string label) {

        for(int i = 0; i < labels.size(); ++i) {
            if(!label.compare(labels.at(i)))
                return i;
        }

        return -1;

    }

    arma::vec SensorData::getDataByIdx(int idx) {
        return vec(values.col(idx));
    }

    std::vector<long long int> SensorData::getTimeInMilliSeconds() {
        return time;
    }

    arma::vec SensorData::getNormalizedTimeInSeconds() {
        auto normalizedTimeInMilliseconds = getNormalizedTimeInMilliSeconds();
        return convertTimesInMillisecondsToTimeInSeconds(normalizedTimeInMilliseconds);
    }

    std::vector<long long int> SensorData::getNormalizedTimeInMilliSeconds() {
        auto timeInSeconds = getTimeInMilliSeconds();
        auto t = convertAndRemoveOffset(timeInSeconds);
        return convertTimesInSecondsToTimeInMilliseconds(t);
    }

    arma::vec SensorData::getDataByLabel(std::string label) {
        return getDataByIdx(labelExists(label));
    }

    arma::mat SensorData::getRange(std::vector<std::string> indexes) {

        vector<int> intIndexes;
        for(int i = 0; i < indexes.size(); ++i) {

            int idx = labelExists(indexes.at(i));
            if(idx >= 0)
                intIndexes.push_back(idx);
            else
                return mat(1, 1);

        }

        return getRange(intIndexes);

    }

    arma::mat SensorData::getRange(std::vector<int> indexes) {

        mat retMat = getDataByIdx(indexes.at(0));
        for(int i = 1; i < indexes.size(); ++i) {
            vec currVec = getDataByIdx(indexes.at(i));
            retMat = join_rows(retMat, currVec);
        }

        return retMat;

    }

    arma::mat SensorData::getRange(int startIdx, int endIdx) {

        vector<int> rangeVec;
        for(; startIdx < endIdx; ++startIdx)
            rangeVec.push_back(startIdx);

        return getRange(rangeVec);

    }

    arma::mat SensorData::getJointPos() {
        if(!jointPosLabels.size())
            throw KukaduException("(SensorData) no joint positions stored");
        return values.cols(0, jointPosLabels.size() - 1);
    }

    arma::mat SensorData::getJointForces() {
        if(!jointPosLabels.size())
            throw KukaduException("(SensorData) no joint positions stored");
        return values.cols(jointPosLabels.size(), jointPosLabels.size() + jointFrcLabels.size() - 1);
    }

    arma::mat SensorData::getCartPos() {
        if(!cartPosLabels.size())
            throw KukaduException("(SensorData) no Cartesian positions stored");
        return values.cols(jointPosLabels.size() + jointFrcLabels.size(), jointPosLabels.size() + jointFrcLabels.size() + cartPosLabels.size() - 1);
    }

    arma::mat SensorData::getCartFrcTrqs() {
        if(!cartFrcTrqLabels.size())
            throw KukaduException("(SensorData) no Cartesian positions stored");
        return values.cols(jointPosLabels.size() + jointFrcLabels.size() + cartPosLabels.size(), jointPosLabels.size() + jointFrcLabels.size() + cartPosLabels.size() + cartFrcTrqLabels.size() - 1);
    }

    double SensorData::getTimeNormalizedTimeInSeconds(int rowIdx) {
        return getNormalizedTimeInSeconds()(rowIdx);
    }

    arma::vec SensorData::getJointPosRow(int rowIdx) {
        if(!jointPosLabels.size())
            throw KukaduException("(SensorData) no joint positions stored");
        vec retVal(jointPosLabels.size());
        for(int i = 0; i < jointPosLabels.size(); ++i)
            retVal(i) = values(rowIdx, i + 1);
        return retVal;
    }

    arma::vec SensorData::getJointForcesRow(int rowIdx) {
        if(!jointPosLabels.size())
            throw KukaduException("(SensorData) no joint positions stored");
        vec retVal(jointFrcLabels.size());
        for(int i = 0; i < jointFrcLabels.size(); ++i)
            retVal(i) = values(rowIdx, i + 1 + jointPosLabels.size());
        return retVal;
    }

    arma::vec SensorData::getCartPosRow(int rowIdx) {
        if(!cartPosLabels.size())
            throw KukaduException("(SensorData) no Cartesian positions stored");
        vec retVal(cartPosLabels.size());
        for(int i = 0; i < cartPosLabels.size(); ++i)
            retVal(i) = values(rowIdx, i + 1 + jointPosLabels.size() + jointFrcLabels.size());
        return retVal;
    }

    arma::vec SensorData::getCartFrcTrqsRow(int rowIdx) {
        if(!cartFrcTrqLabels.size())
            throw KukaduException("(SensorData) no Cartesian positions stored");
        vec retVal(cartFrcTrqLabels.size());
        for(int i = 0; i < cartFrcTrqLabels.size(); ++i)
            retVal(i) = values(rowIdx, i + 1 + jointPosLabels.size() + jointFrcLabels.size() + cartPosLabels.size());
        return retVal;
    }

}
