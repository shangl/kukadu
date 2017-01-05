#include <string>
#include <sstream>
#include <kukadu/robot/hand.hpp>
#include <kukadu/utils/utils.hpp>
#include <kukadu/robot/kukiehand.hpp>
#include <std_msgs/Float64MultiArray.h>
#include <iis_robot_dep/TactileMatrix.h>
#include <kukadu/storage/moduleusagesingleton.hpp>

using namespace std;
using namespace arma;
using namespace iis_robot_dep;

namespace kukadu {

#ifdef USEBOOST
    KukieHand::SDH_IGNORE_JOINT = DBL_MAX;
#endif

    GenericHand::GenericHand(StorageSingleton& dbStorage, std::string handInstanceName) :
        Hardware(dbStorage, HARDWARE_HAND, Hardware::loadTypeIdFromInstanceName(handInstanceName), loadTypeNameFromInstanceName(handInstanceName), Hardware::loadInstanceIdFromName(handInstanceName), handInstanceName) {

    }

    GenericHand::GenericHand(StorageSingleton& dbStorage, int handTypeId, std::string handTypeName, int handInstanceId, std::string handInstanceName) :
        Hardware(dbStorage, HARDWARE_HAND, handTypeId, handTypeName, handInstanceId, handInstanceName) {

    }

    void GenericHand::installHardwareTypeInternal() {
        // nothing specific to do
    }

    void GenericHand::installHardwareInstanceInternal() {
        // nothing specific to do
    }

    void GenericHand::storeCurrentSensorDataToDatabase() {
        auto currentTime = getCurrentTime();
        arma::vec currJoints = getCurrentJoints();
    }

    PlottingHand::PlottingHand(StorageSingleton& storage, int degOfFreedom, int sensingPatchCount, std::pair<int, int> patchDimensions) : GenericHand(storage, loadOrCreateTypeIdFromName("PlottingHand"), "PlottingHand",
                                                                                                                                    loadOrCreateInstanceIdFromName("PlottingHand"), "GenericPlottingHand") {
        this->sensingPatchCount = sensingPatchCount;
        this->patchDimensions = patchDimensions;
        this->degOfFreedom = degOfFreedom;
        currentJoints = zeros(degOfFreedom);

    }

    void PlottingHand::connectHand() {
        KUKADU_MODULE_START_USAGE();
        KUKADU_MODULE_END_USAGE();
    }

    void PlottingHand::closeHand(double percentage, double velocity) {
        KUKADU_MODULE_START_USAGE();
        KUKADU_MODULE_END_USAGE();
    }

    void PlottingHand::moveJoints(arma::vec joints) {
        KUKADU_MODULE_START_USAGE();
        currentJoints = joints;
        KUKADU_MODULE_END_USAGE();
    }

    void PlottingHand::disconnectHand() {
        KUKADU_MODULE_START_USAGE();
        KUKADU_MODULE_END_USAGE();
    }

    arma::vec PlottingHand::getCurrentJoints() {
        return currentJoints;
    }

    std::vector<arma::mat> PlottingHand::getTactileSensing() {

        KUKADU_MODULE_START_USAGE();

        std::vector<arma::mat> retSensing;
        for(int i = 0; i < sensingPatchCount; ++i) {
            arma::mat dummyPatch(patchDimensions.first, patchDimensions.second);
            dummyPatch.fill(0.0);
            retSensing.push_back(dummyPatch);
        }

        KUKADU_MODULE_END_USAGE();

        return retSensing;

    }

    std::string GenericHand::getHandName() { return getHardwareInstanceName(); }

    KukieHand::KukieHand(StorageSingleton& storage, ros::NodeHandle node, std::string simulationType, std::string hand) :
        GenericHand(storage, loadOrCreateTypeIdFromName("KukieHand"), "KukieHand", Hardware::loadOrCreateInstanceIdFromName("kukiehand_" + hand), "kukiehand_" + hand) {


        stopCollecting = false;
        this->node = node;
        waitForReached = true;
        trajPub = node.advertise<std_msgs::Float64MultiArray>(simulationType + "/" + hand + "_sdh/joint_control/move", 1);

        this->hand = hand;

        stateSub = node.subscribe(simulationType + "/" + hand + "_sdh/joint_control/get_state", 1, &KukieHand::stateCallback, this);
        tactileSub = node.subscribe("/" + simulationType + "/" + hand + "_sdh/sensoring/tactile", 1, &KukieHand::tactileCallback, this);
        previousCurrentPosQueueSize = 10;
        isFirstCallback = true;

        ros::Rate r(5);
        while(isFirstCallback) {
            ros::spinOnce();
            r.sleep();
        }

        currentGraspId = eGID_PARALLEL;
        moveJoints(stdToArmadilloVec(currentPos));

    }

    void KukieHand::setWaitForReached(bool waitForReached) {
        this->waitForReached = waitForReached;
    }

    void KukieHand::tactileCallback(const iis_robot_dep::TactileSensor& state) {

        if(!stopCollecting) {

            tactileMutex.lock();

                currentTactileReadings.clear();

                for(int i = 0; i < state.tactile_matrix.size(); ++i) {

                     TactileMatrix tactMat = state.tactile_matrix.at(i);
                     int xSize = tactMat.cells_x;
                     int ySize = tactMat.cells_y;
                     mat currentMat(xSize, ySize);

                     for(int j = 0, run = 0; j < xSize; ++j)
                         for(int k = 0; k < ySize; ++k, ++run)
                             currentMat(j, k) = tactMat.tactile_array.at(run);

                     currentTactileReadings.push_back(currentMat.t());

                }

            tactileMutex.unlock();

        }

    }

    void KukieHand::stateCallback(const sensor_msgs::JointState& state) {

        if(!stopCollecting) {

            currentPosMutex.lock();

                if(!isFirstCallback) {

                    previousCurrentPosQueue.pop();
                    previousCurrentPosQueue.push(currentPos);
                    currentPos = state.position;

                } else {

                    while(!previousCurrentPosQueue.empty())
                        previousCurrentPosQueue.pop();

                    for(int i = 0; i < previousCurrentPosQueueSize; ++i)
                        previousCurrentPosQueue.push(state.position);

                    currentPos = state.position;
                    isFirstCallback = false;

                }

            currentPosMutex.unlock();

        }

    }

    arma::vec KukieHand::getCurrentJoints() {

        currentPosMutex.lock();
            auto currJoints = currentPos;
        currentPosMutex.unlock();

        return currJoints;

    }

    bool KukieHand::vectorsDeviate(const std::vector<double> v1, const std::vector<double> v2, double tolerance) {

        bool tmpReached = true;

        for(int i = 0; i < v1.size(); ++i) {
            if(abs(v1.at(i) - v2.at(i)) > tolerance)
                tmpReached = false;
        }

        return !tmpReached;

    }

    std::vector<double> KukieHand::generateCylindricalPose(double percentage) {

        vector<double> hand_pose;

        hand_pose.push_back(0);
        hand_pose.push_back((-30 + percentage * 30) * M_PI / 180.0);
        hand_pose.push_back((30 + percentage * 35) * M_PI / 180.0);
        hand_pose.push_back((-30 + percentage * 30) * M_PI / 180.0);
        hand_pose.push_back((30 + percentage * 35) * M_PI / 180.0);
        hand_pose.push_back((-30 + percentage * 30) * M_PI / 180.0);
        hand_pose.push_back((30 + percentage * 35) * M_PI / 180.0);

        return hand_pose;

    }

    std::vector<double> KukieHand::generateParallelPose(double percentage) {

        vector<double> hand_pose;

        // finger orientation
        hand_pose.push_back(0);

        // finger 1
        hand_pose.push_back((-75. + percentage * 82.) * M_PI / 180.0);


        hand_pose.push_back((75. - percentage * 82.) * M_PI / 180.0);

        // finger 2, joint 1
        hand_pose.push_back((-75. + percentage * 82.) * M_PI / 180.0);

        // finger 2, joint 2
        hand_pose.push_back((90. - percentage * 82.) * M_PI / 180.0);

        hand_pose.push_back((-75. + percentage * 82.) * M_PI / 180.0);
        hand_pose.push_back((90. - percentage * 82.) * M_PI / 180.0);

        return hand_pose;

    }

    std::vector<double> KukieHand::generateCentricalPose(double percentage) {

        vector<double> hand_pose;

        hand_pose.push_back((60) * M_PI/180.0);
        hand_pose.push_back((-75 + percentage * 82) * M_PI / 180.0);
        hand_pose.push_back((75 - percentage * 82) * M_PI / 180.0);
        hand_pose.push_back((-75 + percentage * 82) * M_PI / 180.0);
        hand_pose.push_back((75 - percentage * 82) * M_PI / 180.0);
        hand_pose.push_back((-75 + percentage * 82) * M_PI / 180.0);
        hand_pose.push_back((75 - percentage * 82) * M_PI / 180.0);

        return hand_pose;

    }

    std::vector<double> KukieHand::generateSphericalPose(double percentage) {

        vector<double> hand_pose;

        hand_pose.push_back((60) * M_PI / 180.0);
        hand_pose.push_back((-40 + percentage * 25) * M_PI / 180.0);
        hand_pose.push_back((40 + percentage * 15) * M_PI / 180.0);
        hand_pose.push_back((-40 + percentage * 25) * M_PI / 180.0);
        hand_pose.push_back((40 + percentage * 15) * M_PI / 180.0);
        hand_pose.push_back((-40 + percentage * 25) * M_PI / 180.0);
        hand_pose.push_back((40 + percentage * 15) * M_PI / 180.0);

        return hand_pose;

    }

    void KukieHand::connectHand() {
        // nothing to do when using ros

        KUKADU_MODULE_START_USAGE();
        KUKADU_MODULE_END_USAGE();

    }

    void KukieHand::closeHand(double percentage, double velocity) {

        KUKADU_MODULE_START_USAGE();

        if(percentage >= 0.0 && percentage <= 1.1) {

            vector<double> hand_pose;

            switch(currentGraspId) {
            case eGID_CENTRICAL:
                hand_pose = generateCentricalPose(percentage);
                break;
            case eGID_CYLINDRICAL:
                hand_pose = generateCylindricalPose(percentage);
                break;
            case eGID_PARALLEL:
                hand_pose = generateParallelPose(percentage);
                break;
            case eGID_SPHERICAL:
                hand_pose = generateSphericalPose(percentage);
                break;
            default:
                string msg = "(KukieHand) grasp type not supported";
                cerr << msg << endl;
                throw KukaduException(msg.c_str());

            }

            moveJoints(stdToArmadilloVec(hand_pose));

        } else {
            string msg = "(KukieHand) grasp percentage out of range";
            cerr << msg << endl;
            throw KukaduException(msg.c_str());
        }

        KUKADU_MODULE_END_USAGE();

    }

    void KukieHand::disconnectHand() {

        // nothing to do when using ros
        KUKADU_MODULE_START_USAGE();
        KUKADU_MODULE_END_USAGE();

    }

    void KukieHand::setGrasp(kukadu_grasps grasp) {

        currentGraspId = grasp;

    }

    KukieHand::~KukieHand() {
        stopCollecting = true;
    }

    void KukieHand::safelyDestroy() {

    }

    void KukieHand::publishSingleJoint(int idx, double pos) {

        vector<double> command;
        auto ignoreJoint = SDH_IGNORE_JOINT;

        auto currJoints = getCurrentJoints();

        for(int i = 0; i < currJoints.n_elem; ++i)
            if(i == idx)
                command.push_back(pos);
            else
                command.push_back(ignoreJoint);

        moveJoints(stdToArmadilloVec(command));

    }

    void KukieHand::moveJoints(arma::vec positions) {

        KUKADU_MODULE_START_USAGE();

        auto currJoints = getCurrentJoints();
        auto ignoreJoint = SDH_IGNORE_JOINT;
        std_msgs::Float64MultiArray newJoints;
        for(int i = 0; i < positions.n_elem; ++i) {
            if(currJoints(i) != ignoreJoint)
                newJoints.data.push_back(positions(i));
            else
                newJoints.data.push_back(currJoints(i));
        }

        targetReached = false;
        movementStarted = false;

        initCurrentPos = currentPos;
        trajPub.publish(newJoints);

        if(waitForReached) {

            currJoints = getCurrentJoints();
            targetReached = !vectorsDeviate(armadilloToStdVec(currJoints), armadilloToStdVec(positions), 0.03);

            while(!targetReached) {

                bool stillMoving = false;
                // check if the fingers are still moving

                currentPosMutex.lock();
                    auto posQueueTmp = previousCurrentPosQueue;
                currentPosMutex.unlock();

                while(!posQueueTmp.empty()) {
                    auto& previousCurrentPos = posQueueTmp.front();
                    bool deviates = vectorsDeviate(previousCurrentPos, armadilloToStdVec(getCurrentJoints()), 0.01);
                    if(deviates) {
                        stillMoving = true;
                        break;
                    }
                    posQueueTmp.pop();
                }

                // if not moving anymore but the movement was started --> the target is reached
                if(!movementStarted && stillMoving)
                    movementStarted = true;
                else if(movementStarted && !stillMoving)
                    targetReached = true;

            }

        }

        KUKADU_MODULE_END_USAGE();

    }

    std::vector<arma::mat> KukieHand::getTactileSensing() {

        KUKADU_MODULE_START_USAGE();

        std::vector<arma::mat> ret;
        tactileMutex.lock();
            ret = currentTactileReadings;
        tactileMutex.unlock();

        KUKADU_MODULE_END_USAGE();

        return ret;

    }

}
