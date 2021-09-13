#include <string>
#include <sstream>
#include <kukadu/robot/hand.hpp>
#include <kukadu/utils/utils.hpp>
#include <kukadu/utils/tictoc.hpp>
#include <kukadu/robot/kukiehand.hpp>
#include <kukadu/types/kukadutypes.hpp>
#include <std_msgs/Float64MultiArray.h>
#include <kukadu/types/kukadutypes.hpp>
#include <iis_robot_dep/TactileMatrix.h>
#include <kukadu/storage/sensorstorage.hpp>
#include <kukadu/storage/moduleusagesingleton.hpp>

using namespace std;
using namespace arma;
using namespace iis_robot_dep;

namespace kukadu {

#ifdef USEBOOST
    KukieHand::SDH_IGNORE_JOINT = DBL_MAX;
#endif

    GenericHand::GenericHand(StorageSingleton& dbStorage, std::string handInstanceName) :
        JointHardware(dbStorage, HARDWARE_HAND, Hardware::loadTypeIdFromInstanceName(handInstanceName), loadTypeNameFromInstanceName(handInstanceName), Hardware::loadInstanceIdFromName(handInstanceName), handInstanceName) {
        isFirstStorage = true;
    }

    GenericHand::GenericHand(StorageSingleton& dbStorage, int handTypeId, std::string handTypeName, int handInstanceId, std::string handInstanceName) :
        JointHardware(dbStorage, HARDWARE_HAND, handTypeId, handTypeName, handInstanceId, handInstanceName) {
        isFirstStorage = true;
    }

    void GenericHand::installHardwareTypeInternal() {

    }

    void GenericHand::installHardwareInstanceInternal() {

        // install joint names
        auto jointNames = getJointNames();
        auto hardwareInstId = getHardwareInstance();

        stringstream s;
        for(auto& jointName : jointNames) {
            s.str("");
            auto nextJointId = getStorage().getNextIdInTable("hardware_joints", "joint_id");
            s << "insert into hardware_joints(hardware_instance_id, joint_id, joint_name) values(" << hardwareInstId << ", " << nextJointId << ", '" << jointName << "')";
            getStorage().executeStatementPriority(s.str());
        }
    }

    void GenericHand::startInternal() {
        connectHand();
    }

    void GenericHand::stopInternal() {

    }

    void GenericHand::startRollBackMode(double) {
        throw KukaduException("(GenericHand) rollback mode not available yet for hands");
    }

    void GenericHand::rollBack(double) {
        throw KukaduException("(GenericHand) rollback mode not available yet for hands");
    }

    void GenericHand::stopJointRollBackMode() {
        throw KukaduException("(GenericHand) rollback mode not available yet for hands");
    }

    std::vector<int> JointHardware::getJointIds() {
        return getJointIds(getJointNames());
    }

    std::vector<int> JointHardware::getJointIds(std::vector<std::string> jointNames) {
        vector<int> jointIds;
        for(int i = 0; i < jointNames.size(); ++i)
            jointIds.push_back(getJointId(jointNames.at(i)));
        return jointIds;
    }

    std::vector<mes_result> GenericHand::jointPtp(arma::vec joints, double maxForce) {
        if(maxForce >= 0.0)
            throw KukaduException("(GenericHand) max force is currently not supported yet");
        moveJoints(joints);
        return {};
    }

    double GenericHand::getPreferredPollingFrequency() {
        return 5.0;
    }

    void GenericHand::storeCurrentSensorDataToDatabase() {

        if(isFirstStorage) {
            prevPrevJoints = prevJoints = nowJoints = {getCurrentTime(), getCurrentJoints()};
            isFirstStorage = false;
        }

        vec jointFrcs = zeros(1);

        auto& storage = getStorage();
        auto robotId = getHardwareInstance();
        auto jointIds = getJointIds();

        prevPrevJoints = prevJoints;
        prevJoints = nowJoints;
        nowJoints = {getCurrentTime(), getCurrentJoints()};

        auto time = getCurrentTime();

        vec vel;
        vec acc;
        // if the previous times are the same, it is the first time - acceleration are 0
        if(prevPrevJoints.time == prevJoints.time)
            acc = zeros(nowJoints.joints.n_elem);
        else {
            auto timeDiffInSec = (double) (prevJoints.time - prevPrevJoints.time) / 1000.0;
            acc = (prevPrevJoints.joints - prevJoints.joints) / timeDiffInSec;
        }

        // same reasoning vor velocity
        if(prevJoints.time == nowJoints.time)
            vel = zeros(nowJoints.joints.n_elem);
        else {
            auto timeDiffInSec = (double) (nowJoints.time - prevJoints.time) / 1000.0;
            vel = (nowJoints.joints - prevJoints.joints) / timeDiffInSec;
        }

        // store the data in the database
        SensorStorage::storeJointInfoToDatabase(storage, robotId, time, jointIds, nowJoints.joints, vel, acc, false, jointFrcs);

    }

    PlottingHand::PlottingHand(StorageSingleton& storage, int degOfFreedom, int sensingPatchCount, std::pair<int, int> patchDimensions) : GenericHand(storage, loadOrCreateTypeIdFromName("PlottingHand"), "PlottingHand",
                                                                                                                                    loadOrCreateInstanceIdFromName("PlottingHand"), "GenericPlottingHand") {
        this->sensingPatchCount = sensingPatchCount;
        this->patchDimensions = patchDimensions;
        this->degOfFreedom = degOfFreedom;
        currentJoints = zeros(degOfFreedom);

        stringstream s;
        for(int i = 0; i < degOfFreedom; ++i) {
            s.str("");
            s << "plotting_joint_" << i;
            jointNames.push_back(s.str());
        }

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

    std::vector<std::string> PlottingHand::getJointNames() {
        return jointNames;
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

    KukieHand::KukieHand(StorageSingleton& storage, std::string robotName, bool simulation) :
        GenericHand(storage, loadTypeIdFromName("KukieHand"), "KukieHand", Hardware::loadInstanceIdFromName(robotName), robotName) {

        auto handLabel = storage.getCachedLabel("kukie_hardware", "hardware_instance_id", "name_prefix", getHardwareInstance());
        construct(handLabel, simulation);

    }

    KukieHand::KukieHand(StorageSingleton& storage, ros::NodeHandle node, bool simulation, std::string hand) :
        GenericHand(storage, loadOrCreateTypeIdFromName("KukieHand"), "KukieHand", Hardware::loadOrCreateInstanceIdFromName("kukiehand_" + hand), "kukiehand_" + hand) {

        construct(hand, simulation);

    }

    void KukieHand::construct(std::string hand, bool simulation) {

        this->hand = hand;
        this->node = ros::NodeHandle();
        sleep(1);

        firstJointNamesRetrieval = true;
        stopCollecting = false;
        waitForReached = true;

        degOfFreedom = -1;

        string simulationType = (simulation) ? "simulation" : "real";
        string moveTopic = simulationType + "/" + hand + "_sdh/joint_control/move";

        trajPub = node.advertise<std_msgs::Float64MultiArray>(moveTopic, 1);

        stateSub = node.subscribe(simulationType + "/" + hand + "_sdh/joint_control/get_state", 1, &KukieHand::stateCallback, this);
        tactileSub = node.subscribe("/" + simulationType + "/" + hand + "_sdh/sensoring/tactile", 1, &KukieHand::tactileCallback, this);
        previousCurrentPosQueueSize = 10;
        isFirstCallback = true;

        ros::Rate r(5);
        while(isFirstCallback) {
            ros::spinOnce();
            r.sleep();
        }

        degOfFreedom = getCurrentJoints().n_elem;

        currentGraspId = eGID_PARALLEL;
        moveJoints(stdToArmadilloVec(currentPos));

    }

    void KukieHand::installHardwareInstanceInternal() {

        StorageSingleton& storage = StorageSingleton::get();

        auto degOfFreedom = getDegreesOfFreedom();
        auto frequency = 0.0;
        auto prefix = hand;
        auto hardwareInstance = getHardwareInstance();

        stringstream s;

        s << "select hardware_instance_id from kukie_hardware where hardware_instance_id = " << hardwareInstance;
        auto result = storage.executeQuery(s.str());

        if(result->next()) {

        } else {
            s.str("");
            s << "insert into kukie_hardware values(" << hardwareInstance << ", " << degOfFreedom << ", " << frequency << ", '" << prefix << "')";
            storage.executeStatementPriority(s.str());
        }

    }

    std::vector<std::string> KukieHand::getJointNames() {

        if(firstJointNamesRetrieval) {
            jointNames.clear();
            stringstream s;
            for(int i = 0; i < degOfFreedom; ++i) {
                s.str("");
                s << getHardwareInstanceName() << "_joint_" << i;
                jointNames.push_back(s.str());
            }
            firstJointNamesRetrieval = false;
        }

        return jointNames;

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

                    if(!previousCurrentPosQueue.empty()) {

                        previousCurrentPosQueue.pop();
                        previousCurrentPosQueue.push(currentPos);
                        currentPos = state.position;

                    }

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

    void KukieHand::safelyDestroy() {

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

        if(positions.n_elem != currJoints.n_elem)
            throw KukaduException("(KukieHand) wrong joint dimension");

        auto ignoreJoint = SDH_IGNORE_JOINT;
        std_msgs::Float64MultiArray newJoints;
        for(int i = 0; i < positions.n_elem; ++i) {
            if(positions(i) != ignoreJoint)
                newJoints.data.push_back(positions(i));
            else {
                newJoints.data.push_back(currJoints(i));
                positions(i) = currJoints(i);
            }
        }

        targetReached = false;
        movementStarted = false;

        initCurrentPos = currentPos;
        trajPub.publish(newJoints);

        if(waitForReached) {

            currJoints = getCurrentJoints();
            targetReached = !vectorsDeviate(armadilloToStdVec(currJoints), armadilloToStdVec(positions), 0.05);

            TicToc tic;

            tic.tic("hand_timeout");

            double elapsedTime = tic.toc("hand_timeout");
            bool stillMoving = false;

            while(!targetReached && !(elapsedTime > 1.0 && !stillMoving)) {

                stillMoving = false;

                // check if the fingers are still moving
                currentPosMutex.lock();
                    auto posQueueTmp = previousCurrentPosQueue;
                currentPosMutex.unlock();

                while(!posQueueTmp.empty()) {

                    auto& previousCurrentPos = posQueueTmp.front();
                    bool deviates = vectorsDeviate(previousCurrentPos, armadilloToStdVec(currJoints), 0.01);
                    if(deviates) {
                        stillMoving = true;
                        break;
                    }
                    currJoints = getCurrentJoints();
                    posQueueTmp.pop();

                }

                // if not moving anymore but the movement was started --> the target is reached
                if(!movementStarted && stillMoving)
                    movementStarted = true;
                else if(movementStarted && !stillMoving)
                    targetReached = true;

                elapsedTime = tic.toc("hand_timeout");

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
