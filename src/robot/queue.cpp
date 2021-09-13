#include <chrono>
#include <memory>
#include <sstream>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <kukadu/utils/utils.hpp>
#include <kukadu/robot/queue.hpp>
#include <kukadu/planning/komo.hpp>
#include <kukadu/robot/kukiequeue.hpp>
#include <kukadu/types/kukadutypes.hpp>
#include <kukadu/storage/sensorstorage.hpp>
#include <iis_robot_dep/FriJointImpedance.h>
#include <iis_robot_dep/CartesianImpedance.h>
#include <kukadu/storage/moduleusagesingleton.hpp>

using namespace std;
using namespace arma;
using namespace chrono;

namespace kukadu {

    int ControlQueue::getRobotId() {

        KUKADU_MODULE_START_USAGE();

        auto robotId = getHardwareInstance();

        KUKADU_MODULE_END_USAGE();

        return robotId;

    }

    void ControlQueue::installHardwareTypeInternal() {
        // nothing special to do here yet
    }

    void ControlQueue::installHardwareInstanceInternal() {

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

    double ControlQueue::getPreferredPollingFrequency() {
        return 1.0 / getCycleTime();
    }

    vector<std::pair<long long int, arma::vec> > ControlQueue::loadData(long long int startTime, long long int endTime, long long maxTotalDuration, long long int maxTimeStepDifference) {

        static bool referenceWarningShown = false;

        auto jointData = JointHardware::loadData(startTime, endTime, maxTotalDuration, maxTimeStepDifference);

        auto& storage = getStorage();
        auto robotId = getHardwareInstance();

        if(!referenceWarningShown) {
            cerr << "(ControlQueue) warning: reference frame and link name not considered yet" << endl;
            referenceWarningShown = true;
        }

        stringstream s;
        s << "select time_stamp, cart_pos_x, cart_pos_y, cart_pos_z, cart_rot_x, cart_rot_y, cart_rot_z" <<
             ", cart_frc_x, cart_frc_y, cart_frc_z, cart_trq_x, cart_trq_y, cart_trq_z, cart_abs_frc " <<
             " from cart_mes as cm inner join cart_mes_pos as cmp on cm.cart_mes_id = cmp.cart_mes_id" <<
             " inner join cart_mes_frc as cmf on cm.cart_mes_id = cmf.cart_mes_id " <<
             " where time_stamp >= " << startTime;
        if(endTime > 0)
            s << " and time_stamp <= " << endTime;
        else
            s << " and time_stamp <= " << (startTime + maxTotalDuration);
        s << " and robot_id = " << robotId <<
             " order by time_stamp asc";

        vec lastCartInformation(13);
        lastCartInformation.fill(0.0);
        auto queryRes = storage.executeQuery(s.str());
        int i = 0;
        while(queryRes->next() && i < jointData.size()) {

            long long int currentTime = queryRes->getInt64("time_stamp");

            auto& jd = jointData.at(i);
            long long int jointTime = jd.first;

            while(currentTime > jointTime && i < jointData.size()) {

                auto& jd = jointData.at(i);
                jointTime = jd.first;

                vec cartInformation(13);
                cartInformation(0) = queryRes->getDouble("cart_pos_x");
                cartInformation(1) = queryRes->getDouble("cart_pos_y");
                cartInformation(2) = queryRes->getDouble("cart_pos_z");
                cartInformation(3) = queryRes->getDouble("cart_rot_x");
                cartInformation(4) = queryRes->getDouble("cart_rot_y");
                cartInformation(5) = queryRes->getDouble("cart_rot_z");
                cartInformation(6) = queryRes->getDouble("cart_frc_x");
                cartInformation(7) = queryRes->getDouble("cart_frc_y");
                cartInformation(8) = queryRes->getDouble("cart_frc_z");
                cartInformation(9) = queryRes->getDouble("cart_trq_x");
                cartInformation(10) = queryRes->getDouble("cart_trq_y");
                cartInformation(11) = queryRes->getDouble("cart_trq_z");
                cartInformation(12) = queryRes->getDouble("cart_abs_frc");

                lastCartInformation = cartInformation;

                jd.second = join_cols(jd.second, cartInformation);
                ++i;

            }

        }

        for(; i < jointData.size(); ++i) {
            auto& jd = jointData.at(i);
            jd.second = join_cols(jd.second, lastCartInformation);
        }

        return jointData;

    }

    void ControlQueue::storeCurrentSensorDataToDatabase() {

        auto& storage = getStorage();
        auto robotId = getRobotId();
        auto jointIds = getJointIds();
        auto referenceFrame = getCartesianReferenceFrame();
        auto linkName = getCartesianLinkName();
        prevPrevJoints = prevJoints;
        prevJoints = nowJoints;
        nowJoints = getCurrentJoints();
        auto jntFrcTrq = getCurrentJntFrc();
        auto cartPose = getCurrentCartesianPose();
        auto cartFrcTrq = getCurrentCartesianFrcTrq();
        auto time = getCurrentTime();
        auto absCartFrc = getAbsoluteCartForce();

        auto storeCartPos = true;
        auto storeCartFrcTrq = true;
        auto storeCartAbsFrc = true;

        vec vel;
        vec acc;
        // if the previous times are the same, it is the first time - acceleration are 0
        if(prevPrevJoints.time == prevJoints.time)
            acc = arma::zeros(nowJoints.joints.n_elem);
        else {
            auto timeDiffInSec = (double) (prevJoints.time - prevPrevJoints.time) / 1000.0;
            acc = (prevPrevJoints.joints - prevJoints.joints) / timeDiffInSec;
        }

        // same reasoning vor velocity
        if(prevJoints.time == nowJoints.time)
            vel = arma::zeros(nowJoints.joints.n_elem);
        else {
            auto timeDiffInSec = (double) (nowJoints.time - prevJoints.time) / 1000.0;
            vel = (nowJoints.joints - prevJoints.joints) / timeDiffInSec;
        }

        // store the data in the database
        SensorStorage::storeJointInfoToDatabase(storage, robotId, time, jointIds, nowJoints.joints, vel, acc, true, jntFrcTrq.joints);
        SensorStorage::storeCartInformation(storage, robotId, time, referenceFrame, linkName, cartPose, cartFrcTrq.joints, absCartFrc, storeCartPos, storeCartFrcTrq, storeCartAbsFrc);

    }

    int ControlQueue::getJointId(std::string jointName) {

        stringstream s;
        s << "select joint_id from hardware_joints where hardware_instance_id = " << getHardwareInstance() << " and joint_name = \"" << jointName << "\"";
        auto idRes = getStorage().executeQuery(s.str());
        if(idRes->next())
            return idRes->getInt("joint_id");
        else
            throw KukaduException("(ControlQueue) searched joint is not part of the robot");

    }

    std::vector<int> ControlQueue::getJointIds() {
        return getJointIds(getJointNames());
    }

    std::vector<int> ControlQueue::getJointIds(std::vector<std::string> jointNames) {
        vector<int> jointIds;
        for(int i = 0; i < jointNames.size(); ++i)
            jointIds.push_back(getJointId(jointNames.at(i)));
        return jointIds;
    }

    KUKADU_SHARED_PTR<kukadu_thread> ControlQueue::startQueue() {

        KUKADU_MODULE_START_USAGE();

        setInitValues();

        frcTrqFilterUpdateThr = make_shared<kukadu_thread>(&ControlQueue::frcTrqFilterUpdateHandler, this);
        thr = make_shared<kukadu_thread>(&ControlQueue::run, this);

        while(!this->isInitialized());
        startQueueHook();

        nowJoints = prevJoints = prevPrevJoints = getCurrentJoints();

        KUKADU_MODULE_END_USAGE();

        return thr;

    }

    void ControlQueue::startInternal(){
        startQueue();
        if(getCurrentMode() != kukadu::KukieControlQueue::KUKA_JNT_IMP_MODE) {
            stopCurrentMode();
            switchMode(kukadu::KukieControlQueue::KUKA_JNT_IMP_MODE);
        }
    }

    void ControlQueue::stopInternal() {
        stopQueue();
    }

    ControlQueue::ControlQueue(StorageSingleton& storage, std::string robotName, std::string className) :
        JointHardware(storage, HARDWARE_ARM, loadTypeIdFromName(className), className, loadInstanceIdFromName(robotName), robotName) {

        // load cycle time and degrees of freedom

        thr = nullptr;
        frcTrqFilterUpdateThr = nullptr;
        cartPtpThr = nullptr;
        jointPtpThr = nullptr;
        jointsColletorThr = nullptr;

        this->robotName = robotName;
        this->sleepTime = desiredCycleTime;

        jointPtpRunning = false;
        cartesianPtpRunning = false;
        frcTrqFilterRunning = true;
        currentTime = getCurrentTime();
        continueCollecting = false;
        currentFrcTrqSensorFilter = make_shared<StandardFilter>();

    }

    ControlQueue::ControlQueue(StorageSingleton& storage, std::string robotName, std::string className, int degreesOfFreedom, double desiredCycleTime) :
        JointHardware(storage, HARDWARE_ARM, loadOrCreateTypeIdFromName(className), className, loadOrCreateInstanceIdFromName(robotName), robotName) {

        thr = nullptr;
        frcTrqFilterUpdateThr = nullptr;
        cartPtpThr = nullptr;
        jointPtpThr = nullptr;
        jointsColletorThr = nullptr;

        this->robotName = robotName;
        this->desiredCycleTime = desiredCycleTime;
        this->sleepTime = desiredCycleTime;

        jointPtpRunning = false;
        cartesianPtpRunning = false;
        frcTrqFilterRunning = true;
        currentTime = getCurrentTime();
        degOfFreedom = degreesOfFreedom;
        continueCollecting = false;
        currentFrcTrqSensorFilter = make_shared<StandardFilter>();

    }

    void ControlQueue::setFrcTrqSensorFilter(KUKADU_SHARED_PTR<FrcTrqSensorFilter> myFilter) {
        KUKADU_MODULE_START_USAGE();
        currentFrcTrqSensorFilter = myFilter;
        KUKADU_MODULE_END_USAGE();
    }

    mes_result ControlQueue::getCurrentProcessedCartesianFrcTrq() {
        KUKADU_MODULE_START_USAGE();
        auto reading = currentFrcTrqSensorFilter->getProcessedReading();
        KUKADU_MODULE_END_USAGE();
        return reading;
    }

    void ControlQueue::setDegOfFreedom(int degOfFreedom) {
        this->degOfFreedom = degOfFreedom;
    }

    void ControlQueue::setCycleTime(double cycleTime) {
        KUKADU_MODULE_START_USAGE();
        loadCycleTimeMutex.lock();
        this->sleepTime = cycleTime;
        this->desiredCycleTime = cycleTime;
        loadCycleTimeMutex.unlock();
        KUKADU_MODULE_END_USAGE();
    }

    double ControlQueue::getAbsoluteCartForce() {
        KUKADU_MODULE_START_USAGE();
        mes_result m = getCurrentCartesianFrcTrq();
        vec forces = m.joints.subvec(0, 2);
        vec prod = forces.t() * forces;
        KUKADU_MODULE_END_USAGE();
        return sqrt(prod(0));

    }

    double ControlQueue::getCycleTime() {
        KUKADU_MODULE_START_USAGE();
        loadCycleTimeMutex.lock();
        auto cycleTmp = desiredCycleTime;
        loadCycleTimeMutex.unlock();
        KUKADU_MODULE_END_USAGE();
        return cycleTmp;
    }

    arma::vec ControlQueue::getStartingJoints() {
        return startingJoints;
    }

    void ControlQueue::setStartingJoints(arma::vec joints) {
        startingJoints = joints;
    }

    void ControlQueue::shutUp() {
        isShutUpFlag = true;
    }

    void ControlQueue::startTalking() {
        isShutUpFlag = false;
    }

    bool ControlQueue::isShutUp() {
        return isShutUpFlag;
    }

    void ControlQueue::setNextTrajectory(std::vector<arma::vec> jointTrajectory) {
        KUKADU_MODULE_START_USAGE();
        for(auto joint : jointTrajectory)
            move(joint);
        synchronizeToQueue(1);
        KUKADU_MODULE_END_USAGE();
    }

    void ControlQueue::move(geometry_msgs::Pose pose) {
        KUKADU_MODULE_START_USAGE();
        cartesianMovementQueue.push(pose);
        KUKADU_MODULE_END_USAGE();
    }

    void ControlQueue::switchMode(int mode) {

        KUKADU_MODULE_START_USAGE();

        if(ros::ok) {

            if(!isShutUp())
                ROS_INFO("(ControlQueue) switching control mode");

            setCurrentControlTypeInternal(mode);
            currentControlType = mode;

        } else
            if(!isShutUp())
                ROS_INFO("(ControlQueue) ros error");

        KUKADU_MODULE_END_USAGE();

    }

    void ControlQueue::setInitValuesInternal() {

        isInit = false;
        finish = 0;

        currentTime = getCurrentTime();

        rollbackMode = false;
        rollBackQueueSize = 0;

        currentJoints = arma::vec(1);

        while(!movementQueue.empty()) movementQueue.pop();
        while(!cartesianMovementQueue.empty()) cartesianMovementQueue.pop();

        setInitValues();

    }

    void ControlQueue::stopQueue() {

        KUKADU_MODULE_START_USAGE();

        finish = 1;
        continueCollecting = false;
        frcTrqFilterRunning = false;
        startingJoints = arma::vec(1);

        if(thr && thr->joinable())
            thr->join();

        if(frcTrqFilterUpdateThr && frcTrqFilterUpdateThr->joinable())
            frcTrqFilterUpdateThr->join();

        if(jointsColletorThr && jointsColletorThr->joinable())
            jointsColletorThr->join();

        KUKADU_MODULE_END_USAGE();

    }

    bool ControlQueue::isInitialized() {
        return isInit;
    }

    void ControlQueue::move(arma::vec joints) {
        KUKADU_MODULE_START_USAGE();
        movementQueue.push(joints);
        KUKADU_MODULE_END_USAGE();
    }

    mes_result ControlQueue::getCurrentCartesianPos() {

        KUKADU_MODULE_START_USAGE();

        mes_result ret;
        ret.joints = vec(7);
        ret.joints(0) = currentCartPose.position.x;
        ret.joints(1) = currentCartPose.position.y;
        ret.joints(2) = currentCartPose.position.z;
        ret.joints(3) = currentCartPose.orientation.x;
        ret.joints(4) = currentCartPose.orientation.y;
        ret.joints(5) = currentCartPose.orientation.z;
        ret.joints(6) = currentCartPose.orientation.w;
        ret.time = getCurrentTime();

        KUKADU_MODULE_END_USAGE();

        return ret;

    }

    bool ControlQueue::getQueueRunning() {
        return !finish;
    }

    void ControlQueue::synchronizeToQueue(int maxNumJointsInQueue) {
        KUKADU_MODULE_START_USAGE();
        ros::Rate r(1.0 / sleepTime);
        if(currentControlType == CONTROLQUEUE_JNT_IMP_MODE || currentControlType == CONTROLQUEUE_JNT_POS_MODE) {
            while(movementQueue.size() > maxNumJointsInQueue) {
                r.sleep();
            }
        } else if(currentControlType == CONTROLQUEUE_CART_IMP_MODE) {
            while(cartesianMovementQueue.size() > maxNumJointsInQueue) {
                r.sleep();
            }
        }
        KUKADU_MODULE_END_USAGE();
    }

    int ControlQueue::getQueueSize() {
        return movementQueue.size();
    }

    void ControlQueue::stopCurrentMode() {
        KUKADU_MODULE_START_USAGE();
        switchMode(CONTROLQUEUE_STOP_MODE);
        switchMode(CONTROLQUEUE_JNT_POS_MODE);
        switchMode(CONTROLQUEUE_STOP_MODE);
        KUKADU_MODULE_END_USAGE();
    }

    void ControlQueue::internalJointPtpCaller() {
        jointPtp(internalJointPasser);
    }

    KUKADU_SHARED_PTR<kukadu_thread> ControlQueue::jointPtpNb(arma::vec joints) {

        KUKADU_MODULE_START_USAGE();

        internalJointPasser = joints;
        jointPtpThr = KUKADU_SHARED_PTR<kukadu_thread>(new kukadu_thread(&ControlQueue::internalJointPtpCaller, this));

        KUKADU_MODULE_END_USAGE();

        return jointPtpThr;

    }

    void ControlQueue::internalCartPtpCaller() {
        cartesianPtp(internalPosePasser);
    }

    KUKADU_SHARED_PTR<kukadu_thread> ControlQueue::cartesianPtpNb(geometry_msgs::Pose pos) {

        KUKADU_MODULE_START_USAGE();

        internalPosePasser = pos;
        cartPtpThr = KUKADU_SHARED_PTR<kukadu_thread>(new kukadu_thread(&ControlQueue::internalJointPtpCaller, this));

        KUKADU_MODULE_END_USAGE();

        return jointPtpThr;

    }

    void ControlQueue::run() {

        setInitValuesInternal();
        ros::Rate sleepRate(1.0 / sleepTime);

        arma::vec movement = arma::vec(1);
        geometry_msgs::Pose movementPose;

        if(getStartingJoints().n_elem > 1) {

            if(!isShutUp())
                ROS_INFO("(ControlQueue) start moving to start position");
            jointPtp(getStartingJoints());

            if(!isShutUp())
                ROS_INFO("(ControlQueue) finished moving to start position");

        }

        isInit = true;

        lastDuration = 0.0;
        movement = getCurrentJoints().joints;

        while(!finish && ros::ok) {

            currentJoints = getCurrentJoints().joints;
            currentCartPose = getCurrentCartesianPose();

            if(rollbackMode) {

                rollbackQueueMutex.lock();
                    rollBackQueue.push_front(currentJoints);

                    // if queue is full --> go back
                    while(rollBackQueue.size() > rollBackQueueSize)
                        rollBackQueue.pop_back();
                rollbackQueueMutex.unlock();

            }

            if(!stopQueueWhilePtp() || !jointPtpRunning && !cartesianPtpRunning) {

                if(currentControlType == CONTROLQUEUE_JNT_IMP_MODE || currentControlType == CONTROLQUEUE_JNT_POS_MODE) {

                    if(movementQueue.size() > 0) {

                        // move to position in queue
                        movement = movementQueue.front();
                        movementQueue.pop();

                    } else {

                        //movement = getCurrentJoints().joints;

                    }

                    if(!stopQueueWhilePtp() || stopQueueWhilePtp() && jointPtpRunning)
                        submitNextJointMove(movement);

                } else if(currentControlType == CONTROLQUEUE_CART_IMP_MODE) {

                    if(cartesianMovementQueue.size() > 0) {

                        // move to position in queue
                        movementPose = cartesianMovementQueue.front();
                        cartesianMovementQueue.pop();

                    } else {

                        movementPose = getCurrentCartesianPose();

                    }

                    submitNextCartMove(movementPose);

                }

            }

            sleepRate.sleep();

        }

        if(!isShutUp())
            cout << "thread finished" << endl;

    }

    void ControlQueue::jointsCollector() {

        double lastTime = DBL_MIN;
        collectedJoints.clear();
        ros::Rate r((int) (1.0 / getCycleTime() + 10.0));
        while(continueCollecting) {
            mes_result lastResult = getCurrentJoints();
            if(lastResult.time != lastTime)
                collectedJoints.push_back(lastResult);
            r.sleep();
        }

    }

    std::vector<mes_result> ControlQueue::jointPtp(arma::vec joints, double maxForce) {

        KUKADU_MODULE_START_USAGE();

        jointPtpRunning = true;

        if(!continueCollecting) {

            continueCollecting = true;
            jointsColletorThr = make_shared<kukadu_thread>(&ControlQueue::jointsCollector, this);
            jointPtpInternal(joints, maxForce);
            continueCollecting = false;

            if(jointsColletorThr && jointsColletorThr->joinable())
                jointsColletorThr->join();

        } else {

            throw KukaduException("(ControlQueue) only one ptp at a time can be executed");

        }

        jointPtpRunning = false;

        KUKADU_MODULE_END_USAGE();

        return collectedJoints;

    }

    std::vector<mes_result> ControlQueue::cartesianPtp(geometry_msgs::Pose pos, double maxForce) {

        KUKADU_MODULE_START_USAGE();

        cartesianPtpRunning = true;

        if(!continueCollecting) {

            continueCollecting = true;
            jointsColletorThr = KUKADU_SHARED_PTR<kukadu_thread>(new kukadu_thread(&ControlQueue::jointsCollector, this));

            try {
                cartPtpInternal(pos, maxForce);
            } catch(KukaduException& ex) {
                continueCollecting = false;

                if(jointsColletorThr && jointsColletorThr->joinable())
                    jointsColletorThr->join();

                throw ex;
            }

            continueCollecting = false;

            if(jointsColletorThr && jointsColletorThr->joinable())
                jointsColletorThr->join();

        } else {

            throw KukaduException("(ControlQueue) only one ptp at a time can be executed");

        }

        cartesianPtpRunning = false;

        KUKADU_MODULE_END_USAGE();

        return collectedJoints;

    }

    std::vector<mes_result> ControlQueue::cartesianPtp(geometry_msgs::Pose pos) {

        KUKADU_MODULE_START_USAGE();

        auto retVal = cartesianPtp(pos, DBL_MAX);

        KUKADU_MODULE_END_USAGE();

        return retVal;

    }

    void ControlQueue::startRollBackMode(double possibleTime) {

        KUKADU_MODULE_START_USAGE();

        rollbackQueueMutex.lock();
            rollBackQueue.clear();
        rollbackQueueMutex.unlock();
        rollBackTime = possibleTime;
        rollbackMode = true;
        // buffer of 1.0 more second
        rollBackQueueSize = (int) ((possibleTime + 1.0) / getCycleTime());

        KUKADU_MODULE_END_USAGE();

    }

    void ControlQueue::stopJointRollBackMode() {

        KUKADU_MODULE_START_USAGE();

        rollbackMode = false;
        rollbackQueueMutex.lock();
            rollBackQueue.clear();
        rollbackQueueMutex.unlock();

        KUKADU_MODULE_END_USAGE();

    }

    void ControlQueue::rollBack(double time) {

        KUKADU_MODULE_START_USAGE();

        rollbackMode = false;
        int rollBackCount = (int) (time / getCycleTime());

        int stretchFactor = ceil((double) rollBackCount / (double) rollBackQueue.size());
        stretchFactor = max((double) stretchFactor, 1.0);

        vec lastCommand(getDegreesOfFreedom());
        rollbackQueueMutex.lock();
        if(rollBackQueue.size())
            lastCommand = rollBackQueue.front();
        rollbackQueueMutex.unlock();

        int newRollBackCount = ceil((double) rollBackCount / (double) stretchFactor);

        // fill command queue with last commands (backwards)
        for(int i = 0; i < newRollBackCount && rollBackQueue.size(); ++i) {

            rollbackQueueMutex.lock();
                vec nextCommand;
                if(rollBackQueue.size())
                    nextCommand = rollBackQueue.front();
                else {
                    rollbackQueueMutex.unlock();
                    break;
                }
            rollbackQueueMutex.unlock();

            // interpolate to stretch the trajectory in case there are not enough measured packets (happens in usage with simulator)
            vec diffUnit = (nextCommand - lastCommand) / (double) stretchFactor;
            for(int j = 0; j < stretchFactor; ++j) {
                move(lastCommand + j * diffUnit);
            }

            lastCommand = nextCommand;
            rollbackQueueMutex.lock();
                if(rollBackQueue.size())
                    rollBackQueue.pop_front();
            rollbackQueueMutex.unlock();

        }

        // wait until everything has been executed
        synchronizeToQueue(1);

        rollbackQueueMutex.lock();
            rollBackQueue.clear();
        rollbackQueueMutex.unlock();

        KUKADU_MODULE_END_USAGE();

    }

    void ControlQueue::frcTrqFilterUpdateHandler(){
        ros::Rate myRate(50);
        while(frcTrqFilterRunning){
            currentFrcTrqSensorFilter->updateFilter(getCurrentCartesianFrcTrq());
            myRate.sleep();
        }
    }

    void KukieControlQueue::installHardwareInstanceInternal() {

        StorageSingleton& storage = StorageSingleton::get();

        auto degOfFreedom = getDegreesOfFreedom();
        auto frequency = 1.0 / getCycleTime();
        auto prefix = armPrefix;
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

    std::tuple<int, double, std::string> KukieControlQueue::loadDbInfo(std::string hardwareName) {

        stringstream s;
        s << "select deg_of_freedom, frequency, name_prefix from kukie_hardware as kh inner join hardware_instances as hi on " <<
             "hi.instance_id = kh.hardware_instance_id where instance_name = '" << hardwareName << "'";
        StorageSingleton& storage = StorageSingleton::get();
        auto result = storage.executeQuery(s.str());

        int degOfFreedom = 0;
        double frequency = 70.0;
        string armPrefix = "";
        if(result->next()) {
            degOfFreedom = result->getInt("deg_of_freedom");
            frequency = result->getDouble("frequency");
            armPrefix = result->getString("name_prefix");
        } else
            throw KukaduException("Cannot create KukieQueue from database. It first has to be installed.");

        return std::tuple<int, double, std::string>(degOfFreedom, frequency, armPrefix);

    }

    KukieControlQueue::KukieControlQueue(StorageSingleton& storage, std::string hardwareName, bool simulation, bool acceptCollisions) :
        degFreedom(get<0>(loadDbInfo(hardwareName))),
        ControlQueue(storage, "kukie_" + get<2>(loadDbInfo(hardwareName)), "KukieControlQueue", degFreedom, get<1>(loadDbInfo(hardwareName))) {

        deviceType = (simulation) ? "simulation" : "real";
        armPrefix = get<2>(loadDbInfo(hardwareName));

        double sleepTime = 1.0 / get<1>(loadDbInfo(hardwareName));

        retJointPosTopic = deviceType + "/" + armPrefix + "/joint_control/get_state";
        commandTopic = deviceType + "/" + armPrefix + "/joint_control/move";
        switchModeTopic = deviceType + "/" + armPrefix + "/settings/switch_mode";
        retCartPosTopic = deviceType + "/" + armPrefix + "/cartesian_control/get_pose_quat_wf";
        stiffnessTopic = deviceType + "/" + armPrefix + "/cartesian_control/set_impedance";
        jntStiffnessTopic = deviceType + "/" + armPrefix + "/joint_control/set_impedance";
        ptpTopic = deviceType + "/" + armPrefix + "/joint_control/ptp";
        commandStateTopic = deviceType + "/" + armPrefix + "/settings/get_command_state";
        ptpReachedTopic = deviceType + "/" + armPrefix + "/joint_control/ptp_reached";
        jntFrcTrqTopic = deviceType + "/" + armPrefix + "/sensoring/est_ext_jnt_trq";
        cartFrcTrqTopic = deviceType + "/" + armPrefix + "/sensoring/cartesian_wrench";
        cartPtpTopic = deviceType + "/" + armPrefix + "/cartesian_control/ptpQuaternion";
        cartPtpReachedTopic = deviceType + "/" + armPrefix + "/cartesian_control/ptp_reached";
        cartMoveRfQueueTopic = deviceType + "/" + armPrefix + "/cartesian_control/move_rf";
        cartMoveWfQueueTopic = deviceType + "/" + armPrefix + "/cartesian_control/move_wf";
        cartPoseRfTopic = deviceType + "/" + armPrefix + "/cartesian_control/get_pose_quat_rf";
        jntSetPtpThreshTopic = deviceType + "/" + armPrefix + "/joint_control/set_ptp_thresh";
        clockCycleTopic = deviceType + "/" + armPrefix + "/settings/get_clock_cycle";
        maxDistPerCycleTopic = deviceType + "/" + armPrefix + "/joint_control/get_max_dist_per_cycle";
        addLoadTopic = "not supported yet";

        constructQueue(commandTopic, retJointPosTopic, switchModeTopic, retCartPosTopic, stiffnessTopic,
                       jntStiffnessTopic, ptpTopic, commandStateTopic, ptpReachedTopic, addLoadTopic, jntFrcTrqTopic, cartFrcTrqTopic,
                       cartPtpTopic, cartPtpReachedTopic, cartMoveRfQueueTopic, cartMoveWfQueueTopic, cartPoseRfTopic, jntSetPtpThreshTopic,
                       clockCycleTopic, maxDistPerCycleTopic,
                       acceptCollisions, node,
                       kin, planner,
                       sleepTime, maxDistPerCycle);

    }

    KukieControlQueue::KukieControlQueue(StorageSingleton& storage, std::string deviceType, std::string armPrefix, ros::NodeHandle node, bool acceptCollisions, KUKADU_SHARED_PTR<Kinematics> kin, KUKADU_SHARED_PTR<PathPlanner> planner, double sleepTime, double maxDistPerCycle) :
        degFreedom(loadDegOfFreedom(node, "/" + deviceType + "/" + armPrefix + "/joint_control/get_state")),
        ControlQueue(storage, "kukie_" + armPrefix, "KukieControlQueue", degFreedom, sleepTime) {

        retJointPosTopic = deviceType + "/" + armPrefix + "/joint_control/get_state";
        commandTopic = deviceType + "/" + armPrefix + "/joint_control/move";
        switchModeTopic = deviceType + "/" + armPrefix + "/settings/switch_mode";
        retCartPosTopic = deviceType + "/" + armPrefix + "/cartesian_control/get_pose_quat_wf";
        stiffnessTopic = deviceType + "/" + armPrefix + "/cartesian_control/set_impedance";
        jntStiffnessTopic = deviceType + "/" + armPrefix + "/joint_control/set_impedance";
        ptpTopic = deviceType + "/" + armPrefix + "/joint_control/ptp";
        commandStateTopic = deviceType + "/" + armPrefix + "/settings/get_command_state";
        ptpReachedTopic = deviceType + "/" + armPrefix + "/joint_control/ptp_reached";
        jntFrcTrqTopic = deviceType + "/" + armPrefix + "/sensoring/est_ext_jnt_trq";
        cartFrcTrqTopic = deviceType + "/" + armPrefix + "/sensoring/cartesian_wrench";
        cartPtpTopic = deviceType + "/" + armPrefix + "/cartesian_control/ptpQuaternion";
        cartPtpReachedTopic = deviceType + "/" + armPrefix + "/cartesian_control/ptp_reached";
        cartMoveRfQueueTopic = deviceType + "/" + armPrefix + "/cartesian_control/move_rf";
        cartMoveWfQueueTopic = deviceType + "/" + armPrefix + "/cartesian_control/move_wf";
        cartPoseRfTopic = deviceType + "/" + armPrefix + "/cartesian_control/get_pose_quat_rf";
        jntSetPtpThreshTopic = deviceType + "/" + armPrefix + "/joint_control/set_ptp_thresh";
        clockCycleTopic = deviceType + "/" + armPrefix + "/settings/get_clock_cycle";
        maxDistPerCycleTopic = deviceType + "/" + armPrefix + "/joint_control/get_max_dist_per_cycle";
        addLoadTopic = "not supported yet";

        this->deviceType = deviceType;
        this->armPrefix = armPrefix;

        constructQueue(commandTopic, retJointPosTopic, switchModeTopic, retCartPosTopic, stiffnessTopic,
                       jntStiffnessTopic, ptpTopic, commandStateTopic, ptpReachedTopic, addLoadTopic, jntFrcTrqTopic, cartFrcTrqTopic,
                       cartPtpTopic, cartPtpReachedTopic, cartMoveRfQueueTopic, cartMoveWfQueueTopic, cartPoseRfTopic, jntSetPtpThreshTopic,
                       clockCycleTopic, maxDistPerCycleTopic,
                       acceptCollisions, node,
                       kin, planner,
                       sleepTime, maxDistPerCycle);

    }

    int KukieControlQueue::getDegreesOfFreedom() {
        return degFreedom;
    }

    void KukieControlQueue::constructQueue(std::string commandTopic, std::string retPosTopic, std::string switchModeTopic, std::string retCartPosTopic,
                            std::string cartStiffnessTopic, std::string jntStiffnessTopic, std::string ptpTopic,
                            std::string commandStateTopic, std::string ptpReachedTopic, std::string addLoadTopic, std::string jntFrcTrqTopic, std::string cartFrcTrqTopic,
                            std::string cartPtpTopic, std::string cartPtpReachedTopic, std::string cartMoveRfQueueTopic, std::string cartMoveWfQueueTopic, std::string cartPoseRfTopic,
                            std::string jntSetPtpThreshTopic, string clockCycleTopic, string maxDistancePerCycleTopic,
                            bool acceptCollisions, ros::NodeHandle node,
                            KUKADU_SHARED_PTR<Kinematics> kin, KUKADU_SHARED_PTR<PathPlanner> planner,
                            double sleepTime, double maxDistPerCycle
                        ) {



        set_ctrlc_exit_handler();

        firstCartsComputed = false;
        this->acceptCollisions = acceptCollisions;

        this->ptpTopic = ptpTopic;
        this->addLoadTopic = addLoadTopic;
        this->commandTopic = commandTopic;
        this->cartPtpTopic = cartPtpTopic;
        this->retJointPosTopic = retPosTopic;
        this->jntFrcTrqTopic = jntFrcTrqTopic;
        this->switchModeTopic = switchModeTopic;
        this->retCartPosTopic = retCartPosTopic;
        this->ptpReachedTopic = ptpReachedTopic;
        this->cartFrcTrqTopic = cartFrcTrqTopic;
        this->stiffnessTopic = cartStiffnessTopic;
        this->jntStiffnessTopic = jntStiffnessTopic;
        this->commandStateTopic = commandStateTopic;
        this->cartPtpReachedTopic = cartPtpReachedTopic;
        this->cartMoveRfQueueTopic = cartMoveRfQueueTopic;
        this->cartMoveWfQueueTopic = cartMoveWfQueueTopic;
        this->jntSetPtpThreshTopic = jntSetPtpThreshTopic;

        cartesianPtpReached = 0;

        kin = nullptr;
        planner = nullptr;

        setInitValues();
        this->node = node;

        if(sleepTime == 0.0) {
            ROS_ERROR("(KukieControlQueue) the sleep time you provided is 0. note that it is required in seconds");
            throw KukaduException("(KukieControlQueue) the sleep time you provided is 0. note that it is required in seconds");
        }

        subJntPos = node.subscribe(retPosTopic, 2, &KukieControlQueue::robotJointPosCallback, this);
        subComState = node.subscribe(commandStateTopic, 2, &KukieControlQueue::commandStateCallback, this);
        subPtpReached = node.subscribe(ptpReachedTopic, 2, &KukieControlQueue::ptpReachedCallback, this);
        subjntFrcTrq = node.subscribe(jntFrcTrqTopic, 2, &KukieControlQueue::jntFrcTrqCallback, this);
        subCartFrqTrq = node.subscribe(cartFrcTrqTopic, 2, &KukieControlQueue::cartFrcTrqCallback, this);
        subCartPtpReached = node.subscribe(cartPtpReachedTopic, 2, &KukieControlQueue::cartPtpReachedCallback, this);
        subCartPoseRf = node.subscribe(cartPoseRfTopic, 2, &KukieControlQueue::cartPosRfCallback, this);
        subMaxDistPerCycle = node.subscribe(maxDistancePerCycleTopic, 2, &KukieControlQueue::maxDistPerCycleCallback, this);
        subCycleTime = node.subscribe(clockCycleTopic, 2, &KukieControlQueue::cycleTimeCallback, this);

        pub_set_cart_stiffness = node.advertise<iis_robot_dep::CartesianImpedance>(stiffnessTopic, 1);
        pub_set_joint_stiffness = node.advertise<iis_robot_dep::FriJointImpedance>(jntStiffnessTopic, 1);
        pubCartPtp = node.advertise<geometry_msgs::Pose>(cartPtpTopic, 1);
        pubCartMoveRfQueue = node.advertise<geometry_msgs::Pose>(cartMoveRfQueueTopic, 1);
        pubCartMoveWfQueue = node.advertise<geometry_msgs::Pose>(cartMoveWfQueueTopic, 1);
        pub_set_ptp_thresh = node.advertise<std_msgs::Float64>(jntSetPtpThreshTopic, 1);

        pubCommand = node.advertise<std_msgs::Float64MultiArray>(commandTopic, 10);
        pubSwitchMode = node.advertise<std_msgs::Int32>(switchModeTopic, 1);
        pubPtp = node.advertise<std_msgs::Float64MultiArray>(ptpTopic, 10);

        isRealRobot = (getRobotDeviceType().compare("real")) ? false : true;

        // this is required because shared_from_this can't be called in constructor (initializiation happens by lazy loading)
        if(kin) {
            kinematicsInitialized = true;
            this->kin = kin;
        } else kinematicsInitialized = false;

        if(planner) {
            plannerInitialized = true;
            this->planner = planner;
        } else plannerInitialized = false;

        ros::Rate r(5);
        if(maxDistPerCycle < 0.0) {
            loadMaxDistPerCycleFromServer = true;
            while(!firstMaxDistPerCycleReceived)
                r.sleep();
        }

        if(sleepTime < 0.0) {
            loadCycleTimeFromServer = true;
            while(!firstControllerCycletimeReceived)
                r.sleep();
            sleepTime = getCycleTime();
        }

        while(!firstJointsReceived || !firstModeReceived)
            r.sleep();

        setDegOfFreedom(getCurrentJoints().joints.n_elem);

        loop_rate = make_shared<ros::Rate>(1.0 / sleepTime);
        setCycleTime(sleepTime);

        currentControlType = impMode;

        currentCartFrqTrq = vec(6);
        currentCartFrqTrq.fill(0.0);

        currentJntFrqTrq = vec(getDegreesOfFreedom());
        currentJntFrqTrq.fill(0.0);

        setStiffness(KUKA_STD_XYZ_STIFF, KUKA_STD_ABC_STIFF, KUKA_STD_CPDAMPING, KUKA_STD_CPMAXDELTA, KUKA_STD_MAXFRC, KUKA_STD_AXISMAXDELTATRQ);

        usleep(1e6);

    }

    std::string KukieControlQueue::getCartesianLinkName() {
        return kin->getCartesianLinkName();
    }

    std::string KukieControlQueue::getCartesianReferenceFrame() {
        return kin->getCartesianReferenceFrame();
    }

    KUKADU_SHARED_PTR<Kinematics> KukieControlQueue::getKinematics() {
        kin = loadKinematics();
        return kin;
    }

    KUKADU_SHARED_PTR<PathPlanner> KukieControlQueue::getPlanner() {
        planner = loadPlanner();
        return planner;
    }

    KUKADU_SHARED_PTR<Kinematics> KukieControlQueue::loadKinematics() {

        if(!kinematicsInitialized) {

            planAndKinMutex.lock();

                // very hacky
                while(!kin) {

                    try {
                        vector<string> robotPrefixes = {getRobotSidePrefix()};
                        kin = make_shared<Komo>(shared_from_this(), resolvePath("$KUKADU_HOME/external/komo/share/data/kuka/data/iis_robot.kvg"),
                                resolvePath("$KUKADU_HOME/external/komo/share/data/kuka/config/MT.cfg"), robotPrefixes, acceptCollisions);
                        kinematicsInitialized = true;
                    } catch(std::bad_weak_ptr& ex) {
                        planAndKinMutex.unlock();
                        cout << "weak pointer not ready yet" << endl;
                    }

                }

            planAndKinMutex.unlock();

        }

        return kin;

    }

    KUKADU_SHARED_PTR<PathPlanner> KukieControlQueue::loadPlanner() {

        if(!plannerInitialized) {

            planAndKinMutex.lock();

                // very hacky
                while(!planner) {

                    try {
                        vector<string> robotPrefixes = {getRobotSidePrefix()};
                        auto actualPlanner = make_shared<Komo>(shared_from_this(), resolvePath("$KUKADU_HOME/external/komo/share/data/kuka/data/iis_robot.kvg"),
                                resolvePath("$KUKADU_HOME/external/komo/share/data/kuka/config/MT.cfg"), robotPrefixes, acceptCollisions);
                        planner = make_shared<CachedPlanner>(getStorage(), shared_from_this(), actualPlanner);
                        plannerInitialized = true;
                    } catch(std::bad_weak_ptr& ex) {
                        planAndKinMutex.unlock();
                        cout << "weak pointer not ready yet" << endl;
                    }

                }

            planAndKinMutex.unlock();

        }

        return planner;

    }

    void KukieControlQueue::setKinematics(KUKADU_SHARED_PTR<Kinematics> kin) {

        KUKADU_MODULE_START_USAGE();

        planAndKinMutex.lock();
            this->kin = kin;
            kinematicsInitialized = true;
        planAndKinMutex.unlock();

        KUKADU_MODULE_END_USAGE();

    }

    void KukieControlQueue::setPathPlanner(KUKADU_SHARED_PTR<PathPlanner> planner) {

        KUKADU_MODULE_START_USAGE();

        planAndKinMutex.lock();
            this->planner = planner;
            plannerInitialized = true;
        planAndKinMutex.unlock();

        KUKADU_MODULE_END_USAGE();

    }

    void KukieControlQueue::startQueueHook() {

        firstCartsComputed = false;
        cartPoseThr = kukadu_thread(&KukieControlQueue::computeCurrentCartPose, this);

        while(!firstCartsComputed)
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

    }

    /******** this is so ugly (but necessary) :( ***********/
    std::map<std::string, int> degsOfFreedomMap;
    void KukieControlQueue::degCallback(const ros::MessageEvent<sensor_msgs::JointState>& event) {
        degsOfFreedomMap[event.getConnectionHeader().at("topic")] = event.getMessage()->position.size();
    }

    int KukieControlQueue::loadDegOfFreedom(ros::NodeHandle node, std::string topic) {
        degsOfFreedomMap[topic] = -1;
        ros::Subscriber jointPos = node.subscribe(topic, 2, KukieControlQueue::degCallback);
        ros::Rate r(50);
        while(degsOfFreedomMap[topic] == -1)
            r.sleep();
        return degsOfFreedomMap[topic];
    }
    /******** ugly part over ***********/

    void KukieControlQueue::maxDistPerCycleCallback(const std_msgs::Float64& msg) {

        if(loadMaxDistPerCycleFromServer) {
            maxDistPerCycle = msg.data;
            firstMaxDistPerCycleReceived = true;
        }

    }

    void KukieControlQueue::cycleTimeCallback(const std_msgs::Float64& msg) {

        if(loadCycleTimeFromServer) {
            // * 0.5 comes from experience --> jerky movement otherwise; there is too much overhead in the run loop (the cycle time can't be the sleep time as well)
            setCycleTime(msg.data / 2.0);
            firstControllerCycletimeReceived = true;
        }

    }

    void KukieControlQueue::submitNextJointMove(arma::vec joints) {

        std_msgs::Float64MultiArray nextCommand;
        for(int i = 0; i < getDegreesOfFreedom(); ++i)
            nextCommand.data.push_back(joints[i]);
        pubCommand.publish(nextCommand);

    }

    void KukieControlQueue::submitNextCartMove(geometry_msgs::Pose pose) {

        pubCartMoveWfQueue.publish(pose);

    }

    std::string KukieControlQueue::getRobotSidePrefix() {

        KukaduTokenizer tok(armPrefix, "_");
        return tok.next();

    }

    void KukieControlQueue::cartPosRfCallback(const geometry_msgs::Pose msg) {
        currentCartPoseRf = msg;
    }

    geometry_msgs::Pose KukieControlQueue::getCurrentCartesianPoseRf() {
        KUKADU_MODULE_START_USAGE();
        KUKADU_MODULE_END_USAGE();
        return currentCartPoseRf;
    }

    void KukieControlQueue::setJntPtpThresh(double thresh) {
        std_msgs::Float64 th;
        th.data = thresh;
        pub_set_ptp_thresh.publish(th);
    }

    // relative pos in worldframe
    geometry_msgs::Pose KukieControlQueue::moveCartesianRelativeWf(geometry_msgs::Pose basePoseRf, geometry_msgs::Pose offset) {

        KUKADU_MODULE_START_USAGE();

        double newTargetWorldPos[4] = {1, 1, 1, 1};

        // add relative coordinates
        newTargetWorldPos[0] = basePoseRf.position.x + offset.position.x;
        newTargetWorldPos[1] = basePoseRf.position.y + offset.position.y;
        newTargetWorldPos[2] = basePoseRf.position.z + offset.position.z;

        // store it back to current pose
        basePoseRf.position.x = newTargetWorldPos[0];
        basePoseRf.position.y = newTargetWorldPos[1];
        basePoseRf.position.z = newTargetWorldPos[2];

        // publish robot frame pose to move
        move(basePoseRf);

        KUKADU_MODULE_END_USAGE();

        return basePoseRf;

    }

    std::string KukieControlQueue::getRobotDeviceType() {
        return deviceType;
    }

    std::string KukieControlQueue::getRobotFileName() {
        return string("kuka_lwr_") + deviceType + string("_") + armPrefix;
    }

    std::vector<std::string> KukieControlQueue::getJointNames() {
        loadKinematics();
        return kin->getJointNames();
    }

    void KukieControlQueue::jntMoveCallback(const std_msgs::Float64MultiArray& msg) {

    }

    void KukieControlQueue::cartFrcTrqCallback(const geometry_msgs::Wrench& msg) {

        cartFrcTrqMutex.lock();
            currentCartFrqTrq = vec(6);
            currentCartFrqTrq(0) = msg.force.x;
            currentCartFrqTrq(1) = msg.force.y;
            currentCartFrqTrq(2) = msg.force.z;
            currentCartFrqTrq(3) = msg.torque.x;
            currentCartFrqTrq(4) = msg.torque.y;
            currentCartFrqTrq(5) = msg.torque.z;
        cartFrcTrqMutex.unlock();

    }

    void KukieControlQueue::jntFrcTrqCallback(const std_msgs::Float64MultiArray& msg) {

        if(isRealRobot)
            currentJntFrqTrq = stdToArmadilloVec(msg.data);
        else {
            currentJntFrqTrq = vec(getDegreesOfFreedom());
            currentJntFrqTrq.fill(0.0);
        }

    }

    void KukieControlQueue::robotJointPosCallback(const sensor_msgs::JointState& msg) {

        currentJointsMutex.lock();

            currJoints = arma::vec(msg.position.size());
            for(int i = 0; i < msg.position.size(); ++i) currJoints(i) = msg.position.at(i);

            firstJointsReceived = true;

        currentJointsMutex.unlock();

    }

    arma::vec KukieControlQueue::getFrcTrqCart(){
        return currentCartFrqTrq;
    }

    void KukieControlQueue::computeCurrentCartPose() {

        loadKinematics();
        ros::Rate myRate(50);
        while(getQueueRunning()) {

            currCarts = kin->computeFk(armadilloToStdVec(getCurrentJoints().joints));
            firstCartsComputed = true;
            myRate.sleep();

        }

    }

    geometry_msgs::Pose KukieControlQueue::getCurrentCartesianPose() {

        KUKADU_MODULE_START_USAGE();
        KUKADU_MODULE_END_USAGE();

        return currCarts;

    }

    void KukieControlQueue::commandStateCallback(const std_msgs::Float32MultiArray& msg) {
        impMode = msg.data[1];
        firstModeReceived = true;
    }

    void KukieControlQueue::ptpReachedCallback(const std_msgs::Int32MultiArray& msg) {
        ptpReached = msg.data[0];
    }

    void KukieControlQueue::cartPtpReachedCallback(const std_msgs::Int32MultiArray& msg) {
        cartesianPtpReached = msg.data[0];
    }

    void KukieControlQueue::setInitValues() {
        impMode = -1;
        ptpReached = false;
        firstJointsReceived = false;
        firstModeReceived = false;
    }

    int KukieControlQueue::getCurrentMode() {

        KUKADU_MODULE_START_USAGE();
        KUKADU_MODULE_END_USAGE();

        return impMode;
    }

    mes_result KukieControlQueue::getCurrentCartesianFrcTrq() {

        KUKADU_MODULE_START_USAGE();

        mes_result ret;

        cartFrcTrqMutex.lock();
            ret.joints = currentCartFrqTrq;
        cartFrcTrqMutex.unlock();

        ret.time = getCurrentTime();

        KUKADU_MODULE_END_USAGE();

        return ret;

    }

    bool KukieControlQueue::stopQueueWhilePtp() {
        return false;
    }

    mes_result KukieControlQueue::getCurrentJntFrc() {

        KUKADU_MODULE_START_USAGE();

        mes_result ret;

        ret.joints = currentJntFrqTrq;
        ret.time = getCurrentTime();

        KUKADU_MODULE_END_USAGE();

        return ret;

    }

    void KukieControlQueue::setCurrentControlTypeInternal(int controlType) {

        std_msgs::Int32 newMode;
        newMode.data = controlType;
        if(ros::ok()) {
            pubSwitchMode.publish(newMode);
            ros::spinOnce();
        }
        while(impMode != controlType) {
            loop_rate->sleep();
            ros::spinOnce();
        }

    }

    void KukieControlQueue::cartPtpInternal(geometry_msgs::Pose pos, double maxForce) {

        cartesianPtpReached = false;

        while(!planner)
            loadPlanner();

        auto currentPose = getCurrentCartesianPose();
        vector<geometry_msgs::Pose> desiredPlan;
        desiredPlan.push_back(currentPose);
        desiredPlan.push_back(pos);

        planAndKinMutex.lock();

            vector<vec> desiredJointPlan;
            try {
                desiredJointPlan = planner->planCartesianTrajectory(getCurrentJoints().joints, desiredPlan, false, true);
            } catch(KukaduException& ex) {
                planAndKinMutex.unlock();
                throw ex;
            }

        planAndKinMutex.unlock();

        bool maxForceExceeded = false;
        if(desiredJointPlan.size() > 0) {

            for(int i = 0; i < desiredJointPlan.size() && !maxForceExceeded; ++i) {

                if(getAbsoluteCartForce() > maxForce)
                    maxForceExceeded = true;
                else
                    move(desiredJointPlan.at(i));
                synchronizeToQueue(1);

            }

        } else {
            ROS_ERROR("(KukieControlQueue) Cartesian position not reachable");
        }

    }

    void KukieControlQueue::jointPtpInternal(arma::vec joints, double maxForce) {

        ptpReached = false;
        loadPlanner();
        bool performPtp = false;
        vec currentState = getCurrentJoints().joints;
        for(int i = 0; i < joints.n_elem; ++i)
            if(abs(currentState(i) - joints(i)) > 0.01) {
                performPtp = true;
                break;
            }

        vector<vec> desiredJointPlan;
        if(performPtp) {

            vector<arma::vec> desiredPlan;
            desiredPlan.push_back(getCurrentJoints().joints);
            desiredPlan.push_back(joints);

            planAndKinMutex.lock();
                desiredJointPlan = planner->planJointTrajectory(desiredPlan);
            planAndKinMutex.unlock();

        }

        bool checkMaxForce = false;
        if(maxForce >= 0.0)
            checkMaxForce = true;

        if(checkMaxForce)
            startRollBackMode(1.5);

        if(performPtp) {
            if(desiredJointPlan.size() > 0) {

                for(int i = 0; i < desiredJointPlan.size(); ++i) {

                    if(checkMaxForce && getAbsoluteCartForce() > maxForce) {
                        rollBack(1.0);
                        break;
                    } else {
                        move(desiredJointPlan.at(i));
                        synchronizeToQueue(1);
                    }
                }

            } else {
                ROS_ERROR("(KukieControlQueue) Joint plan not reachable");
            }
        }

        if(checkMaxForce)
            stopJointRollBackMode();

    }

    double KukieControlQueue::computeDistance(float* a1, float* a2, int size) {
        double ret = 0.0;
        for(int i = 0; i < size; ++i) {
            ret = pow(a1[i] - a2[i], 2);
        }

        return ret;
    }

    void KukieControlQueue::setAdditionalLoad(float loadMass, float loadPos) {

        KUKADU_MODULE_START_USAGE();

        if(isRealRobot) {

            std_msgs::Float32MultiArray msg;
            msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
            msg.layout.dim[0].size = 2;
            msg.layout.dim[0].stride = 0;
            msg.layout.dim[0].label = "Load";
            msg.data.push_back(loadMass);
            msg.data.push_back(loadPos);

            pubAddLoad.publish(msg);

            int tmpMode = getCurrentMode();
            stopCurrentMode();
            switchMode(tmpMode);

        } else {
            if(!isShutUp())
                ROS_INFO("(setAdditionalLoad) this functionality is not available in simulator - ignored");
        }

        KUKADU_MODULE_END_USAGE();

    }

    void KukieControlQueue::startKinestheticTeachingStiffness() {
        setStiffness(0.2, 0.01, 0.2, 15000, 150, 1500);
    }

    void KukieControlQueue::stopKinestheticTeachingStiffness() {

        int diffCount = 5;

        double deltaCpStiffXyz = (prevCpstiffnessxyz - 0.2) / diffCount;
        double deltaCpStiffAbc = (prevCpstiffnessabc - 0.01) / diffCount;
        double deltaCpDamping = (prevCpdamping - 0.2) / diffCount;
        double deltaCpMaxDelta = (prevCpmaxdelta - 15000) / diffCount;
        double deltaMaxFrc = (prevMaxforce - 150) / diffCount;
        double deltaAxisMaxDeltaTrq = (prevAxismaxdeltatrq - 1500) / diffCount;

        ros::Rate r2(2);
        r2.sleep();

        ros::Rate r(5);
        for(int i = 0; i < diffCount; ++i) {
            setStiffness(0.2 + deltaCpStiffXyz * i, 0.01 + deltaCpStiffAbc * i, 0.2 + deltaCpDamping * i,
                         15000, 150, 1500, false);
            r.sleep();
        }

        r2.sleep();

        for(int i = 0; i < diffCount; ++i) {
            setStiffness(prevCpstiffnessxyz, prevCpstiffnessabc, prevCpdamping,
                         15000 + deltaCpMaxDelta * i, 150 + deltaMaxFrc * i, 1500 + deltaAxisMaxDeltaTrq * i, false);
            r.sleep();
        }

        /*
        r2.sleep();
        setStiffness(prevCpstiffnessxyz, prevCpstiffnessabc, prevCpdamping,
                     prevCpmaxdelta, prevMaxforce, prevMaxforce);
                     */

    }

    void KukieControlQueue::setStiffness(float cpstiffnessxyz, float cpstiffnessabc, float cpdamping, float cpmaxdelta, float maxforce, float axismaxdeltatrq) {
        setStiffness(cpstiffnessxyz, cpstiffnessabc, cpdamping, cpmaxdelta, maxforce, axismaxdeltatrq, true);
    }

    void KukieControlQueue::setStiffness(float cpstiffnessxyz, float cpstiffnessabc, float cpdamping, float cpmaxdelta, float maxforce, float axismaxdeltatrq, bool storePrevValues) {

        KUKADU_MODULE_START_USAGE();

        if(storePrevValues) {
            this->prevCpstiffnessxyz = this->cpstiffnessxyz;
            this->prevCpstiffnessabc = this->cpstiffnessabc;
            this->prevCpdamping = this->cpdamping;
            this->prevCpmaxdelta = this->cpmaxdelta;
            this->prevMaxforce = this->maxforce;
            this->prevAxismaxdeltatrq = this->axismaxdeltatrq;
        }

        this->cpstiffnessxyz = cpstiffnessxyz;
        this->cpstiffnessabc = cpstiffnessabc;
        this->cpdamping = cpdamping;
        this->cpmaxdelta = cpmaxdelta;
        this->maxforce = maxforce;
        this->axismaxdeltatrq = axismaxdeltatrq;

        if(isRealRobot) {

            if(cpdamping < 0)
                cpdamping = KUKA_STD_CPDAMPING;

            iis_robot_dep::CartesianImpedance imp;

            imp.stiffness.linear.x = imp.stiffness.linear.y = imp.stiffness.linear.z = cpstiffnessxyz;
            imp.damping.linear.x = imp.damping.linear.y = imp.damping.linear.z = cpdamping;
            imp.stiffness.angular.x = imp.stiffness.angular.y = imp.stiffness.angular.z = cpstiffnessabc;
            imp.damping.angular.x = imp.damping.angular.y = imp.damping.angular.z = cpdamping;
            imp.cpmaxdelta = cpmaxdelta;
            imp.axismaxdeltatrq = axismaxdeltatrq;

            iis_robot_dep::FriJointImpedance newImpedance;
            for (int j = 0; j < 7; j++){
                newImpedance.stiffness[j] = cpstiffnessxyz;
                newImpedance.damping[j] = cpdamping;
            }

            pub_set_cart_stiffness.publish(imp);
            pub_set_joint_stiffness.publish(newImpedance);

        } else {
            if(!isShutUp())
                ROS_INFO("(setStiffness) this functionality is not available in simulator - ignored");
        }

        KUKADU_MODULE_END_USAGE();

    }

    mes_result KukieControlQueue::getCurrentJoints() {

        KUKADU_MODULE_START_USAGE();

        mes_result res;
        res.time = getCurrentTime();
        res.joints = currJoints;

        KUKADU_MODULE_END_USAGE();

        return res;
    }

    void KukieControlQueue::safelyDestroy() {

    }

    PlottingControlQueue::PlottingControlQueue(StorageSingleton& storage, std::string robotName, int degOfFreedom, std::string referenceFrame, std::string linkName, double timeStep) :
        ControlQueue(storage, robotName, "PlottingControlQueue", degOfFreedom, timeStep) {

        this->linkName = linkName;
        this->referenceFrame = referenceFrame;

        vector<string> jntNames;
        for(int i = 0; i < getDegreesOfFreedom(); ++i) {
            stringstream s;
            s << i;
            jntNames.push_back(string("joint_") + s.str());
        }
        construct(jntNames, timeStep);

    }

    void PlottingControlQueue::construct(std::vector<std::string> jointNames, double timeStep) {

        this->jointNames = jointNames;
        setInitValues();

    }

    void PlottingControlQueue::startQueueHook() {

    }

    void PlottingControlQueue::startKinestheticTeachingStiffness() {
        // nothing to do
    }

    void PlottingControlQueue::stopKinestheticTeachingStiffness() {
        // nothing to do
    }

    void PlottingControlQueue::setInitValues() {

        set_ctrlc_exit_handler();
        currTime = getCurrentTime();
        currJoints = arma::vec(1);

    }

    std::string PlottingControlQueue::getRobotFileName() {
        return string("simulation_plotting_control_queue");
    }

    std::vector<std::string> PlottingControlQueue::getJointNames() {
        return jointNames;
    }

    void PlottingControlQueue::rollBack(double time) {
        KUKADU_MODULE_START_USAGE();
        KUKADU_MODULE_END_USAGE();
    //    throw KukaduException("(PlottingControlQueue) roll back mode not supported");
    }

    void PlottingControlQueue::stopJointRollBackMode() {
        KUKADU_MODULE_START_USAGE();
        KUKADU_MODULE_END_USAGE();
    //    throw KukaduException("(PlottingControlQueue) roll back mode not supported");
    }

    void PlottingControlQueue::startJointRollBackMode(double possibleTime) {
        KUKADU_MODULE_START_USAGE();
        KUKADU_MODULE_END_USAGE();
    //    throw KukaduException("(PlottingControlQueue) roll back mode not supported");
    }

    void PlottingControlQueue::setJntPtpThresh(double thresh) {
        KUKADU_MODULE_START_USAGE();
        KUKADU_MODULE_END_USAGE();
    }

    mes_result PlottingControlQueue::getCurrentCartesianFrcTrq() {

        KUKADU_MODULE_START_USAGE();

        mes_result ret;
        vec frcTrq(6);
        frcTrq.fill(0.0);

        ret.joints = frcTrq;
        ret.time = getCurrentTime();

        KUKADU_MODULE_END_USAGE();

        return ret;

    }

    mes_result PlottingControlQueue::getCurrentJntFrc() {

        KUKADU_MODULE_START_USAGE();

        mes_result ret;
        vec frcTrq(getDegreesOfFreedom());
        frcTrq.fill(0.0);

        ret.joints = frcTrq;
        ret.time = getCurrentTime();

        KUKADU_MODULE_END_USAGE();

        return ret;

    }

    arma::vec PlottingControlQueue::getStartingJoints() {
        return startJoints;
    }

    void PlottingControlQueue::addCartesianPosToQueue(geometry_msgs::Pose pose) {
        fakeCurrentPose = pose;
    }

    void PlottingControlQueue::switchMode(int mode) {
        KUKADU_MODULE_START_USAGE();
        KUKADU_MODULE_END_USAGE();
    }

    void PlottingControlQueue::submitNextJointMove(arma::vec joints) {
        KUKADU_MODULE_START_USAGE();
        KUKADU_MODULE_END_USAGE();
    }

    void PlottingControlQueue::submitNextCartMove(geometry_msgs::Pose pose) {
        KUKADU_MODULE_START_USAGE();
        KUKADU_MODULE_END_USAGE();
    }

    bool PlottingControlQueue::stopQueueWhilePtp() {
        return false;
    }

    void PlottingControlQueue::setCurrentControlTypeInternal(int controlType) {

    }

    void PlottingControlQueue::stopCurrentMode() {
        KUKADU_MODULE_START_USAGE();
        KUKADU_MODULE_END_USAGE();
    }

    void PlottingControlQueue::synchronizeToControlQueue(int maxNumJointsInQueue) {
        KUKADU_MODULE_START_USAGE();
        KUKADU_MODULE_END_USAGE();
    }

    void PlottingControlQueue::setStartingJoints(arma::vec joints) {
        currJoints = joints;
        startJoints = joints;
    }

    void PlottingControlQueue::cartPtpInternal(geometry_msgs::Pose pos, double maxForce) {
        fakeCurrentPose = pos;
    }

    void PlottingControlQueue::jointPtpInternal(arma::vec joints, double maxForce) {
        currJoints = joints;
    }

    int PlottingControlQueue::getCurrentMode() {
        return CONTROLQUEUE_JNT_POS_MODE;
    }

    std::string PlottingControlQueue::getCartesianLinkName() {
        return linkName;
    }

    std::string PlottingControlQueue::getCartesianReferenceFrame() {
        return referenceFrame;
    }

    void PlottingControlQueue::setAdditionalLoad(float loadMass, float loadPos) {
        KUKADU_MODULE_START_USAGE();
        KUKADU_MODULE_END_USAGE();
    }

    void PlottingControlQueue::setStiffness(float cpstiffnessxyz, float cpstiffnessabc, float cpdamping, float cpmaxdelta, float maxforce, float axismaxdeltatrq) {
        KUKADU_MODULE_START_USAGE();
        KUKADU_MODULE_END_USAGE();
    }

    mes_result PlottingControlQueue::getCurrentCartesianPos() {
        KUKADU_MODULE_START_USAGE();
        mes_result res;
        res.time = 0.0;
        res.joints = stdToArmadilloVec(createJointsVector(7, fakeCurrentPose.position.x, fakeCurrentPose.position.y, fakeCurrentPose.position.z,
                                                          fakeCurrentPose.orientation.x, fakeCurrentPose.orientation.y, fakeCurrentPose.orientation.z, fakeCurrentPose.orientation.w));
        KUKADU_MODULE_END_USAGE();
        return res;
    }

    geometry_msgs::Pose PlottingControlQueue::getCurrentCartesianPose() {
        KUKADU_MODULE_START_USAGE();
        KUKADU_MODULE_END_USAGE();
        return fakeCurrentPose;
    }

    mes_result PlottingControlQueue::getCurrentJoints() {
        KUKADU_MODULE_START_USAGE();
        mes_result res;
        res.time = getCurrentTime();
        res.joints = currJoints;
        KUKADU_MODULE_END_USAGE();
        return res;
    }

    void PlottingControlQueue::safelyDestroy() {
    }

}
