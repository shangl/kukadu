#include <chrono>
#include <sstream>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <kukadu/utils/utils.hpp>
#include <kukadu/robot/queue.hpp>
#include <kukadu/planning/komo.hpp>
#include <kukadu/robot/kukiequeue.hpp>
#include <kukadu/types/kukadutypes.hpp>
#include <iis_robot_dep/FriJointImpedance.h>
#include <iis_robot_dep/CartesianImpedance.h>

using namespace std;
using namespace arma;
using namespace chrono;

namespace kukadu {

    int ControlQueue::getRobotId() {

        if(!robot)
            robot = make_shared<Robot>(dbStorage, getRobotName());

        return robot->getRobotId();

    }

    int ControlQueue::loadDegOfFreedom(StorageSingleton& storage, std::string robotName) {

        if(!robot)
            robot = make_shared<Robot>(storage, robotName);

        return robot->getDegOfFreedom();

    }

    std::string ControlQueue::getRobotName() {
        return robotName;
    }

    int ControlQueue::getJointId(std::string jointName) {

        stringstream s;
        s << "select joint_id from robot_joints where robot_id=" << getRobotId() << " and joint_name=\"" << jointName << "\"";
        auto idRes = dbStorage.executeQuery(s.str());
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

        setInitValues();

        frcTrqFilterUpdateThr = KUKADU_SHARED_PTR<kukadu_thread>(new kukadu_thread(&ControlQueue::frcTrqFilterUpdateHandler, this));
        thr = KUKADU_SHARED_PTR<kukadu_thread>(new kukadu_thread(&ControlQueue::run, this));

        while(!this->isInitialized());
        startQueueHook();

        return thr;

    }

    ControlQueue::ControlQueue(StorageSingleton& storage, std::string robotName, double desiredCycleTime) : dbStorage(storage) {

        this->robotName = robotName;
        this->desiredCycleTime = desiredCycleTime;
        this->sleepTime = desiredCycleTime;

        jointPtpRunning = false;
        cartesianPtpRunning = false;
        frcTrqFilterRunning=true;
        currentTime = getCurrentTime();
        degOfFreedom = loadDegOfFreedom(storage, robotName);
        continueCollecting = false;
        currentFrcTrqSensorFilter = make_shared<StandardFilter>();

    }

    void ControlQueue::setFrcTrqSensorFilter(KUKADU_SHARED_PTR<FrcTrqSensorFilter> myFilter) {
        currentFrcTrqSensorFilter=myFilter;
    }

    mes_result ControlQueue::getCurrentProcessedCartesianFrcTrq() {
        return currentFrcTrqSensorFilter->getProcessedReading();
    }

    void ControlQueue::setDegOfFreedom(int degOfFreedom) {
        this->degOfFreedom = degOfFreedom;
    }

    int ControlQueue::getDegreesOfFreedom() {
        return degOfFreedom;
    }

    void ControlQueue::setCycleTime(double cycleTime) {
        loadCycleTimeMutex.lock();
        this->sleepTime = cycleTime;
        this->desiredCycleTime = cycleTime;
        loadCycleTimeMutex.unlock();
    }

    double ControlQueue::getAbsoluteCartForce() {

        mes_result m = getCurrentCartesianFrcTrq();
        vec forces = m.joints.subvec(0, 2);
        vec prod = forces.t() * forces;
        return sqrt(prod(0));

    }

    double ControlQueue::getCycleTime() {
        loadCycleTimeMutex.lock();
        auto cycleTmp = desiredCycleTime;
        loadCycleTimeMutex.unlock();
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
        for(auto joint : jointTrajectory)
            move(joint);
        synchronizeToQueue(1);
    }

    void ControlQueue::move(geometry_msgs::Pose pose) {
        cartesianMovementQueue.push(pose);
    }

    void ControlQueue::switchMode(int mode) {

        if(ros::ok) {
            if(!isShutUp())
                ROS_INFO("(ControlQueue) switching control mode");

            setCurrentControlTypeInternal(mode);
            currentControlType = mode;
        } else
            if(!isShutUp())
                ROS_INFO("(ControlQueue) ros error");

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
        finish = 1;
        startingJoints = arma::vec(1);
    }

    bool ControlQueue::isInitialized() {
        return isInit;
    }

    void ControlQueue::move(arma::vec joints) {
        movementQueue.push(joints);
    }

    mes_result ControlQueue::getCurrentCartesianPos() {

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
        return ret;

    }

    bool ControlQueue::getQueueRunning() {
        return !finish;
    }

    void ControlQueue::synchronizeToQueue(int maxNumJointsInQueue) {
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
    }

    void ControlQueue::stopCurrentMode() {
        switchMode(CONTROLQUEUE_STOP_MODE);
        switchMode(CONTROLQUEUE_JNT_POS_MODE);
        switchMode(CONTROLQUEUE_STOP_MODE);
    }

    void ControlQueue::internalJointPtpCaller() {
        jointPtp(internalJointPasser);
    }

    KUKADU_SHARED_PTR<kukadu_thread> ControlQueue::jointPtpNb(arma::vec joints) {

        internalJointPasser = joints;
        jointPtpThr = KUKADU_SHARED_PTR<kukadu_thread>(new kukadu_thread(&ControlQueue::internalJointPtpCaller, this));
        return jointPtpThr;

    }

    void ControlQueue::internalCartPtpCaller() {
        cartesianPtp(internalPosePasser);
    }

    KUKADU_SHARED_PTR<kukadu_thread> ControlQueue::cartesianPtpNb(geometry_msgs::Pose pos) {

        internalPosePasser = pos;
        cartPtpThr = KUKADU_SHARED_PTR<kukadu_thread>(new kukadu_thread(&ControlQueue::internalJointPtpCaller, this));
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

                rollBackQueue.push_front(currentJoints);

                // if queue is full --> go back
                while(rollBackQueue.size() > rollBackQueueSize)
                    rollBackQueue.pop_back();

            }

            if(!stopQueueWhilePtp() || !jointPtpRunning && !cartesianPtpRunning) {

                if(currentControlType == CONTROLQUEUE_JNT_IMP_MODE || currentControlType == CONTROLQUEUE_JNT_POS_MODE) {

                    if(movementQueue.size() > 0) {

                        // move to position in queue
                        movement = movementQueue.front();
                        movementQueue.pop();

                    } else {

                        if(stopQueueWhilePtp())
                            movement = getCurrentJoints().joints;

                    }

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

            ros::spinOnce();

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

    std::vector<mes_result> ControlQueue::jointPtp(arma::vec joints) {

        jointPtpRunning = true;

        if(!continueCollecting) {

            continueCollecting = true;
            jointsColletorThr = KUKADU_SHARED_PTR<kukadu_thread>(new kukadu_thread(&ControlQueue::jointsCollector, this));
            jointPtpInternal(joints);
            continueCollecting = false;
            jointsColletorThr->join();

        } else {

            throw KukaduException("(ControlQueue) only one ptp at a time can be executed");

        }

        jointPtpRunning = false;
        return collectedJoints;

    }

    std::vector<mes_result> ControlQueue::cartesianPtp(geometry_msgs::Pose pos, double maxForce) {

        cartesianPtpRunning = true;

        if(!continueCollecting) {

            continueCollecting = true;
            jointsColletorThr = KUKADU_SHARED_PTR<kukadu_thread>(new kukadu_thread(&ControlQueue::jointsCollector, this));

            try {
                cartPtpInternal(pos, maxForce);
            } catch(KukaduException& ex) {
                continueCollecting = false;
                jointsColletorThr->join();
                throw ex;
            }

            continueCollecting = false;
            jointsColletorThr->join();

        } else {

            throw KukaduException("(ControlQueue) only one ptp at a time can be executed");

        }

        cartesianPtpRunning = false;
        return collectedJoints;

    }

    std::vector<mes_result> ControlQueue::cartesianPtp(geometry_msgs::Pose pos) {

        return cartesianPtp(pos, DBL_MAX);

    }

    void ControlQueue::startRollBackMode(double possibleTime) {

        rollBackQueue.clear();
        rollbackMode = true;
        rollBackTime = possibleTime;
        // buffer of 1.0 more second
        rollBackQueueSize = (int) ((possibleTime + 1.0) / getCycleTime());

    }

    void ControlQueue::stopJointRollBackMode() {

        rollbackMode = false;
        rollBackQueue.clear();

    }

    void ControlQueue::rollBack(double time) {

        rollbackMode = false;
        int rollBackCount = (int) (time / getCycleTime());

        int stretchFactor = ceil((double) rollBackCount / (double) rollBackQueue.size());
        stretchFactor = max((double) stretchFactor, 1.0);

        vec lastCommand(getDegreesOfFreedom());
        if(rollBackQueue.size())
            lastCommand = rollBackQueue.front();

        int newRollBackCount = ceil((double) rollBackCount / (double) stretchFactor);

        // fill command queue with last commands (backwards)
        for(int i = 0; i < newRollBackCount && rollBackQueue.size(); ++i) {

            vec nextCommand = rollBackQueue.front();

            // interpolate to stretch the trajectory in case there are not enough measured packets (happens in usage with simulator)
            vec diffUnit = (nextCommand - lastCommand) / (double) stretchFactor;
            for(int j = 0; j < stretchFactor; ++j) {
                move(lastCommand + j * diffUnit);
            }

            lastCommand = nextCommand;
            rollBackQueue.pop_front();

        }

        // wait until everything has been executed
        synchronizeToQueue(1);
        rollBackQueue.clear();

    }

    void ControlQueue::frcTrqFilterUpdateHandler(){
        ros::Rate myRate(50);
        while(frcTrqFilterRunning){
            currentFrcTrqSensorFilter->updateFilter(getCurrentCartesianFrcTrq());
            myRate.sleep();
        }
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

        usleep(1e6);

    }

    std::string KukieControlQueue::getCartesianLinkName() {
        return kin->getCartesianLinkName();
    }

    std::string KukieControlQueue::getCartesianReferenceFrame() {
        return kin->getCartesianReferenceFrame();
    }

    KUKADU_SHARED_PTR<Kinematics> KukieControlQueue::getKinematics() {
        return kin;
    }

    KUKADU_SHARED_PTR<Kinematics> KukieControlQueue::loadKinematics() {

        if(!kinematicsInitialized) {

            planAndKinMutex.lock();
            kin = make_shared<Komo>(shared_from_this(), resolvePath("$KUKADU_HOME/external/komo/share/data/kuka/data/iis_robot.kvg"), resolvePath("$KUKADU_HOME/external/komo/share/data/kuka/config/MT.cfg"), getRobotSidePrefix(), acceptCollisions);

            kinematicsInitialized = true;
            planAndKinMutex.unlock();

        }

        return kin;

    }

    KUKADU_SHARED_PTR<PathPlanner> KukieControlQueue::loadPlanner() {

        if(!plannerInitialized) {

            planAndKinMutex.lock();
            planner = make_shared<Komo>(shared_from_this(), resolvePath("$KUKADU_HOME/external/komo/share/data/kuka/data/iis_robot.kvg"), resolvePath("$KUKADU_HOME/external/komo/share/data/kuka/config/MT.cfg"), getRobotSidePrefix(), acceptCollisions);
            plannerInitialized = true;
            planAndKinMutex.unlock();

        }

        return planner;

    }

    void KukieControlQueue::setKinematics(KUKADU_SHARED_PTR<Kinematics> kin) {

        planAndKinMutex.lock();

        this->kin = kin;
        kinematicsInitialized = true;

        planAndKinMutex.unlock();

    }

    void KukieControlQueue::setPathPlanner(KUKADU_SHARED_PTR<PathPlanner> planner) {

        planAndKinMutex.lock();

        this->planner = planner;
        plannerInitialized = true;

        planAndKinMutex.unlock();

    }

    void KukieControlQueue::startQueueHook() {

        firstCartsComputed = false;
        cartPoseThr = kukadu_thread(&KukieControlQueue::computeCurrentCartPose, this);

        while(!firstCartsComputed)
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

    }

    KukieControlQueue::KukieControlQueue(StorageSingleton& storage, std::string robotName, std::string deviceType, std::string armPrefix, ros::NodeHandle node, bool acceptCollisions, KUKADU_SHARED_PTR<Kinematics> kin, KUKADU_SHARED_PTR<PathPlanner> planner, double sleepTime, double maxDistPerCycle) : ControlQueue(storage, robotName, sleepTime) {

        commandTopic = "/" + deviceType + "/" + armPrefix + "/joint_control/move";
        retJointPosTopic = "/" + deviceType + "/" + armPrefix + "/joint_control/get_state";
        switchModeTopic = "/" + deviceType + "/" + armPrefix + "/settings/switch_mode";
        retCartPosTopic = "/" + deviceType + "/" + armPrefix + "/cartesian_control/get_pose_quat_wf";
        stiffnessTopic = "/" + deviceType + "/" + armPrefix + "/cartesian_control/set_impedance";
        jntStiffnessTopic = "/" + deviceType + "/" + armPrefix + "/joint_control/set_impedance";
        ptpTopic = "/" + deviceType + "/" + armPrefix + "/joint_control/ptp";
        commandStateTopic = "/" + deviceType + "/" + armPrefix + "/settings/get_command_state";
        ptpReachedTopic = "/" + deviceType + "/" + armPrefix + "/joint_control/ptp_reached";
        jntFrcTrqTopic = "/" + deviceType + "/" + armPrefix + "/sensoring/est_ext_jnt_trq";
        cartFrcTrqTopic = "/" + deviceType + "/" + armPrefix + "/sensoring/cartesian_wrench";
        cartPtpTopic = "/" + deviceType + "/" + armPrefix + "/cartesian_control/ptpQuaternion";
        cartPtpReachedTopic = "/" + deviceType + "/" + armPrefix + "/cartesian_control/ptp_reached";
        cartMoveRfQueueTopic = "/" + deviceType + "/" + armPrefix + "/cartesian_control/move_rf";
        cartMoveWfQueueTopic = "/" + deviceType + "/" + armPrefix + "/cartesian_control/move_wf";
        cartPoseRfTopic = "/" + deviceType + "/" + armPrefix + "/cartesian_control/get_pose_quat_rf";
        jntSetPtpThreshTopic = "/" + deviceType + "/" + armPrefix + "/joint_control/set_ptp_thresh";
        clockCycleTopic = "/" + deviceType + "/" + armPrefix + "/settings/get_clock_cycle";
        maxDistPerCycleTopic = "/" + deviceType + "/" + armPrefix + "/joint_control/get_max_dist_per_cycle";
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

    void KukieControlQueue::maxDistPerCycleCallback(const std_msgs::Float64& msg) {

        if(loadMaxDistPerCycleFromServer) {
            maxDistPerCycle = msg.data;
            firstMaxDistPerCycleReceived = true;
        }

    }

    void KukieControlQueue::cycleTimeCallback(const std_msgs::Float64& msg) {

        if(loadCycleTimeFromServer) {
            setCycleTime(msg.data);
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
        return currentCartPoseRf;
    }

    void KukieControlQueue::setJntPtpThresh(double thresh) {
        std_msgs::Float64 th;
        th.data = thresh;
        pub_set_ptp_thresh.publish(th);
    }

    // relative pos in worldframe
    geometry_msgs::Pose KukieControlQueue::moveCartesianRelativeWf(geometry_msgs::Pose basePoseRf, geometry_msgs::Pose offset) {

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

        return basePoseRf;

    }

    std::string KukieControlQueue::getRobotDeviceType() {
        return deviceType;
    }

    std::string KukieControlQueue::getRobotFileName() {
        return string("kuka_lwr_") + deviceType + string("_") + armPrefix;
    }

    std::vector<std::string> KukieControlQueue::getJointNames() {

        return kin->getJointNames();

    }

    void KukieControlQueue::jntMoveCallback(const std_msgs::Float64MultiArray& msg) {

    }

    void KukieControlQueue::cartFrcTrqCallback(const geometry_msgs::Wrench& msg) {

        cartFrcTrqMutex.lock();
            currentCartFrqTrq = vec(6);
            if(isRealRobot) {
                currentCartFrqTrq(0) = msg.force.x;
                currentCartFrqTrq(1) = msg.force.y;
                currentCartFrqTrq(2) = msg.force.z;
                currentCartFrqTrq(3) = msg.torque.x;
                currentCartFrqTrq(4) = msg.torque.y;
                currentCartFrqTrq(5) = msg.torque.z;
            } else {
                currentCartFrqTrq.fill(0.0);
            }
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

            planAndKinMutex.lock();

                currCarts = kin->computeFk(armadilloToStdVec(getCurrentJoints().joints));
                firstCartsComputed = true;
                myRate.sleep();

            planAndKinMutex.unlock();

        }

    }

    geometry_msgs::Pose KukieControlQueue::getCurrentCartesianPose() {

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
        return impMode;
    }

    mes_result KukieControlQueue::getCurrentCartesianFrcTrq() {

        mes_result ret;

        cartFrcTrqMutex.lock();
            ret.joints = currentCartFrqTrq;
        cartFrcTrqMutex.unlock();

        ret.time = getCurrentTime();

        return ret;

    }

    bool KukieControlQueue::stopQueueWhilePtp() {
        return false;
    }

    mes_result KukieControlQueue::getCurrentJntFrc() {

        mes_result ret;

        ret.joints = currentJntFrqTrq;
        ret.time = getCurrentTime();

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

    void KukieControlQueue::jointPtpInternal(arma::vec joints) {

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

        if(performPtp) {
            if(desiredJointPlan.size() > 0) {

                for(int i = 0; i < desiredJointPlan.size(); ++i)
                    move(desiredJointPlan.at(i));
                synchronizeToQueue(1);

            } else {
                ROS_ERROR("(KukieControlQueue) Joint plan not reachable");
            }
        }

    }

    double KukieControlQueue::computeDistance(float* a1, float* a2, int size) {
        double ret = 0.0;
        for(int i = 0; i < size; ++i) {
            ret = pow(a1[i] - a2[i], 2);
        }

        return ret;
    }

    void KukieControlQueue::setAdditionalLoad(float loadMass, float loadPos) {

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

    }

    void KukieControlQueue::setStiffness(float cpstiffnessxyz, float cpstiffnessabc, float cpdamping, float cpmaxdelta, float maxforce, float axismaxdeltatrq) {

        if(isRealRobot) {
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
            ros::spinOnce();
        } else {
            if(!isShutUp())
                ROS_INFO("(setStiffness) this functionality is not available in simulator - ignored");
        }

    }

    mes_result KukieControlQueue::getCurrentJoints() {
        mes_result res;
        res.time = getCurrentTime();
        res.joints = currJoints;
        return res;
    }

    void KukieControlQueue::safelyDestroy() {
    }

    PlottingControlQueue::PlottingControlQueue(StorageSingleton& storage, std::string robotName, std::string referenceFrame, std::string linkName, double timeStep) : ControlQueue(storage, robotName, timeStep) {

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
    //    throw KukaduException("(PlottingControlQueue) roll back mode not supported");
    }

    void PlottingControlQueue::stopJointRollBackMode() {
    //    throw KukaduException("(PlottingControlQueue) roll back mode not supported");
    }

    void PlottingControlQueue::startJointRollBackMode(double possibleTime) {
    //    throw KukaduException("(PlottingControlQueue) roll back mode not supported");
    }

    void PlottingControlQueue::setJntPtpThresh(double thresh) {

    }

    mes_result PlottingControlQueue::getCurrentCartesianFrcTrq() {

        mes_result ret;
        vec frcTrq(6);
        frcTrq.fill(0.0);

        ret.joints = frcTrq;
        ret.time = getCurrentTime();

        return ret;

    }

    mes_result PlottingControlQueue::getCurrentJntFrc() {

        mes_result ret;
        vec frcTrq(getDegreesOfFreedom());
        frcTrq.fill(0.0);

        ret.joints = frcTrq;
        ret.time = getCurrentTime();

        return ret;

    }

    arma::vec PlottingControlQueue::getStartingJoints() {
        return startJoints;
    }

    void PlottingControlQueue::addJointsPosToQueue(arma::vec joints) {
        currJoints = joints;
        currTime += (long) (getCycleTime() * 1e3);
    }

    void PlottingControlQueue::addCartesianPosToQueue(geometry_msgs::Pose pose) {
        fakeCurrentPose = pose;
    }

    void PlottingControlQueue::switchMode(int mode) {

    }

    void PlottingControlQueue::submitNextJointMove(arma::vec joints) {

    }

    void PlottingControlQueue::submitNextCartMove(geometry_msgs::Pose pose) {

    }

    bool PlottingControlQueue::stopQueueWhilePtp() {
        return false;
    }

    void PlottingControlQueue::setCurrentControlTypeInternal(int controlType) {

    }

    void PlottingControlQueue::stopCurrentMode() {
    }

    void PlottingControlQueue::synchronizeToControlQueue(int maxNumJointsInQueue) {
    }

    void PlottingControlQueue::setStartingJoints(arma::vec joints) {
        currJoints = joints;
        startJoints = joints;
    }

    void PlottingControlQueue::cartPtpInternal(geometry_msgs::Pose pos, double maxForce) {
        fakeCurrentPose = pos;
    }

    void PlottingControlQueue::jointPtpInternal(arma::vec joints) {
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
    }

    void PlottingControlQueue::setStiffness(float cpstiffnessxyz, float cpstiffnessabc, float cpdamping, float cpmaxdelta, float maxforce, float axismaxdeltatrq) {
    }

    mes_result PlottingControlQueue::getCurrentCartesianPos() {
        mes_result res;
        res.time = 0.0;
        res.joints = stdToArmadilloVec(createJointsVector(7, fakeCurrentPose.position.x, fakeCurrentPose.position.y, fakeCurrentPose.position.z,
                                                          fakeCurrentPose.orientation.x, fakeCurrentPose.orientation.y, fakeCurrentPose.orientation.z, fakeCurrentPose.orientation.w));
        return res;
    }

    geometry_msgs::Pose PlottingControlQueue::getCurrentCartesianPose() {
        return fakeCurrentPose;
    }

    mes_result PlottingControlQueue::getCurrentJoints() {
        mes_result res;
        res.time = getCurrentTime();
        res.joints = currJoints;
        return res;
    }

    void PlottingControlQueue::safelyDestroy() {
    }

}
