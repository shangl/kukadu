#include <map>
#include <algorithm>
#include <kukadu/control/controller.hpp>
#include <kukadu/storage/moduleusagesingleton.hpp>
#include <kukadu/storage/sensorstoragesingleton.hpp>
#include <kukadu/storage/sensorstorage.hpp>
#include <kukadu/robot/hardwarefactory.hpp>
#include <kukadu/robot.hpp>

using namespace std;

namespace kukadu {

    Controller::Controller(StorageSingleton& dbStorage, std::string caption, std::vector<KUKADU_SHARED_PTR<Hardware> > usedHardware,
                           double simulationFailingProbability, bool isSkill, std::string skillName) : storage(dbStorage) {

        isInstalled = false;
        this->isSkill = isSkill;
        this->skillName = skillName;
        this->usedHardware = usedHardware;

        std::replace(caption.begin(), caption.end(), ' ', '_');

        isShutUp = true;

        std::replace(caption.begin(), caption.end(), ' ', '_');

        this->caption = caption;
        this->simulation = false;

        this->simulationFailingProbability = simulationFailingProbability;

        controllerId = CONTROLLER_ID_NOT_FOUND;

    }

    std::vector<KUKADU_SHARED_PTR<Hardware> > Controller::getUsedHardware() {
        return usedHardware;
    }

    void Controller::createSkillFromThis(std::string skillName) {
        this->isSkill = true;
        this->skillName = skillName;

        auto usedHw = getUsedHardware();
        if(storage.checkLabelExists("skills", "label", skillName) && RobotConfiguration::configurationExists(usedHw)) {

            int configId = RobotConfiguration::getConfigurationId(usedHw);
            auto skillId = storage.getCachedLabelId("skills", "skill_id", "label", skillName);

            stringstream s;

            s << "SELECT count(*) as c FROM skills_robot WHERE skill_id = " << skillId;
            s << " and robot_config_id = " << configId;

            auto result = storage.executeQuery(s.str());
            result->next();
            if(result->getInt("c") == 0) {
                s.str("");
                s << "INSERT INTO skills_robot (skill_id, robot_config_id) VALUES (" << skillId << ", " << configId << ")";
                storage.executeStatementPriority(s.str());
            } else {
                throw KukaduException("(Controller) skill with provided name and robot configuration already exists");
            }
        }  else if (storage.checkLabelExists("skills", "label", skillName)) {
            auto idForNewConfig = storage.getNextIdInTable("robot_config", "robot_config_id");

            stringstream s;
            s << "INSERT INTO robot_config (robot_config_id, hardware_instance_id, order_id) VALUES ";
            unsigned int i = 0;
            for(; i < usedHw.size()-1; i++){
                s << "(" << idForNewConfig << ", " << (usedHw.at(i))->getHardwareInstance() << ", " << i+1 << "), ";
            }
            s << "(" << idForNewConfig << ", "  << (usedHw.at(i))->getHardwareInstance() << ", " << i+1 << ")";

            storage.executeStatementPriority(s.str());
            s.str(string());

            auto skillId = storage.getCachedLabelId("skills", "skill_id", "label", skillName);
            s << "INSERT INTO skills_robot (skill_id, robot_config_id) VALUES (" << skillId << ", " << idForNewConfig << ")";
            storage.executeStatementPriority(s.str());
        } else if (RobotConfiguration::configurationExists(usedHw)) {
            int configId = RobotConfiguration::getConfigurationId(usedHw);
            auto controllerId = getControllerId();

            stringstream s;
            s << "insert into skills(label, controller_type) values('" << skillName << "', " << controllerId << ")";
            storage.executeStatementPriority(s.str());

            s.str(string());
            auto skillId = storage.getCachedLabelId("skills", "skill_id", "label", skillName);
            s << "INSERT INTO skills_robot (skill_id, robot_config_id) VALUES (" << skillId << ", " << configId << ")";
            storage.executeStatementPriority(s.str());
        } else {
            auto idForNewConfig = storage.getNextIdInTable("robot_config", "robot_config_id");

            stringstream s;
            s << "INSERT INTO robot_config (robot_config_id, hardware_instance_id, order_id) VALUES ";
            unsigned int i = 0;
            for(; i < usedHw.size()-1; i++){
                s << "(" << idForNewConfig << ", " << (usedHw.at(i))->getHardwareInstance() << ", " << i+1 << "), ";
            }
            s << "(" << idForNewConfig << ", "  << (usedHw.at(i))->getHardwareInstance() << ", " << i+1 << ")";

            storage.executeStatementPriority(s.str());

            s.str(string());
            auto controllerId = getControllerId();
            s << "insert into skills(label, controller_type) values('" << skillName << "', " << controllerId << ")";
            storage.executeStatementPriority(s.str());

            s.str(string());
            auto skillId = storage.getCachedLabelId("skills", "skill_id", "label", skillName);
            s << "INSERT INTO skills_robot (skill_id, robot_config_id) VALUES (" << skillId << ", " << idForNewConfig << ")";
            storage.executeStatementPriority(s.str());
        }
    }

    int Controller::getControllerId() {
        if(!isInstalled)
            installDb();
        return controllerId;
    }

    KUKADU_SHARED_PTR<ControllerResult> Controller::execute() {

        KUKADU_MODULE_START_USAGE();

        if(!isInstalled)
            installDb();

        auto& stSingleton = SensorStorageSingleton::get();

        auto usedHardware = getUsedHardware();
        stSingleton.registerHardware(usedHardware);
        stSingleton.initiateStorage(usedHardware);

        int skillId = CONTROLLER_ID_NOT_FOUND;
        long long int startTime = getCurrentTime();

        stringstream s;
        s << "insert into controller_executions(controller_id, start_timestamp, end_timestamp, successful) values(" << controllerId << ", " << startTime << ", null, false)";
        storage.executeStatementPriority(s.str());

        if(isSkill) {
            s.str("");
            skillId = storage.getCachedLabelId("skills", "skill_id", "label", skillName);
            s << "insert into skill_executions(skill_id, start_timestamp, end_timestamp, successful) values(" << skillId << ", "
              << startTime << ", null, false)";
            storage.executeStatementPriority(s.str());
        }

        auto retVal = executeInternal();

        stSingleton.stopStorage(usedHardware);

        s.str("");
        s << "update controller_executions set end_timestamp = " << getCurrentTime() << ", successful = true where controller_id = " <<
             controllerId << " and start_timestamp = " << startTime;

        if(isSkill) {
            auto skillSuccessful = getLastSkillExecutionSuccessful();
            s.str("");
            s << "update skill_executions set end_timestamp = " << getCurrentTime() <<
                 ", successful = " << ((skillSuccessful) ? "true" : "false") <<
                 " where skill_id = " <<
                 skillId << " and start_timestamp = " << startTime;
            storage.executeStatementPriority(s.str());
        }

        storage.executeStatementPriority(s.str());

        KUKADU_MODULE_END_USAGE();

        return retVal;

    }

    bool Controller::isControllerInstalled() {

        try { int controllerIdTmp = storage.getCachedLabelId("controller_types", "controller_id", "controller_implementation_class", getClassName()); controllerId = controllerIdTmp; } catch(KukaduException& ex) {}
        if(controllerId != CONTROLLER_ID_NOT_FOUND)
            return true;
        else
            return false;

    }

    void Controller::installDb() {

        if(!isControllerInstalled()) {

            auto augmentedInfo = getAugmentedInfoTableName();

            auto insertSql = "insert into controller_types(controller_implementation_class, augmented_info_table) values('" + getClassName() + "', '" + ((augmentedInfo.first) ? augmentedInfo.second : "") + "')";
            storage.executeStatementPriority(insertSql);

            controllerId = storage.getCachedLabelId("controller_types", "controller_id", "controller_implementation_class", getClassName());

        }

        isInstalled = true;

    }

    std::pair<bool, std::string> Controller::getAugmentedInfoTableName() {
        return {false, ""};
    }

    bool Controller::requiresGrasp() {
        if(!isInstalled)
            installDb();
        return requiresGraspInternal();
    }

    bool Controller::producesGrasp() {
        if(!isInstalled)
            installDb();
        return producesGraspInternal();
    }

    void Controller::initialize() {

    }

    std::string Controller::getCaption() {
        return caption;
    }

    void Controller::setSimulationMode(bool simulationMode) {
        KUKADU_MODULE_START_USAGE();
        simulation = simulationMode;
        setSimulationModeInChain(simulationMode);
        KUKADU_MODULE_END_USAGE();
    }

    bool Controller::getSimulationMode() {
        return simulation;
    }

    void Controller::shutUp() {
        KUKADU_MODULE_START_USAGE();
        isShutUp = true;
        KUKADU_MODULE_END_USAGE();
    }

    void Controller::startTalking() {
        KUKADU_MODULE_START_USAGE();
        isShutUp = false;
        KUKADU_MODULE_END_USAGE();
    }

    double Controller::getSimFailingProb() {
        return simulationFailingProbability;
    }

    bool Controller::getIsSkill() {
        return isSkill;
    }

    bool Controller::getLastSkillExecutionSuccessful() {
        if(!getIsSkill())
            throw KukaduException("(Controller) controller does not represent a skill");
        return true;
    }

    /****************** private functions ******************************/

    void Controller::setSimulationModeInChain(bool simulationMode) {

    }

    /****************** end ********************************************/


    /*****************CartesianPtp *************************/
    CartesianPtp::CartesianPtp(StorageSingleton& storage, KUKADU_SHARED_PTR<ControlQueue> leftQueue)
        : Controller(storage, "cartesian_move_skill_wrapper", {leftQueue}, 0.01) {

        this->leftQueue = leftQueue;

        geometry_msgs::Pose nextPose;
        nextPose.position.x = 0.5;
        nextPose.position.y = 1.2;
        nextPose.position.z = 0.9;

        nextPose.orientation.x = -0.06;
        nextPose.orientation.y = -0.14;
        nextPose.orientation.z = -0.29;
        nextPose.orientation.w = 0.94;

        this->cartesians = nextPose;
    }

    bool CartesianPtp::requiresGraspInternal() {
        return false;
    }

    bool CartesianPtp::producesGraspInternal() {
        return false;
    }

    void CartesianPtp::setCartesians(geometry_msgs::Pose cartesians) {
        this->cartesians = cartesians;
    }

    std::shared_ptr<ControllerResult> CartesianPtp::executeInternal() {
        leftQueue->cartesianPtp(this->cartesians);
        return nullptr;
    }

    std::string CartesianPtp::getClassName() {
        return "CartesianPtp";
    }

    void CartesianPtp::createSkillFromThisInternal(std::string skillName) {
        // nothing to do
    }

    /************************* JointPtp *********************************/
    JointPtp::JointPtp(StorageSingleton& storage, KUKADU_SHARED_PTR<JointHardware> hardware)
        : Controller(storage, "JointPtp", {hardware}, 0.01) {

        this->hardware = hardware;
        this->joints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    }

    bool JointPtp::requiresGraspInternal() {
        return false;
    }

    bool JointPtp::producesGraspInternal() {
        return false;
    }

    void JointPtp::setJoints(std::vector<double> joints) {
        this->joints = joints;
    }

    std::shared_ptr<ControllerResult> JointPtp::executeInternal() {

        if(hardware)
            hardware->jointPtp(stdToArmadilloVec(this->joints));

        return nullptr;

    }

    std::string JointPtp::getClassName() {
        return "JointPtp";
    }

    void JointPtp::createSkillFromThisInternal(std::string skillName) {
        // nothing to do
    }

    std::vector<KUKADU_SHARED_PTR<Hardware> > mergeHardware(std::vector<KUKADU_SHARED_PTR<kukadu::Controller> > controllers) {

        std::vector<KUKADU_SHARED_PTR<Hardware> > combinedHardware;
        for(auto& controller : controllers) {
            auto hw = controller->getUsedHardware();
            combinedHardware.insert(combinedHardware.end(), hw.begin(), hw.end());
        }\

        return combinedHardware;

    }

    LocalizeObject::LocalizeObject(StorageSingleton& dbStorage, KUKADU_SHARED_PTR<Kinect> hardware)
        : Controller(dbStorage, "LocalizeObject", {hardware}, 0.01) {

        //loc = make_shared<PCBlobDetector>(hardware);
        loc = make_shared<PCBlobDetector>(hardware, dbStorage, "origin", stdToArmadilloVec({0.7, 0.3, 0.04}), 0.3, 0.4, false);
        this->objectName = "something";
    }

    void LocalizeObject::createSkillFromThisInternal(std::string skillName) {
        // nothing additonal to do
    }

    bool LocalizeObject::requiresGraspInternal() {
        return false;
    }

    bool LocalizeObject::producesGraspInternal() {
        return false;
    }

    void LocalizeObject::setObjectToLoad(std::string objectToLoad){
        this->objectName = objectToLoad;
    }

    KUKADU_SHARED_PTR<ControllerResult> LocalizeObject::executeInternal() {
        auto pos = loc->localizeObject(this->objectName);
        cout << pos.position.x << " " << pos.position.y << " " << pos.position.z << endl;
        return nullptr;
    }

    void LocalizeObject::setCenterX(double x) {
        loc->setCenterX(x);
    }

    void LocalizeObject::setCenterY(double y) {
        loc->setCenterY(y);
    }

    void LocalizeObject::setCenterZ(double z) {
        loc->setCenterZ(z);
    }

    void LocalizeObject::setBoxDimX(double x) {
        loc->setBoxDimX(x);
    }

    void LocalizeObject::setBoxDimY(double y) {
        loc->setBoxDimX(y);
    }

    std::string LocalizeObject::getClassName() {
        return "LocalizeObject";
    }

    KinestheticTeaching::KinestheticTeaching(StorageSingleton &storage, KUKADU_SHARED_PTR<JointHardware> hardware)
            : Controller(storage, "KinestheticTeaching", {hardware}, 0.01) {

    }

    bool KinestheticTeaching::requiresGraspInternal() {
        return false;
    }

    bool KinestheticTeaching::producesGraspInternal() {
        return false;
    }


    std::shared_ptr<ControllerResult> KinestheticTeaching::executeInternal() {
        std::string storeDir = "/tmp/kukadu_demo/";

        cout << "starting measurement" << endl;
        //HardwareFactory::setSimulation(false);
        auto& hardwareFactory = HardwareFactory::get();
        auto leftQueue = KUKADU_DYNAMIC_POINTER_CAST<ControlQueue>(hardwareFactory.loadHardware("kukie_left_arm"));
        leftQueue->install();
        leftQueue->start();
        auto rightQueue = KUKADU_DYNAMIC_POINTER_CAST<ControlQueue>(hardwareFactory.loadHardware("kukie_right_arm"));
        rightQueue->install();
        rightQueue->start();
        auto leftHand = KUKADU_DYNAMIC_POINTER_CAST<GenericHand>(hardwareFactory.loadHardware("kukiehand_left"));
        leftHand->install();
        leftHand->start();
        auto rightHand = KUKADU_DYNAMIC_POINTER_CAST<GenericHand>(hardwareFactory.loadHardware("kukiehand_right"));
        rightHand->install();
        rightHand->start();

        vector<KUKADU_SHARED_PTR<ControlQueue> > queueVectors;
        queueVectors.push_back(leftQueue);
        queueVectors.push_back(rightQueue);

        vector<KUKADU_SHARED_PTR<GenericHand> > handVectors;
        handVectors.push_back(leftHand);
        handVectors.push_back(rightHand);


        //move to this position to make execution possible
        leftQueue->jointPtp({-0.7, 0.7, 1.5, -1.74, -1.85, 1.27, 0.71});

        SensorStorage sensorStorage(storage, queueVectors, handVectors, 1000);
        sensorStorage.setExportMode(SensorStorage::STORE_RBT_CART_POS | SensorStorage::STORE_RBT_JNT_POS);
        sensorStorage.startDataStorage(storeDir);
        cout << "measurement started" << endl;

        ros::Rate r(2);
        r.sleep();

        leftQueue->jointPtp({-1.5, 1.56, 2.33, -1.74, -1.85, 1.27, 0.71});

        sensorStorage.stopDataStorage();


        leftQueue->stop();
        rightQueue->stop();
        rightHand->stop();
        leftHand->stop();
        return nullptr;
    }

    std::string KinestheticTeaching::getClassName() {
        return "KinestheticTeaching";
    }

    void KinestheticTeaching::createSkillFromThisInternal(std::string skillName) {
        // nothing to do
    }



}
