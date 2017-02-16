#include <map>
#include <kukadu/control/controller.hpp>
#include <kukadu/storage/moduleusagesingleton.hpp>
#include <kukadu/storage/sensorstoragesingleton.hpp>

using namespace std;

namespace kukadu {

    /****************** public functions *******************************/

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

        if(!storage.checkLabelExists("skills", "label", skillName)) {

            auto controllerId = getControllerId();
            auto usedHw = getUsedHardware();

            stringstream s;
            s << "insert into skills(label, controller_type) values('" << skillName << "', " << controllerId << ")";
            storage.executeStatementPriority(s.str());

            auto skillId = storage.getCachedLabelId("skills", "skill_id", "label", skillName);

            s.str("");
            s << "insert into skills_robot(skill_id, hardware_instance_id) values";
            int insertedCount = 0;

            map<int, KUKADU_SHARED_PTR<Hardware> > hardwareMap;

            // get rid of the duplicates
            for(auto& hw : usedHw)
                if(hw)
                    hardwareMap[hw->getHardwareInstance()] = hw;

            for(auto& hwPair : hardwareMap) {

                auto& hw = hwPair.first;
                if(hw) {
                    s << "(" << skillId << ", " << hw << "),";
                    ++insertedCount;
                }

            }

            if(insertedCount) {

                string skillRobotStr = s.str();
                skillRobotStr = skillRobotStr.substr(0, skillRobotStr.length() - 1);
                storage.executeStatementPriority(skillRobotStr);

            }

            createSkillFromThisInternal(skillName);

        } else
            throw KukaduException("(Controller) skill with provided name already exists");

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

}
