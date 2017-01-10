#include <kukadu/control/controller.hpp>
#include <kukadu/storage/moduleusagesingleton.hpp>
#include <kukadu/storage/sensorstoragesingleton.hpp>

using namespace std;

namespace kukadu {

    /****************** public functions *******************************/

    Controller::Controller(StorageSingleton& dbStorage, std::string caption, std::vector<KUKADU_SHARED_PTR<Hardware> > usedHardware, double simulationFailingProbability) : storage(dbStorage) {

        isInstalled = false;
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
        createSkillFromThisInternal(skillName);
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

        long long int startTime = getCurrentTime();

        stringstream s;
        s << "insert into controller_executions(controller_id, start_timestamp, end_timestamp, successful) values(" << controllerId << ", " << startTime << ", null, false)";
        storage.executeStatement(s.str());

        auto retVal = executeInternal();

        s.str("");
        s << "update controller_executions set end_timestamp = " << getCurrentTime() << ", successful = true where controller_id = " << controllerId << " and start_timestamp = " << startTime;

        stSingleton.stopStorage(usedHardware);

        storage.executeStatement(s.str());

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

    /****************** private functions ******************************/

    void Controller::setSimulationModeInChain(bool simulationMode) {

    }

    /****************** end ********************************************/

}
