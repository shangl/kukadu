#include <sstream>
#include <algorithm>
#include <kukadu/robot/hardware.hpp>

using namespace std;
using namespace arma;

namespace kukadu {

    RobotConfiguration::RobotConfiguration(StorageSingleton& storage, int configurationId) : StorageHolder(storage) {
        stringstream s;
        s << "SELECT * FROM `robot_config` WHERE robot_config_id=" << configurationId << " ORDER BY order_id ASC";
        auto robotConfig = storage.executeQuery(s.str());

        while (robotConfig->next()) {
            auto hardwareId = robotConfig->getInt("hardware_Instance_id");
            hardwareIds.push_back(hardwareId);
        }
    }

    bool RobotConfiguration::containsHardwareInOrder(std::vector<KUKADU_SHARED_PTR<Hardware> > hardwareComponents) {
        if(hardwareComponents.size() != hardwareIds.size()) {
            return false;
        }

        auto argumentIterator = hardwareComponents.begin();
        auto propertyIterator = hardwareIds.begin();

        for(;(*argumentIterator)->getHardwareInstance() == *propertyIterator && argumentIterator != --hardwareComponents.end(); argumentIterator++, propertyIterator++);

        return (*argumentIterator)->getHardwareInstance() == *propertyIterator;
    }

    bool RobotConfiguration::containsHardwareAsSet(std::vector<KUKADU_SHARED_PTR<Hardware> > hardwareComponents) {
        auto propertyIterator = hardwareIds.begin();
        auto argumentContainsAllProperties = hardwareComponents.size() == hardwareIds.size();

        for(; propertyIterator != hardwareIds.end() && argumentContainsAllProperties; ++propertyIterator) {
            auto elementFound = false;
            for(auto argumentIterator = hardwareComponents.begin(); !elementFound && argumentIterator != hardwareComponents.end(); ++argumentIterator){
                elementFound = (*argumentIterator)->getHardwareInstance()==*propertyIterator;
            }

            argumentContainsAllProperties &= elementFound;
        }

        return argumentContainsAllProperties;
    }

    bool RobotConfiguration::containsHardware(std::vector<KUKADU_SHARED_PTR<Hardware> > hardwareComponents) {

        if(hardwareComponents.size() < hardwareIds.size())
            return false;

        for(auto& desiredHwId : hardwareIds) {
            bool found = false;
            for(auto& hw : hardwareComponents) {
                if(hw->getHardwareInstance() == desiredHwId)
                    found = true;
            }
            if(!found)
                return false;
        }

        return true;

    }

    bool RobotConfiguration::configurationExists(std::vector<std::shared_ptr<kukadu::Hardware>> hardware) {
        return getConfigurationId(hardware) != -1;
    }

    int RobotConfiguration::getConfigurationId(std::vector<std::shared_ptr<kukadu::Hardware>> hardware) {
        auto& storage = kukadu::StorageSingleton::get();

        vector<int> usedHwIds;
        for(auto& hw : hardware) {
            if(std::find(usedHwIds.begin(), usedHwIds.end(), hw->getHardwareInstance()) == usedHwIds.end()) {
                usedHwIds.push_back(hw->getHardwareInstance());
            }
        }

        stringstream s;
        s << "select distinct robot_config_id from robot_config";

        int i = 1;
        for(auto hwIdsIterator = usedHwIds.begin(); hwIdsIterator != usedHwIds.end(); ++hwIdsIterator, ++i){
            if (hwIdsIterator == usedHwIds.begin()){
                s << " where ";
            }

            if(hwIdsIterator == --usedHwIds.end()) {
                s << "order_id=" << i << " and hardware_instance_id=" << *hwIdsIterator;
            } else {
                s << "order_id=" << i << " and hardware_instance_id=" << *hwIdsIterator << " and ";
            }
        }

        s << " and order_id <> " << i;

        stringstream t;
        t << "SELECT COUNT(*) as count FROM (" << s.str() << ") tmp";


        auto amountOfConfigurations = storage.executeQuery(t.str());
        amountOfConfigurations->next();

        //no config exists
        if(amountOfConfigurations->getInt("count") == 0){
            return -1;
        } else {
            auto configResult = storage.executeQuery(s.str());
            configResult->next();
            return configResult->getInt("robot_config_id");
        }
    }



    Hardware::Hardware(StorageSingleton& dbStorage, int hardwareClass, int hardwareType, std::string hardwareTypeName, int hardwareInstanceId, std::string hardwareInstanceName) :
        StorageHolder(dbStorage) {
        this->hardwareType = hardwareType;
        this->hardwareClass = hardwareClass;
        this->hardwareTypeName = hardwareTypeName;
        this->hardwareInstanceId = hardwareInstanceId;
        this->hardwareInstanceName = hardwareInstanceName;
        registerHardware();

        this->installed = false;
        this->started = false;
    }

    void Hardware::registerHardware() {

    }

    void Hardware::install() {
        if(!this->installed) {
            installHardwareType();
            installHardwareInstance();
            this->installed = true;
        }
    }

    void Hardware::start() {

        if(!this->started) {
            startInternal();
            this->started = true;
        }

    }

    bool Hardware::isStarted() {
        return started;
    }

    void Hardware::stop() {
        if (this->started) {
            stopInternal();
            this->started = false;
        }
    }

    void Hardware::installHardwareType() {

        bool allOk = false;
        bool alreadyExists = false;
        try {

            // make a sanity check --> if the type already in the database --> check if it corresponds to the passed id
            // if it does not exist yet --> insert it
            int typeId = loadTypeIdFromName(getHardwareTypeName());
            if(typeId == hardwareType)
                allOk = true;
            alreadyExists = true;

        } catch(KukaduException& ex) { allOk = true; }

        if(!allOk)
            throw KukaduException("(Hardware) hardware type id does not match the stored information");

        if(!alreadyExists) {

            stringstream s;
            s << "insert into hardware(hardware_id, hardware_name, hardware_class) values(" << getHardwareType() << ", '"
              << getHardwareTypeName() << "', " << getHardwareClass() << ")";

            getStorage().executeStatementPriority(s.str());

            installHardwareTypeInternal();
        }

    }

    void Hardware::installHardwareInstance() {

        bool allOk = false;
        bool alreadyExists = false;
        try {

            // make a sanity check --> if the type already in the database --> check if it corresponds to the passed id
            // if it does not exist yet --> insert it
            int typeId = loadInstanceIdFromName(getHardwareInstanceName());
            if(typeId == hardwareInstanceId)
                allOk = true;
            alreadyExists = true;

        } catch(KukaduException& ex) { allOk = true; }

        if(!allOk)
            throw KukaduException("(Hardware) provided hardware intstance id does not match the stored information");

        if(!alreadyExists) {

            stringstream s;
            s << "insert into hardware_instances(instance_id, instance_name, hardware_id) values(" << getHardwareInstance() << ", '" << getHardwareInstanceName() << "', " << getHardwareType() << ")";
            getStorage().executeStatementPriority(s.str());

            // if it should be inserted --> also call the implementation specific installation
            installHardwareInstanceInternal();

        }

    }

    int Hardware::getHardwareClass() {
        return hardwareClass;
    }

    int Hardware::getHardwareType() {
        return hardwareType;
    }

    int Hardware::getHardwareInstance() {
        return hardwareInstanceId;
    }

    std::string Hardware::getHardwareTypeName() {
        return hardwareTypeName;
    }

    std::string Hardware::getHardwareInstanceName() {
        return hardwareInstanceName;
    }

    int Hardware::loadTypeIdFromName(const std::string& typeName) {
        auto& storage = StorageSingleton::get();
        return storage.getCachedLabelId("hardware", "hardware_id", "hardware_name", typeName);
    }

    int Hardware::loadInstanceIdFromName(const std::string& instanceName) {
        auto& storage = StorageSingleton::get();
        return storage.getCachedLabelId("hardware_instances", "instance_id", "instance_name", instanceName);
    }

    int Hardware::loadTypeIdFromInstanceId(const int& instanceId) {
        auto& storage = StorageSingleton::get();
        stringstream s;
        s << "select hardware_id from hardware_instances where instance_id = " << instanceId;
        auto typeRes = storage.executeQuery(s.str());
        if(typeRes->next())
            return typeRes->getInt("hardware_id");
        else
            throw KukaduException("(Hardware) requested instance id does not exist");
    }

    int Hardware::loadTypeIdFromInstanceName(const std::string& instanceName) {
        auto& storage = StorageSingleton::get();
        stringstream s;
        s << "select hardware_id from hardware_instances where instance_name = " << instanceName;
        auto typeRes = storage.executeQuery(s.str());
        if(typeRes->next())
            return typeRes->getInt("hardware_id");
        else
            throw KukaduException("(Hardware) requested instance id does not exist");
    }

    std::string Hardware::loadTypeNameFromInstanceName(const std::string& instanceName) {
        auto& storage = StorageSingleton::get();
        stringstream s;
        s << "select hardware_name from hardware_instances as inst inner join hardware as har on inst.hardware_id = har.hardware_id where instance_name = " << instanceName;
        auto typeRes = storage.executeQuery(s.str());
        if(typeRes->next())
            return typeRes->getString("hardware_name");
        else
            throw KukaduException("(Hardware) failed to load hardware type name");
    }

    int Hardware::loadOrCreateTypeIdFromName(const std::string& typeName) {
        try {
            return loadTypeIdFromName(typeName);
        } catch(KukaduException& ex) {
            return StorageSingleton::get().getNextIdInTable("hardware", "hardware_id");
        }
        return -1;
    }

    int Hardware::loadOrCreateInstanceIdFromName(const std::string& instanceName) {
        try {
            return loadInstanceIdFromName(instanceName);
        } catch(KukaduException& ex) {
            return StorageSingleton::get().getNextIdInTable("hardware_instances", "instance_id");
        }
        return -1;
    }

    JointHardware::JointHardware(StorageSingleton& dbStorage, int hardwareClass, int hardwareType, std::string hardwareTypeName,
                                 int hardwareInstanceId, std::string hardwareInstanceName) : Hardware(dbStorage, hardwareClass, hardwareType, hardwareTypeName, hardwareInstanceId, hardwareInstanceName) {

    }

    vector<std::pair<long long int, arma::vec> > JointHardware::loadData(long long int startTime, long long int endTime, long long maxTotalDuration,
                                                                         long long int maxTimeStepDifference) {

        if(startTime < 0 || endTime < 0 || maxTimeStepDifference < 0)
            throw KukaduException("(JointHardware) provided time must be positive");

        vector<std::pair<long long int, arma::vec> > loadedData;

        auto& storage = getStorage();
        auto robotId = getHardwareInstance();
        auto jointIds = getJointIds();

        stringstream s;
        s << "select joint_id, time_stamp, position, velocity, acceleration, frc from joint_mes where " <<
             "robot_id = " << robotId << " and joint_id in (";
        for(int i = 0; i < jointIds.size(); ++i) {
            s << jointIds.at(i);
            if(i < (jointIds.size() - 1))
                s << ",";
        }
        s << ") and time_stamp >= " << startTime;

        if(endTime > 0)
            s << " and time_stamp <= " << endTime;
        else
            s << " and time_stamp <= " << (startTime + maxTotalDuration);

        s << " order by time_stamp asc, joint_id asc";

        auto degOfFreedom = getDegreesOfFreedom();

        vector<bool> posAlreadyLoaded;
        for(int i = 0; i < degOfFreedom; ++i)
            posAlreadyLoaded.push_back(false);

        vec nextJointPosVec(degOfFreedom);
        vec nextJointVelVec(degOfFreedom);
        vec nextJointAccVec(degOfFreedom);
        vec nextJointFrcVec(degOfFreedom);

        int currentJointId = -1;
        long long int currentTimeStamp = -1;
        double currentPos = 0.0;
        double currentVel = 0.0;
        double currentAcc = 0.0;
        double currentFrc = 0.0;

        long long int prevTimeStamp = -1;

        map<int, int> mapJointIdsToLoadedVec;
        for(int i = 0; i < jointIds.size(); ++i)
            mapJointIdsToLoadedVec[jointIds.at(i)] = i;

        auto jointQuery = storage.executeQuery(s.str());
        while(jointQuery->next()) {

            currentJointId = jointQuery->getInt("joint_id");
            currentTimeStamp = jointQuery->getInt64("time_stamp");
            currentPos = jointQuery->getDouble("position");
            currentVel = jointQuery->getDouble("velocity");
            currentAcc = jointQuery->getDouble("acceleration");
            currentFrc = jointQuery->getDouble("frc");

            // if the time distance is too big - its not one connected measurement anymore
            if(endTime <= 0 && prevTimeStamp != -1 && (currentTimeStamp - prevTimeStamp) > maxTimeStepDifference)
                break;

            // if the timestamp changes, a different sample started --> return it
            if(currentTimeStamp != prevTimeStamp) {

                int allDataLoaded = degOfFreedom;
                // set back the loaded flags and check, if all joints were loaded in the previous sample
                for(int i = 0; i < degOfFreedom; ++i) {
                    if(posAlreadyLoaded.at(i))
                        --allDataLoaded;
                    posAlreadyLoaded.at(i) = false;
                }

                // if not all data points are loaded --> ignore the last data sample
                if(allDataLoaded) { }
                // otherwise add it to the return set
                else {

                    // join all data and add it to the return set
                    vec sample = join_cols(join_cols(join_cols(nextJointPosVec, nextJointVelVec), nextJointAccVec), nextJointFrcVec);
                    loadedData.push_back({prevTimeStamp, sample});

                }

                prevTimeStamp = currentTimeStamp;

            }

            // if the stored joint id is really part of the hardware
            if(mapJointIdsToLoadedVec.find(currentJointId) != mapJointIdsToLoadedVec.end()) {

                auto jointIndex = mapJointIdsToLoadedVec[currentJointId];
                posAlreadyLoaded.at(jointIndex) = true;
                nextJointPosVec(jointIndex) = currentPos;
                nextJointVelVec(jointIndex) = currentVel;
                nextJointAccVec(jointIndex) = currentAcc;
                nextJointFrcVec(jointIndex) = currentFrc;

            }
            // if not --> ignore
            else {

            }

        }

        return loadedData;

    }

    int JointHardware::getJointId(std::string jointName) {

        stringstream s;
        s << "select joint_id from hardware_joints where hardware_instance_id = " << getHardwareInstance() << " and joint_name = \"" << jointName << "\"";
        auto idRes = getStorage().executeQuery(s.str());
        if(idRes->next())
            return idRes->getInt("joint_id");
        else
            throw KukaduException("(JointHardware) searched joint is not part of the robot");

    }

    int JointHardware::getDegreesOfFreedom() {
        return getJointIds().size();
    }

}
