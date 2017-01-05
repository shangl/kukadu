#include <sstream>
#include <kukadu/robot/hardware.hpp>

using namespace std;

namespace kukadu {

    Hardware::Hardware(StorageSingleton& dbStorage, int hardwareType, std::string hardwareTypeName, int hardwareInstanceId, std::string hardwareInstanceName) :
        StorageHolder(dbStorage) {
        this->hardwareType = hardwareType;
        this->hardwareTypeName = hardwareTypeName;
        this->hardwareInstanceId = hardwareInstanceId;
        this->hardwareInstanceName = hardwareInstanceName;
        registerHardware();
    }

    void Hardware::registerHardware() {

    }

    void Hardware::install() {
        installHardwareType();
        installHardwareInstance();
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
            throw KukaduException("(Hardware) ^ hardware type id does not match the stored information");

        if(!alreadyExists) {

            stringstream s;
            s << "insert into hardware(hardware_id, name) values(" << getHardwareType() << ", '" << getHardwareTypeName() << "')";
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
        return storage.getCachedLabelId("hardware", "hardware_id", "name", typeName);
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
        s << "select name from hardware_instances as inst inner join hardware as har on inst.hardware_id = har.hardware_id where instance_name = " << instanceName;
        auto typeRes = storage.executeQuery(s.str());
        if(typeRes->next())
            return typeRes->getString("name");
        else
            throw KukaduException("(Hardware) failed to load hardware type name");
    }

    int Hardware::loadOrCreateInstanceIdFromName(const std::string& instanceName) {
        try {
            return loadInstanceIdFromName(instanceName);
        } catch(KukaduException& ex) {
            return StorageSingleton::get().getNextIdInTable("hardware_instances", "hardware_id");
        }
        return -1;
    }

}
