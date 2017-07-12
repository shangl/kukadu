#include <kukadu/robot/kukiehand.hpp>
#include <kukadu/robot/kukiequeue.hpp>
#include <kukadu/robot/hardwarefactory.hpp>

using namespace std;

namespace kukadu {

std::map<std::string, std::function<KUKADU_SHARED_PTR<Hardware>(StorageSingleton&, std::string, bool)> > HardwareFactory::hardwareFactories{
    {
        "KukieControlQueue", [](StorageSingleton& storage, std::string hardwareName, bool simulation) {
            return make_shared<KukieControlQueue>(storage, hardwareName, simulation);
        }
    },
    {
        "KukieHand", [](StorageSingleton& storage, std::string hardwareName, bool simulation) {
            return make_shared<KukieHand>(storage, hardwareName, simulation);
        }
    }
};

HardwareFactory::HardwareFactory() : storage(StorageSingleton::get()) {

}

HardwareFactory& HardwareFactory::get() {
    static HardwareFactory instance;
    return instance;
}

KUKADU_SHARED_PTR<Hardware> HardwareFactory::loadHardware(std::string hardwareName) {

    if(createdHardware[hardwareName])
        return createdHardware[hardwareName];

    // if the hardware was not created yet
    auto hardwareId = storage.getCachedLabelId("hardware_instances", "hardware_id", "instance_name", hardwareName);
    auto className = storage.getCachedLabel("hardware", "hardware_id", "hardware_name", hardwareId);


    auto& storage = StorageSingleton::get();
    auto tmp = hardwareFactories[className](storage, hardwareName, true);

    auto& created = createdHardware[hardwareName] = hardwareFactories[className](StorageSingleton::get(), hardwareName, true);

    return created;

}

std::vector<std::string> HardwareFactory::listAvailableHardware() {

    vector<string> hardwareList;
    auto result = storage.executeQuery("select distinct(instance_name) from hardware_instances order by instance_name");
    while(result->next())
        hardwareList.push_back(result->getString("instance_name"));
    return hardwareList;

}

bool HardwareFactory::hardwareExists(std::string hardwareName) {

    auto hardwareList = listAvailableHardware();
    if(std::find(hardwareList.begin(), hardwareList.end(), hardwareName) != hardwareList.end())
        return true;
    return false;

}

}
