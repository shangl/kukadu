#include <kukadu/vision/kinect.hpp>
#include <kukadu/robot/kukiehand.hpp>
#include <kukadu/robot/kukiequeue.hpp>
#include <kukadu/robot/hardwarefactory.hpp>

using namespace std;

namespace kukadu {

class NoHardware : public Hardware {

protected:

    virtual void installHardwareTypeInternal() { }
    virtual void installHardwareInstanceInternal() { }
    virtual void startInternal() { }
    virtual void stopInternal() { }

public:

    NoHardware(StorageSingleton& nothingStorage) : Hardware(nothingStorage, 0, 0, "no_hardware",
    0, "no_hardware_instance") { }

    virtual void storeCurrentSensorDataToDatabase() { }

    virtual double getPreferredPollingFrequency() { return 1.0; }

    virtual std::vector<std::pair<long long int, arma::vec> >
    loadData(long long int startTime, long long int endTime,
             long long int maxTotalDuration = 3600000,
             long long int maxTimeStepDifference = 5000) {
        return {};
    };

};

std::string createHardwareName(std::string hardware, bool simulate) {
    stringstream s;
    s << hardware << "...sim=" << simulate;
    return s.str();
}

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
    },
    {
        "Kinect", [](StorageSingleton& storage, std::string hardwareName, bool simulation) {
            return make_shared<Kinect>(storage, hardwareName, false);
        }
    },
    {
            "no_hardware", [](StorageSingleton& storage, std::string hardwareName, bool simulation) {
        return make_shared<NoHardware>(storage);
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

    loadHardwareMutex.lock();

    std::string mapHardwareName = createHardwareName(hardwareName, this->simulation);
    if(createdHardware.find(mapHardwareName) != createdHardware.end()) {
        loadHardwareMutex.unlock();
        return createdHardware[mapHardwareName];
    }

    // if the hardware was not created yet
    auto hardwareId = storage.getCachedLabelId("hardware_instances", "hardware_id", "instance_name", hardwareName);
    auto className = storage.getCachedLabel("hardware", "hardware_id", "hardware_name", hardwareId);

    KUKADU_SHARED_PTR<Hardware> created = nullptr;
    auto& storage = StorageSingleton::get();
    if(hardwareFactories.find(className) != hardwareFactories.end()) {
        created = hardwareFactories[className](storage, hardwareName, this->simulation);
        createdHardware[mapHardwareName] = created;
    } else {
        loadHardwareMutex.unlock();
        throw KukaduException("(HardwareFactory) hardware type is not installed to the factory");
    }

    loadHardwareMutex.unlock();

    return created;

}


void HardwareFactory::setSimulation(bool isSimulation) {
    this->simulation = isSimulation;
}

bool HardwareFactory::getSimulation() {
    return simulation;
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

void HardwareFactory::stopAllCreatedHardware() {
    for(auto& created : createdHardware)
        if(created.second)
            created.second->stop();
}

}
