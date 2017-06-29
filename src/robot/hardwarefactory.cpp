#include <kukadu/robot/kukiequeue.hpp>
#include <kukadu/robot/hardwarefactory.hpp>

using namespace std;

namespace kukadu {

    std::map<std::string, std::function<KUKADU_SHARED_PTR<Hardware>(StorageSingleton&, std::string, bool)> > HardwareFactory::hardwareFactories{
        {
            "KukieControlQueue", [](StorageSingleton& storage, std::string hardwareName, bool simulation) {
                return make_shared<KukieControlQueue>(storage, hardwareName, simulation);
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

        return hardwareFactories["KukieControlQueue"](StorageSingleton::get(), hardwareName, true);

    }

    std::vector<std::string> HardwareFactory::listAvailableHardware() {

    }

    bool HardwareFactory::hardwareExists(std::string skillName) {

    }

}
