#ifndef KUKADU_HARDWARE_FACTORY
#define KUKADU_HARDWARE_FACTORY

#include <kukadu/robot/hardware.hpp>
#include <kukadu/storage/storagesingleton.hpp>

namespace kukadu {

    class HardwareFactory {

    private:
        bool simulation = true;

        StorageSingleton& storage;

        static std::map<std::string, std::function<KUKADU_SHARED_PTR<Hardware>(StorageSingleton&, std::string, bool)> > hardwareFactories;

        std::map<std::string, KUKADU_SHARED_PTR<Hardware> > createdHardware;

        HardwareFactory();

    public:

        static HardwareFactory& get();

        KUKADU_SHARED_PTR<Hardware> loadHardware(std::string hardwareName);

        std::vector<std::string> listAvailableHardware();

        bool hardwareExists(std::string hardwareName);

        void setSimulation(bool isSimulation);

        void stopAllCreatedHardware();

    };

}

#endif
