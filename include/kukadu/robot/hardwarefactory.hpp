#ifndef KUKADU_HARDWARE_FACTORY
#define KUKADU_HARDWARE_FACTORY

#include <mutex>
#include <kukadu/robot/hardware.hpp>
#include <kukadu/storage/storagesingleton.hpp>

namespace kukadu {

    class HardwareFactory {

    private:

        bool simulation = true;

        std::mutex loadHardwareMutex;

        StorageSingleton& storage;

        static std::map<std::string, std::function<KUKADU_SHARED_PTR<Hardware>(StorageSingleton&, std::string, bool)> > hardwareFactories;

        std::map<std::string, KUKADU_SHARED_PTR<Hardware> > createdHardware;

        HardwareFactory();

        // cannot copy hardware factory
        HardwareFactory(const HardwareFactory& cp) : storage(cp.storage) {}
        HardwareFactory& operator=(const HardwareFactory& fac) { return *(new HardwareFactory()); }

    public:

        static HardwareFactory& get();

        KUKADU_SHARED_PTR<Hardware> loadHardware(std::string hardwareName);

        std::vector<std::string> listAvailableHardware();

        bool hardwareExists(std::string hardwareName);

        void setSimulation(bool isSimulation);
        bool getSimulation();

        void stopAllCreatedHardware();

    };

}

#endif
