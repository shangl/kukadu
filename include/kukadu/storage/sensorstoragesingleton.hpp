#ifndef KUKADU_SENSORSTORAGESINGLETON_H
#define KUKADU_SENSORSTORAGESINGLETON_H

#include <map>
#include <vector>
#include <string>
#include <kukadu/robot/hardware.hpp>
#include <kukadu/types/kukadutypes.hpp>

namespace kukadu {

    class SensorStorageSingleton {

    private:

        std::vector<std::string> registeredInstanceNames;
        std::map<std::string, KUKADU_SHARED_PTR<Hardware> > registeredHardware;
        // stores whether a storage thread is still running and the reference to the running thread
        std::map<std::string, std::pair<bool, KUKADU_SHARED_PTR<kukadu_thread> > > startedThreads;

        SensorStorageSingleton();

    public:

        static SensorStorageSingleton& get();

        void registerHardware(KUKADU_SHARED_PTR<Hardware> hardware);

        void initiateStorageAllRegistered();
        void initiateStorage(std::string instanceName);
        void initiateStorage(std::vector<std::string> instanceNames);

        void stopStorageAll();
        void stopStorage(std::vector<std::string> instanceNames);

    };

}

#endif
