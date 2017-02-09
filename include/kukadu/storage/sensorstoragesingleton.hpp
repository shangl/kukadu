#ifndef KUKADU_SENSORSTORAGESINGLETON_H
#define KUKADU_SENSORSTORAGESINGLETON_H

#include <map>
#include <mutex>
#include <vector>
#include <string>
#include <kukadu/robot/hardware.hpp>
#include <kukadu/types/kukadutypes.hpp>

namespace kukadu {

    class SensorStorageSingleton {

    private:

        std::mutex instanceMutex;

        std::vector<std::string> registeredInstanceNames;
        std::map<std::string, KUKADU_SHARED_PTR<Hardware> > registeredHardware;
        // stores whether a storage thread is still running and the reference to the running thread
        std::map<std::string, std::pair<bool, KUKADU_SHARED_PTR<kukadu_thread> > > startedThreads;
        std::map<std::string, int> startedCount;

        SensorStorageSingleton();

    public:

        static SensorStorageSingleton& get();

        void registerHardware(KUKADU_SHARED_PTR<Hardware> hardware);
        void registerHardware(std::vector<KUKADU_SHARED_PTR<Hardware> > hardware);

        KUKADU_SHARED_PTR<Hardware> getRegisteredHardware(std::string name);

        void initiateStorageAllRegistered();

        /* returns whether the storage was started (true) or if it already was running (false) */
        bool initiateStorage(std::string instanceName);

        /* returns whether the storage was started (true) or if it already was running (false) for each instance */
        std::vector<bool> initiateStorage(std::vector<std::string> instanceNames);
        std::vector<bool> initiateStorage(std::vector<KUKADU_SHARED_PTR<Hardware> > hardware);

        // always stops everything - can be dangerous
        void stopStorageAll();

        // stops an instance only if stop was called as often as the start was called
        void stopStorage(std::vector<std::string> instanceNames);
        void stopStorage(std::vector<KUKADU_SHARED_PTR<Hardware> > hardware);

    };

}

#endif
