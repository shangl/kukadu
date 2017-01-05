#ifndef KUKADU_HARDWARE_H
#define KUKADU_HARDWARE_H

#include <string>
#include <kukadu/storage/storagesingleton.hpp>

namespace kukadu {

    enum hardware_classes{HARDWARE_HAND = 100, HARDWARE_ARM = 200, HARDWARE_CAMERA = 300, HARDWARE_DEPTH_CAMERA = 400};

    class Hardware : public StorageHolder {

        int hardwareType;
        int hardwareClass;
        int hardwareInstanceId;

        std::string hardwareTypeName;
        std::string hardwareInstanceName;

        // registers hardware in the SensorSingleton --> required for automatic storage of the sensor data
        // is called in the constructor
        void registerHardware();

        // performs the general installation (adding the typename and id to the hardware table); calls installHardwareTypeInternal() at the end
        virtual void installHardwareType() final;

        // performs the general installation (adding the typename and id to the hardware_instances table); calls installHardwareInstanceInternal() at the end
        virtual void installHardwareInstance() final;

    protected:

        virtual void installHardwareTypeInternal() = 0;
        virtual void installHardwareInstanceInternal() = 0;

    public:

        Hardware(StorageSingleton& dbStorage, int hardwareClass, int hardwareType, std::string hardwareTypeName, int hardwareInstanceId, std::string hardwareInstanceName);

        virtual void install() final;

        int getHardwareType();
        int getHardwareClass();
        int getHardwareInstance();

        std::string getHardwareTypeName();
        std::string getHardwareInstanceName();

        static int loadTypeIdFromName(const std::string& typeName);
        static int loadInstanceIdFromName(const std::string& instanceName);
        static int loadTypeIdFromInstanceId(const int& instanceId);
        static int loadTypeIdFromInstanceName(const std::string& instanceName);

        static int loadOrCreateTypeIdFromName(const std::string& typeName);
        static int loadOrCreateInstanceIdFromName(const std::string& instanceName);

        static std::string loadTypeNameFromInstanceName(const std::string& instanceName);

        virtual void storeCurrentSensorDataToDatabase() = 0;
        virtual double getPreferredPollingFrequency() = 0;

    };

}

#endif
