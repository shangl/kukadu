#ifndef KUKADU_HARDWARE_H
#define KUKADU_HARDWARE_H

#include <string>
#include <armadillo>
#include <kukadu/utils/types.hpp>
#include <kukadu/storage/storagesingleton.hpp>

namespace kukadu {

    enum hardware_classes {
        HARDWARE_HAND = 100, HARDWARE_ARM = 200, HARDWARE_CAMERA = 300, HARDWARE_DEPTH_CAMERA = 400
    };

    class Hardware : public StorageHolder {

        int hardwareType;
        int hardwareClass;
        int hardwareInstanceId;

        bool started;
        bool installed;

        std::string hardwareTypeName;
        std::string hardwareInstanceName;

        // registers hardware in the SensorSingleton --> required for automatic storage of the sensor data
        // is called in the constructor
        void registerHardware();

        // performs the general installation (adding the typename and id to the hardware table); calls installHardwareTypeInternal() at the end
        virtual void installHardwareType();

        // performs the general installation (adding the typename and id to the hardware_instances table); calls installHardwareInstanceInternal() at the end
        virtual void installHardwareInstance();

    protected:

        virtual void installHardwareTypeInternal() = 0;

        virtual void installHardwareInstanceInternal() = 0;

        virtual void startInternal() = 0;

        virtual void stopInternal() = 0;

    public:

        Hardware(StorageSingleton &dbStorage, int hardwareClass, int hardwareType, std::string hardwareTypeName,
                 int hardwareInstanceId, std::string hardwareInstanceName);

        void install();

        virtual bool isStarted();

        virtual void start();

        virtual void stop();

        int getHardwareType();

        int getHardwareClass();

        int getHardwareInstance();

        std::string getHardwareTypeName();

        std::string getHardwareInstanceName();

        static int loadTypeIdFromName(const std::string &typeName);

        static int loadInstanceIdFromName(const std::string &instanceName);

        static int loadTypeIdFromInstanceId(const int &instanceId);

        static int loadTypeIdFromInstanceName(const std::string &instanceName);

        static int loadOrCreateTypeIdFromName(const std::string &typeName);

        static int loadOrCreateInstanceIdFromName(const std::string &instanceName);

        static std::string loadTypeNameFromInstanceName(const std::string &instanceName);

        virtual void storeCurrentSensorDataToDatabase() = 0;

        virtual double getPreferredPollingFrequency() = 0;

        /* returns a vector of sensor data of the given hardware and the corresponding time */
        /* the maximum duration of the whole exported series is 1 hour (3600000 ms) */
        virtual std::vector<std::pair<long long int, arma::vec> >
        loadData(long long int startTime, long long int endTime,
                 long long int maxTotalDuration = 3600000,
                 long long int maxTimeStepDifference = 5000) = 0;

    };

    class RobotConfiguration : public StorageHolder {

    private:

    public:

        RobotConfiguration(StorageSingleton &storage, int configurationId);

        bool containsHardwareInOrder(std::vector<KUKADU_SHARED_PTR<Hardware> > hardwareComponents);

        bool containsHardwareAsSet(std::vector<KUKADU_SHARED_PTR<Hardware> > hardwareComponents);

        bool containsHardware(std::vector<KUKADU_SHARED_PTR<Hardware> > hardwareComponents);

        static bool configurationExists(std::vector<std::shared_ptr<kukadu::Hardware>> hardware);

        static int getConfigurationId(std::vector<std::shared_ptr<kukadu::Hardware>> hardware);

    private:
        std::vector<int> hardwareIds;
    };

    class JointHardware : public Hardware {

    public:

        JointHardware(StorageSingleton &dbStorage, int hardwareClass, int hardwareType, std::string hardwareTypeName,
                      int hardwareInstanceId, std::string hardwareInstanceName);

        virtual std::vector<std::string> getJointNames() = 0;

        virtual int getJointId(std::string jointName);

        virtual int getDegreesOfFreedom();

        virtual std::vector<int> getJointIds();

        virtual std::vector<int> getJointIds(std::vector<std::string> jointNames);

        /* returns the time and a vector consisting of (joint positions, joint velocities, joint accelerations, joint forces);
         * the order of the joints is given by (order given by getJointIds()) */
        /* maxTimeStepDifference measures how much time is allowed to be between 2 samples in order to separate skills if endTime is not defined and therefore = 0 */
        virtual std::vector<std::pair<long long int, arma::vec> >
        loadData(long long int startTime, long long int endTime,
                 long long int maxTotalDuration = 3600000,
                 long long maxTimeStepDifference = 5000);

        virtual std::vector<kukadu::mes_result> jointPtp(arma::vec joints, double maxForce = -1.0) = 0;

        virtual void startRollBackMode(double) = 0;

        virtual void rollBack(double) = 0;

        virtual void stopJointRollBackMode() = 0;
    };

}

#endif
