#ifndef KUKADU_ROBOT_H
#define KUKADU_ROBOT_H

#include <map>
#include <string>
#include <vector>
#include <utility>
#include <kukadu/storage/storagesingleton.hpp>

namespace kukadu {

    class Robot {

        int robotId;
        int maxJointId;
        int degOfFreedom;
        std::string robotName;
        std::map<int, std::string> robotJoints;

        StorageSingleton& dbStorage;

        static int loadDegOfFreedom(StorageSingleton& dbStorage, const int& robotId);
        static int loadRobotId(StorageSingleton& dbStorage, const std::string& robotName);

        static std::string loadRobotName(StorageSingleton& dbStorage, const int& robotId);

        static std::pair<std::map<int, std::string>, int> loadRobotJoints(StorageSingleton& dbStorage, int& robotId);

        void reload();

    public:

        Robot(StorageSingleton& storage, int robotId);
        Robot(StorageSingleton& storage, std::string robotName);

        bool insertJoint(std::string jointName);

        int getRobotId();
        int getDegOfFreedom();

        bool deleteRobot();
        bool deleteJoint(int jointId);

        std::string getRobotName();
        std::string getJointName(int jointId);

        static bool createRobot(StorageSingleton& dbStorage, std::string robotName);
        static bool checkRobotExists(StorageSingleton& dbStorage, std::string robotName);

    };

}

#endif
