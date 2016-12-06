#ifndef KUKADU_ROBOT_H
#define KUKADU_ROBOT_H

#include <string>
#include <vector>
#include <kukadu/storage/storagesingleton.hpp>

namespace kukadu {

    class Robot {

        int robotId;
        int degOfFreedom;
        std::string robotName;
        std::vector<std::string> robotJoints;

        StorageSingleton& dbStorage;

        static int loadDegOfFreedom(StorageSingleton& dbStorage, const int& robotId);
        static int loadRobotId(StorageSingleton& dbStorage, const std::string& robotName);

        static std::string loadRobotName(StorageSingleton& dbStorage, const int& robotId);

        static std::vector<std::string> loadRobotJoints(StorageSingleton& dbStorage, int& robotId);

    public:

        Robot(StorageSingleton& storage, int robotId);
        Robot(StorageSingleton& storage, std::string robotName);

        int getRobotId();
        int getDegOfFreedom();

        std::string getRobotName();

        static bool checkRobotExists(StorageSingleton& dbStorage, std::string robotName);

    };

}

#endif
