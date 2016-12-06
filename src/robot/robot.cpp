#include <kukadu/robot/robot.hpp>
#include <sstream>

using namespace std;

namespace kukadu {

    Robot::Robot(StorageSingleton& storage, int robotId) : dbStorage(storage) {

        this->robotId = robotId;
        this->robotName = loadRobotName(dbStorage, robotId);
        this->degOfFreedom = loadDegOfFreedom(dbStorage, robotId);
        this->robotJoints = loadRobotJoints(dbStorage, robotId);

    }

    Robot::Robot(StorageSingleton& storage, std::string robotName) : dbStorage(storage) {

        this->robotName = robotName;
        this->robotId = loadRobotId(dbStorage, robotName);
        this->degOfFreedom = loadDegOfFreedom(dbStorage, robotId);
        this->robotJoints = loadRobotJoints(dbStorage, robotId);

    }

    std::vector<std::string> Robot::loadRobotJoints(StorageSingleton& dbStorage, int& robotId) {
        return {};
    }

    std::string Robot::loadRobotName(StorageSingleton& dbStorage, const int& robotId) {

        string robotName;
        auto idQuery = "SELECT robot_name FROM robot WHERE robot_id = " + robotId;
        auto idResult = dbStorage.executeQuery(idQuery);
        if(idResult->next())
            robotName = idResult->getString("robot_name");
        else
            throw KukaduException("(Robot) cannot find robot id in database");
        return robotName;

    }

    int Robot::loadRobotId(StorageSingleton& dbStorage, const std::string& robotName) {

        int robotId = 0;
        auto idQuery = "SELECT robot_id FROM robot WHERE robot_name=\"" + robotName + "\"";
        auto idResult = dbStorage.executeQuery(idQuery);
        if(idResult->next())
            robotId = idResult->getInt("robot_id");
        else
            throw KukaduException("(Robot) cannot find robot id in database");
        return robotId;

    }

    int Robot::getRobotId() {
        return robotId;
    }

    std::string Robot::getRobotName() {
        return robotName;
    }

    int Robot::loadDegOfFreedom(StorageSingleton& dbStorage, const int& robotId) {

        int degOfFreedom = 0;
        stringstream s;
        s << "SELECT deg_of_freedom FROM robot WHERE robot_id = " << robotId;
        auto idQuery = s.str();
        auto idResult = dbStorage.executeQuery(idQuery);
        if(idResult->next())
            degOfFreedom = idResult->getInt("deg_of_freedom");
        else
            throw KukaduException("(Robot) cannot find robot degrees of freedom in database");
        return degOfFreedom;

    }

    int Robot::getDegOfFreedom() {
        return degOfFreedom;
    }

    bool Robot::checkRobotExists(StorageSingleton& dbStorage, std::string robotName) {

        try {
            // try catch shouldnt be used as control structure, but I am too lazy now
            int id = loadRobotId(dbStorage, robotName);
            return true;
        } catch(KukaduException& ex) { }
        return false;

    }

}
