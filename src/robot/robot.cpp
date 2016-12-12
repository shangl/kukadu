#include <sstream>
#include <kukadu/robot/robot.hpp>
#include <kukadu/utils/utils.hpp>

using namespace std;

namespace kukadu {

    Robot::Robot(StorageSingleton& storage, int robotId) : dbStorage(storage) {

        this->robotId = robotId;
        this->robotName = loadRobotName(dbStorage, robotId);
        this->degOfFreedom = loadDegOfFreedom(dbStorage, robotId);
        auto loadedJoints = loadRobotJoints(dbStorage, robotId);
        this->robotJoints = loadedJoints.first;
        this->maxJointId = loadedJoints.second;

    }

    Robot::Robot(StorageSingleton& storage, std::string robotName) : dbStorage(storage) {

        this->robotName = robotName;
        this->robotId = loadRobotId(dbStorage, robotName);
        this->degOfFreedom = loadDegOfFreedom(dbStorage, robotId);
        auto loadedJoints = loadRobotJoints(dbStorage, robotId);
        this->robotJoints = loadedJoints.first;
        this->maxJointId = loadedJoints.second;

    }

    // returns map[joint_id] = joint_name and maximum joint_id
    std::pair<std::map<int, std::string>, int> Robot::loadRobotJoints(StorageSingleton& dbStorage, int& robotId) {

        int maxJointId = 0;
        map<int, string> jointsMap;

        stringstream s;
        s << "select joint_id, joint_name from robot_joints where robot_id = " << robotId;
        auto queryRes = dbStorage.executeQuery(s.str());
        while(queryRes->next()) {
            auto jointId = queryRes->getInt("joint_id");
            jointsMap[jointId] = queryRes->getString("joint_name");
            maxJointId = std::max(maxJointId, jointId);
        }
        return {jointsMap, maxJointId};

    }

    void Robot::reload() {

        this->robotId = loadRobotId(dbStorage, robotName);
        this->degOfFreedom = loadDegOfFreedom(dbStorage, robotId);
        auto loadedJoints = loadRobotJoints(dbStorage, robotId);
        this->robotJoints = loadedJoints.first;
        this->maxJointId = loadedJoints.second;

    }

    bool Robot::insertJoint(std::string jointName) {

        if(!mapContainsValue(robotJoints, jointName)) {

            vector<string> stmts;

            stringstream s;
            s << "insert into robot_joints(robot_id, joint_id, joint_name) values(" << robotId << ", " << ++maxJointId << ", \"" << jointName << "\")";
            stmts.push_back(s.str());

            s.str("");
            s << "update robot set deg_of_freedom = " << maxJointId << " where robot_id = " << robotId;
            stmts.push_back(s.str());
            dbStorage.executeStatements(stmts);

            reload();

            return true;

        }

        return false;

    }

    bool Robot::deleteRobot() {

        for(auto& joints : robotJoints)
            deleteJoint(joints.first);

        stringstream s;
        s << "delete from robot where robot_id = " << robotId;
        dbStorage.executeStatement(s.str());

        return true;

    }

    std::string Robot::getJointName(int jointId) {
        return robotJoints[jointId];
    }

    bool Robot::deleteJoint(int jointId) {

        if(robotJoints.find(jointId) != robotJoints.end()) {

            stringstream s;
            s << "delete from robot_joints where robot_id = " << robotId << " and joint_id = " << jointId;
            dbStorage.executeStatement(s.str());
            return true;

        }

        return false;

    }

    std::string Robot::loadRobotName(StorageSingleton& dbStorage, const int& robotId) {

        string robotName;
        stringstream s;
        s << "select robot_name from robot where robot_id = " << robotId;
        auto idQuery = s.str();
        auto idResult = dbStorage.executeQuery(idQuery);
        if(idResult->next())
            robotName = idResult->getString("robot_name");
        else
            throw KukaduException("(Robot) cannot find robot id in database");
        return robotName;

    }

    int Robot::loadRobotId(StorageSingleton& dbStorage, const std::string& robotName) {

        int robotId = 0;
        auto idQuery = "select robot_id from robot where robot_name=\"" + robotName + "\"";
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
        s << "select deg_of_freedom from robot where robot_id = " << robotId;
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

    bool Robot::createRobot(StorageSingleton& dbStorage, std::string robotName) {
        if(!checkRobotExists(dbStorage, robotName)) {
            dbStorage.executeStatement("insert into robot(robot_id, robot_name, deg_of_freedom) values(null, \"" + robotName + "\", 0)");
            return true;
        }
        return false;
    }

}
