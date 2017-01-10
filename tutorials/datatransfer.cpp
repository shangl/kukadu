#include <ctime>
#include <cmath>
#include <cstdio>
#include <vector>
#include <fstream>
#include <cstdlib>
#include <iostream>
#include <unistd.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <boost/program_options.hpp>
#include <kukadu/kukadu.hpp>

#define DOSIMULATION 1

using namespace std;
using namespace arma;
using namespace kukadu;
namespace po = boost::program_options;

int main(int argc, char** args) {

    if(argc <= 2) {
        cout << "usage: rosrun kukadu datatransfer {left, right} %PATHTOFILE%" << endl;
        return EXIT_FAILURE;
    }

    double az = 48.0;
    double bz = 11.75;

    ros::init(argc, args, "kukadu"); ros::NodeHandle* node = new ros::NodeHandle(); usleep(1e6);
    ros::AsyncSpinner spinner(10);
    spinner.start();

    ModuleUsageSingleton::get().stopStatisticsModule();
    StorageSingleton& storage = StorageSingleton::get();

    auto simLeftQueue = make_shared<KukieControlQueue>(storage, "robinn", "simulation", string(args[1]) + "_arm", *node);
    simLeftQueue->install();

    simLeftQueue->stopCurrentMode();
    simLeftQueue->startQueue();
    simLeftQueue->switchMode(KukieControlQueue::KUKA_JNT_POS_MODE);

    string skillName = "";
    cout << "under which name do you want to store the new skill?" << endl;
    cin >> skillName;

    auto availableSkills = SkillFactory::get().listAvailableSkills();
    if(std::find(availableSkills.begin(), availableSkills.end(), skillName) == availableSkills.end()) {

        auto timeStamps = SensorStorage::transferArmDataToDb(storage, simLeftQueue, resolvePath(string(args[2])));
        cout << "file successfully transferred with within the time stamps " << timeStamps.first << " " << timeStamps.second << endl;

        storage.waitForEmptyCache();

        JointDMPLearner dmpLearn(storage, simLeftQueue, az, bz, timeStamps.first, timeStamps.second);
        auto trainedDmp = dmpLearn.fitTrajectories();
        DMPExecutor trainedExecutor(storage, trainedDmp, simLeftQueue);

        // create the skill
        trainedExecutor.createSkillFromThis(skillName);

    } else
        cout << "the skill name already existed. loading the skill from the database" << endl;

    // execute the skill now
    auto firstDmp = KUKADU_DYNAMIC_POINTER_CAST<DMPExecutor>(SkillFactory::get().loadSkill(skillName, simLeftQueue));
    firstDmp->setExecutionMode(TrajectoryExecutor::EXECUTE_ROBOT);
    firstDmp->execute();

    cout << "stopping queue" << endl;
    simLeftQueue->stopCurrentMode();
    simLeftQueue->stopQueue();

    cout << "waiting until cache is empty" << endl;
    storage.waitForEmptyCache();

    cout << "done - exit" << endl;

    return EXIT_SUCCESS;

}
