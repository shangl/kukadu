#include <string>
#include <ros/ros.h>
#include <kukadu/kukadu.hpp>

using namespace std;
using namespace kukadu;

int main(int argc, char** args) {

    ros::init(argc, args, "kukadu_hand_demo"); ros::NodeHandle* node = new ros::NodeHandle(); sleep(1);
    ros::AsyncSpinner spinner(10); spinner.start();

    cout << "setting up database connection" << endl;
    auto& storage = StorageSingleton::get();

    cout << "setting up hand" << endl;
    KukieHand hand(storage, *node, "simulation", "left");

    cout << "installing the hand if it is not there yet" << endl;
    hand.install();

    cout << "moving the hand to position 0" << endl;
    hand.moveJoints(stdToArmadilloVec({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));

    cout << "moving the hand to position 1" << endl;
    hand.moveJoints(stdToArmadilloVec({1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));

    cout << "moving the hand to position 2" << endl;
    hand.moveJoints(stdToArmadilloVec({1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0}));

    cout << "stopping module statistics" << endl;
    ModuleUsageSingleton::get().stopStatisticsModule();

    cout << "waiting until the storage cache is empty" << endl;
    storage.waitForEmptyCache();

    cout << "press a key to stop the program" << endl;
    getchar();

    return EXIT_SUCCESS;

}
