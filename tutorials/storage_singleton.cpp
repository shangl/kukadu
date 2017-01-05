#include <iostream>
#include <ros/ros.h>
#include <kukadu/kukadu.hpp>

using namespace std;
using namespace arma;
using namespace kukadu;

int main(int argc, char** args) {

    string arm = "left";

    ros::init(argc, args, "kukadu"); ros::NodeHandle* node = new ros::NodeHandle(); usleep(1e6);
    ros::AsyncSpinner spinner(10);
    spinner.start();

    StorageSingleton& storage = StorageSingleton::get();

    cout << "setting up an arm" << endl;
    auto simLeftQueue = make_shared<KukieControlQueue>(storage, "robinn", "simulation", arm + string("_arm"), *node);

    cout << "setting up a hand" << endl;
    auto hand = make_shared<KukieHand>(storage, *node, "simulation", "left");

    cout << "installing queue (only required for the first time of using a concrete hardware instance (but no harm in calling it always)" << endl;
    simLeftQueue->install();

    cout << "installing hand" << endl;
    hand->install();

    vector<KUKADU_SHARED_PTR<ControlQueue> > queueVectors;
    queueVectors.push_back(simLeftQueue);

    simLeftQueue->stopCurrentMode();
    simLeftQueue->startQueue();
    simLeftQueue->switchMode(KukieControlQueue::KUKA_JNT_POS_MODE);

    cout << "creating singleton" << endl;
    auto& storageSingleton = SensorStorageSingleton::get();

    cout << "registering queue and hand" << endl;
    storageSingleton.registerHardware(simLeftQueue);
    storageSingleton.registerHardware(hand);

    cout << "initiating the storage" << endl;
    storageSingleton.initiateStorageAllRegistered();

    cout << "press key to stop storage" << endl;
    getchar();

    cout << "stopping the storage" << endl;
    storageSingleton.stopStorageAll();

    cout << "stopping queue" << endl;
    simLeftQueue->stopCurrentMode();
    simLeftQueue->stopQueue();

    cout << "stopping statistics module" << endl;
    ModuleUsageSingleton::get().stopStatisticsModule();

    cout << "waiting until cache is empty" << endl;

    // wait until all the information is stored in the database
    storage.waitForEmptyCache();

    cout << "done - exit" << endl;
    return EXIT_SUCCESS;

}
