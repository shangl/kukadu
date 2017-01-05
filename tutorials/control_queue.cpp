#include <kukadu/kukadu.hpp>

using namespace std;
using namespace kukadu;

int main(int argc, char** args) {

    cout << "setting up ros node" << endl;
    ros::init(argc, args, "kukadu_controlqueue_demo"); ros::NodeHandle node; sleep(1);
    ros::AsyncSpinner spinner(10); spinner.start();

    StorageSingleton& storage = StorageSingleton::get();

    cout << "setting up control queue" << endl;
    auto realLeftQueue = make_shared<KukieControlQueue>(storage, "robinn", "simulation", "left_arm", node);
    realLeftQueue->install();

    cout << "starting queue" << endl;
    auto realLqThread = realLeftQueue->startQueue();

    cout << "switching to impedance mode if it is not there yet" << endl;
    if(realLeftQueue->getCurrentMode() != KukieControlQueue::KUKA_JNT_IMP_MODE) {
        realLeftQueue->stopCurrentMode();
        realLeftQueue->switchMode(KukieControlQueue::KUKA_JNT_IMP_MODE);
    }

    // joint point to point movement in order to go to the start position
    // remark: you only define where to go, not how to get there
    realLeftQueue->jointPtp(stdToArmadilloVec({-1.0, 1.0, -0.5, 0.0, 0.0, 0.0, 0.0}));

    // retrieving the current joint state of the robot after the joint
    // point to point movement
    auto startState = realLeftQueue->getCurrentJoints().joints;

    // execution a trajectory for the 3rd joint (i.e. rotation the arm)
    // here you also provide HOW to get to the target
    for(auto currentState = startState; currentState(2) < startState(2) + 1.0; currentState(2) += 0.005) {
        // sending a the next desired position
        realLeftQueue->move(currentState);
        // the queue has an intrinsic clock, so you can wait until the packet has been
        // submit in order to not send the positions too fast
        realLeftQueue->synchronizeToQueue(1);
    }

    cout << "execution done" << endl;
    cout << "press a key to end the program" << endl;
    getchar();

    /****** done with moving? --> clean up everything and quit *******/

    // leaves the mode for robot movement
    realLeftQueue->stopCurrentMode();

    // stops the queue
    realLeftQueue->stopQueue();

    storage.waitForEmptyCache();

    return EXIT_SUCCESS;

}
