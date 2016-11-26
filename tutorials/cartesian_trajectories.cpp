#include <kukadu/kukadu.hpp>
#include <geometry_msgs/Pose.h>

using namespace std;
using namespace kukadu;

int main(int argc, char** args) {

    cout << "setting up ros node" << endl;
    ros::init(argc, args, "kukadu_planning"); ros::NodeHandle node; sleep(1);
    ros::AsyncSpinner spinner(10); spinner.start();

    StorageSingleton& storage = StorageSingleton::get();

    cout << "setting up control queue" << endl;
    auto realLeftQueue = make_shared<KukieControlQueue>(storage, "robinn", "simulation", "left_arm", node);

    cout << "starting queue" << endl;
    auto realLqThread = realLeftQueue->startQueue();

    cout << "switching to impedance mode if it is not there yet" << endl;
    if(realLeftQueue->getCurrentMode() != KukieControlQueue::KUKA_JNT_IMP_MODE) {
        realLeftQueue->stopCurrentMode();
        realLeftQueue->switchMode(KukieControlQueue::KUKA_JNT_IMP_MODE);
    }

    Komo leftSimKomoPlanner(realLeftQueue, resolvePath("$KUKADU_HOME/external/komo/share/data/kuka/data/iis_robot.kvg"), resolvePath("$KUKADU_HOME/external/komo/share/data/kuka/config/MT.cfg"), "left");

    /****** stuff is set up now --> here you can move as much as you want *******/

    // this places the robot somewhere
    cout << "joint ptp" << endl;
    realLeftQueue->jointPtp({-1.5, 1.56, 2.33, -1.74, -1.85, 1.27, 0.71});

    // some cartesian position from which the trajectory should be started
    cout << "ptp without max force" << endl;
    geometry_msgs::Pose nextPose; nextPose.position.x = 0.21; nextPose.position.y = 1.3; nextPose.position.z = 0.6;
    nextPose.orientation.x = -0.06; nextPose.orientation.y = -0.14; nextPose.orientation.z = -0.29; nextPose.orientation.w = 0.94;

    // creating the cartesian trajectory with some intermediate steps the robot should pass through
    cout << "planning in z direction" << endl;
    auto stepSize = 0.01;
    vector<geometry_msgs::Pose> cartesianTrajectory;
    for(double i = 0.0; i < 0.1; i += stepSize) {
        nextPose.position.z -= stepSize;
        cartesianTrajectory.push_back(nextPose);
    }

    cout << "planning in x direction" << endl;
    for(double i = 0.0; i < 0.1; i += 0.01) {
        nextPose.position.y -= stepSize;
        cartesianTrajectory.push_back(nextPose);
    }

    // after creating the trajectory, the planner can plan the trajectory and returns a joint trajectory
    // corresponding to the cartesian path
    cout << "planning" << endl;
    // argument 1: cartesian trajectory (with the intermediate poses of the endeffector)
    // argument 2: whether or not the joint trajectory should be smoothed a bit or not (smoothing not perfect yet)
    // argument 3: whether or not considering the current pose of the robot --> will first move from the current
    // state to the first pose in the cartesian trajectory (if you set this to false, you should be sure
    // about what you are doing)
    auto jointPlan = leftSimKomoPlanner.planCartesianTrajectory(cartesianTrajectory, true, true);

    // the complete trajectory can be added to the control queue and will be executed
    // as soon as the previously loaded movements are done (in this case no previous movement is in the
    // queue anymore)
    realLeftQueue->setNextTrajectory(jointPlan);
    realLeftQueue->synchronizeToQueue(1);

    cout << "execution done" << endl;

    /****** done with moving? --> clean up everything and quit *******/

    realLeftQueue->stopCurrentMode();
    realLeftQueue->stopQueue();
    realLqThread->join();

}
