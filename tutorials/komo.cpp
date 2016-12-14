#include <kukadu/kukadu.hpp>
#include <geometry_msgs/Pose.h>

using namespace std;
using namespace kukadu;

int main(int argc, char** args) {

    cout << "setting up ros node" << endl;
    ros::init(argc, args, "kukadu_planning"); ros::NodeHandle node; sleep(1);
    ros::AsyncSpinner spinner(10); spinner.start();

    cout << "setting up control queue" << endl;
    auto realLeftQueue = KUKADU_SHARED_PTR<KukieControlQueue>(new KukieControlQueue("simulation", "left_arm", node));

    cout << "starting queue" << endl;
    auto realLqThread = realLeftQueue->startQueue();

    cout << "switching to impedance mode if it is not there yet" << endl;
    if(realLeftQueue->getCurrentMode() != KukieControlQueue::KUKA_JNT_IMP_MODE) {
        realLeftQueue->stopCurrentMode();
        realLeftQueue->switchMode(KukieControlQueue::KUKA_JNT_IMP_MODE);
    }

    /****** stuff is set up now --> here you can move as much as you want *******/

    cout << "joint ptp" << endl;
    realLeftQueue->jointPtp({-1.5, 1.56, 2.33, -1.74, -1.85, 1.27, 0.71});

    cout << "ptp without max force" << endl;
    geometry_msgs::Pose nextPose; nextPose.position.x = 0.3; nextPose.position.y = 1.3; nextPose.position.z = 0.6;
    nextPose.orientation.x = -0.06; nextPose.orientation.y = -0.14; nextPose.orientation.z = -0.29; nextPose.orientation.w = 0.94;
    realLeftQueue->cartesianPtp(nextPose);
    cout << "first ptp done" << endl;

    cout << "ptp with max force" << endl;
    nextPose.position.z = 0.45;
    realLeftQueue->cartesianPtp(nextPose, 10.0);
    cout << "second ptp done" << endl;

    /****** done with moving? --> clean up everything and quit *******/

    realLeftQueue->stopCurrentMode();
    realLeftQueue->stopQueue();
    realLqThread->join();

}
