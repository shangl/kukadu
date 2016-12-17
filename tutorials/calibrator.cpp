#include <fstream>
#include <kukadu/kukadu.hpp>

using namespace std;
using namespace kukadu;

int main(int argc, char** args) {

    ros::init(argc, args, "kukadu_controlqueue_demo"); ros::NodeHandle node; sleep(1);
    ros::AsyncSpinner spinner(10); spinner.start();

    StorageSingleton& storage = StorageSingleton::get();

    cout << "setting up control queue" << endl;
    auto realLeftQueue = make_shared<KukieControlQueue>(storage, "robinn", "real", "left_arm", node);

    cout << "starting queue" << endl;
    KUKADU_SHARED_PTR<kukadu_thread> realLqThread = realLeftQueue->startQueue();

    cout << "switching to impedance mode if it is not there yet" << endl;
    if(realLeftQueue->getCurrentMode() != KukieControlQueue::KUKA_JNT_IMP_MODE) {
        realLeftQueue->stopCurrentMode();
        realLeftQueue->setStiffness(0.2, 0.01, 0.2, 15000, 150, 1500);
        realLeftQueue->switchMode(KukieControlQueue::KUKA_JNT_IMP_MODE);
    }

    ArLocalizer arLocal(node, "camera/rgb/image_raw", true);

    ofstream outFile;
    outFile.open(resolvePath("$HOME/calibdata.txt").c_str());

    ros::Rate r(0.5);
    while(true) {

        std::map<std::string, geometry_msgs::Pose> poses = arLocal.localizeObjects();
        if(poses.find("t15") != poses.end()) {

            geometry_msgs::Pose arPose = poses["t15"];
            outFile << "(" << arPose.position.x << ", " << arPose.position.y << ", " << arPose.position.z << ") " <<
                    "(" << arPose.orientation.x << ", " << arPose.orientation.y << ", " << arPose.orientation.z << ", " << arPose.orientation.w << ")" << endl;

            geometry_msgs::Pose robotPose = realLeftQueue->getCurrentCartesianPose();
            outFile << "\t(" << robotPose.position.x << ", " << robotPose.position.y << ", " << robotPose.position.z << ") " <<
                    "(" << robotPose.orientation.x << ", " << robotPose.orientation.y << ", " << robotPose.orientation.z << ", " << robotPose.orientation.w << ")" << endl;

        }
        outFile.flush();

        r.sleep();

    }

    return EXIT_SUCCESS;

}
