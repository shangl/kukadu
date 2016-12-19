#include <fstream>
#include <kukadu/kukadu.hpp>

using namespace std;
using namespace arma;
using namespace kukadu;

int main(int argc, char** args) {

    ros::init(argc, args, "kukadu_controlqueue_demo"); ros::NodeHandle node; sleep(1);
    ros::AsyncSpinner spinner(10); spinner.start();

    StorageSingleton& storage = StorageSingleton::get();

    cout << "setting up control queue" << endl;
    auto realLeftQueue = make_shared<KukieControlQueue>(storage, "robinn", "simulation", "left_arm", node);

    /*
    cout << "starting queue" << endl;
    KUKADU_SHARED_PTR<kukadu_thread> realLqThread = realLeftQueue->startQueue();

    cout << "switching to impedance mode if it is not there yet" << endl;
    if(realLeftQueue->getCurrentMode() != KukieControlQueue::KUKA_JNT_IMP_MODE) {
        realLeftQueue->stopCurrentMode();
        realLeftQueue->setStiffness(0.2, 0.01, 0.2, 15000, 150, 1500);
        realLeftQueue->switchMode(KukieControlQueue::KUKA_JNT_IMP_MODE);
    }
    */

    // auto arLocal = make_shared<ArLocalizer>(node, "camera/rgb/image_raw", true);

    cout << "setting up calibrator" << endl;
    auto calibrator = make_shared<KinectCalibrator>(realLeftQueue, nullptr, "t15");
    calibrator->setReadDataFromFile(resolvePath("$HOME/calibdata.txt"));

    cout << "starting data collection procedure" << endl;
    calibrator->startDataCollection();

    getchar();

    calibrator->endDataCollection();

    auto calibration = calibrator->calibrate();
    cout << "rotation matrix:" << endl << calibration.first << endl;
    cout << "translation vector in robot frame:" << endl << calibration.second << endl;

    vec transInCamera = inv(calibration.first) * calibration.second;
    cout << "translation vector in camera frame:" << endl << transInCamera << endl;

    vec rpy = rotationMatrixToRpy(calibration.first);
    cout << "rpy:" << endl << rpy << endl;

    return EXIT_SUCCESS;

}
