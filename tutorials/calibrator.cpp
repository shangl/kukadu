#include <string>
#include <kukadu/kukadu.hpp>

using namespace std;
using namespace arma;
using namespace kukadu;

int main(int argc, char** args) {

    ros::init(argc, args, "kukadu_controlqueue_demo"); ros::NodeHandle node; sleep(1);
    ros::AsyncSpinner spinner(10); spinner.start();

    StorageSingleton& storage = StorageSingleton::get();

    string calibFile = "";
    if(argc == 2)
        calibFile = string(args[1]);

    KUKADU_SHARED_PTR<ControlQueue> realLeftQueue;
    KUKADU_SHARED_PTR<kukadu_thread> realLqThread;
    KUKADU_SHARED_PTR<Localizer> arLocal;

    if(calibFile == "") {

        cout << "setting up control queue" << endl;
        realLeftQueue = make_shared<KukieControlQueue>(storage, "robinn", "real", "left_arm", node);
        realLeftQueue->install();

        cout << "starting queue" << endl;
        realLqThread = realLeftQueue->startQueue();

        cout << "switching to impedance mode if it is not there yet" << endl;
        if(realLeftQueue->getCurrentMode() != KukieControlQueue::KUKA_JNT_IMP_MODE) {
            realLeftQueue->stopCurrentMode();
            realLeftQueue->setStiffness(0.2, 0.01, 0.2, 15000, 150, 1500);
            realLeftQueue->switchMode(KukieControlQueue::KUKA_JNT_IMP_MODE);
        }

        arLocal = make_shared<ArLocalizer>(node, "camera/rgb/image_raw", true);

    }

    cout << "setting up calibrator" << endl;
    auto calibrator = make_shared<CameraCalibrator>(realLeftQueue, arLocal, "t15");

    if(calibFile != "")
        calibrator->setReadDataFromFile(resolvePath(calibFile));

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

    tf::TransformBroadcaster br;
    auto transform = affineTransMatrixToTf(calibrator->calibrateAffineTransMatrix());

    cout << "publishing tf from " << calibrator->getOriginalFrame() << " to " << calibrator->getTargetFrame() << endl;

    while(true)
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), calibrator->getOriginalFrame(), calibrator->getTargetFrame()));

    if(calibFile != "") {

        // leaves the mode for robot movement
        realLeftQueue->stopCurrentMode();

        // stops the queue
        realLeftQueue->stopQueue();

        // waits until everything has stopped
        if(realLqThread->joinable())
            realLqThread->join();

    }

    storage.waitForEmptyCache();

    return EXIT_SUCCESS;

}
