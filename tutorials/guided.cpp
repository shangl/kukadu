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

    double tau, az, bz, dmpStepSize, tolAbsErr, tolRelErr, ac;

    string storeDir = resolvePath("/tmp/kukadu_demo_guided");
    string prefix = "real";
    string arm = "left";

    if(argc == 2)
        storeDir = string(args[1]);

    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
            ("dmp.tau", po::value<double>(), "tau")
            ("dmp.az", po::value<double>(), "az")
            ("dmp.bz", po::value<double>(), "bz")
            ("dmp.dmpStepSize", po::value<double>(), "dmp time step size")
            ("dmp.tolAbsErr", po::value<double>(), "tolerated absolute error")
            ("dmp.tolRelErr", po::value<double>(), "tolerated relative error")
            ("dmp.ac", po::value<double>(), "ac")
    ;

    ifstream parseFile(resolvePath("$KUKADU_HOME/cfg/guided.prop").c_str(), std::ifstream::in);
    po::variables_map vm;
    po::store(po::parse_config_file(parseFile, desc), vm);
    po::notify(vm);

    if (vm.count("dmp.tau")) tau = vm["dmp.tau"].as<double>();
    else return 1;
    if (vm.count("dmp.az")) az = vm["dmp.az"].as<double>();
    else return 1;
    if (vm.count("dmp.bz")) bz = vm["dmp.bz"].as<double>();
    else return 1;
    if (vm.count("dmp.dmpStepSize")) dmpStepSize = vm["dmp.dmpStepSize"].as<double>();
    else return 1;
    if (vm.count("dmp.tolAbsErr")) tolAbsErr = vm["dmp.tolAbsErr"].as<double>();
    else return 1;
    if (vm.count("dmp.tolRelErr")) tolRelErr = vm["dmp.tolRelErr"].as<double>();
    else return 1;
    if (vm.count("dmp.ac")) ac = vm["dmp.ac"].as<double>();
    else return 1;

    cout << "all properties loaded" << endl;

    ros::init(argc, args, "kukadu"); ros::NodeHandle* node = new ros::NodeHandle(); usleep(1e6);
    ros::AsyncSpinner spinner(10);
    spinner.start();

    StorageSingleton& storage = StorageSingleton::get();

    auto leftQueue = make_shared<KukieControlQueue>(storage, "robinn", "real", arm + "_arm", *node);
    auto simLeftQueue = make_shared<KukieControlQueue>(storage, "robinn", "simulation", arm + "_arm", *node);

    leftQueue->install();
    simLeftQueue->install();

    vector<KUKADU_SHARED_PTR<ControlQueue> > queueVectors;
    queueVectors.push_back(leftQueue);

    deleteDirectory(storeDir);

    cout << "press enter to measure trajectory" << endl;
    getchar();

    leftQueue->stopCurrentMode();
    KUKADU_SHARED_PTR<kukadu_thread> laThr = leftQueue->startQueue();
    if(!prefix.compare("simulation")) {
        leftQueue->switchMode(KukieControlQueue::KUKA_JNT_POS_MODE);
    } else {
        leftQueue->setStiffness(0.2, 0.01, 0.2, 15000, 150, 1500);
        leftQueue->switchMode(KukieControlQueue::KUKA_JNT_IMP_MODE);
    }

    cout << "starting measurement" << endl;
    SensorStorage storeObj(storage, queueVectors, std::vector<KUKADU_SHARED_PTR<GenericHand> >(), 1000);
    storeObj.setExportMode(SensorStorage::STORE_RBT_CART_POS | SensorStorage::STORE_RBT_JNT_POS);
    storeObj.startDataStorage(storeDir);
    cout << "measuerment started" << endl;

    if(!prefix.compare("simulation")) {
        cout << "moving arm in simulation" << endl;
        leftQueue->jointPtp(createJointsVector(-0.9692263007164001, 1.113829493522644, 1.1473214626312256, -1.444376826286316, -0.28663957118988037, -0.8957559466362, -0.2651996612548828));
        cout << "movement done" << endl;
    } else {
        ros::Rate r(1);
        for(int i = 0; i < 12; ++i) {
            r.sleep();
            cout << i << endl;
        }
    }

    storeObj.stopDataStorage();

    mes_result finalJoints = leftQueue->getCurrentJoints();
    for(int i = 0; i < 10; ++i)
        leftQueue->move(finalJoints.joints);
    sleep(0.5);

    leftQueue->stopCurrentMode();

    cout << "press enter to execute in simulation" << endl;
    getchar();

    KUKADU_SHARED_PTR<Dmp> dmpFinalPush;
    KUKADU_SHARED_PTR<SensorData> dataFinalPush;
    KUKADU_SHARED_PTR<JointDMPLearner> learnerFinalPush;

    arma::vec timesFinalPush;

    dataFinalPush = SensorStorage::readStorage(simLeftQueue, storeDir + string("/kuka_lwr_") + prefix + string("_left_arm_0"));
    timesFinalPush = dataFinalPush->getNormalizedTimeInSeconds();
    learnerFinalPush = KUKADU_SHARED_PTR<JointDMPLearner>(new JointDMPLearner(az, bz, timesFinalPush, dataFinalPush->getJointPos()));
    dmpFinalPush = learnerFinalPush->fitTrajectories();

    // simLeftQueue->setStiffness(KukieControlQueue::KUKA_STD_XYZ_STIFF, KukieControlQueue::KUKA_STD_ABC_STIFF, KukieControlQueue::KUKA_STD_CPDAMPING, 15000, 150, 1500);
    simLeftQueue->switchMode(KukieControlQueue::KUKA_JNT_POS_MODE);
    laThr = simLeftQueue->startQueue();
    DMPExecutor execFinalPush(storage, dmpFinalPush, simLeftQueue);
    execFinalPush.setExecutionMode(TrajectoryExecutor::EXECUTE_ROBOT);
    execFinalPush.setAc(ac);
    execFinalPush.execute();

    simLeftQueue->stopCurrentMode();
    simLeftQueue->stopQueue();
    laThr->join();

    cout << "press enter to execute on robot" << endl;
    getchar();

    // leftQueue->setStiffness(KukieControlQueue::KUKA_STD_XYZ_STIFF, KukieControlQueue::KUKA_STD_ABC_STIFF, KukieControlQueue::KUKA_STD_CPDAMPING, 15000, 150, 1500);
    if(!prefix.compare("simulation")) {
        leftQueue->switchMode(KukieControlQueue::KUKA_JNT_POS_MODE);
    } else {
        leftQueue->setStiffness(KukieControlQueue::KUKA_STD_XYZ_STIFF, KukieControlQueue::KUKA_STD_ABC_STIFF, KukieControlQueue::KUKA_STD_CPDAMPING, 15000, 150, 1500);
        leftQueue->switchMode(KukieControlQueue::KUKA_JNT_IMP_MODE);
    }

    laThr = leftQueue->startQueue();
    DMPExecutor execFinalPush2(storage, dmpFinalPush, leftQueue);
    execFinalPush2.setExecutionMode(TrajectoryExecutor::EXECUTE_ROBOT);
    execFinalPush2.setAc(ac);

    leftQueue->stopCurrentMode();
    leftQueue->stopQueue();

    storage.waitForEmptyCache();

    return EXIT_SUCCESS;

}
