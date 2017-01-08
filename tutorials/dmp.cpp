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
    string prefix = "simulation";
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

    ifstream parseFile(resolvePath("$KUKADU_HOME/cfg/dmp.prop").c_str(), std::ifstream::in);
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

    auto simLeftQueue = make_shared<KukieControlQueue>(storage, "robinn", "simulation", arm + string("_arm"), *node);
    simLeftQueue->install();

    SensorStorage::transferArmDataToDb(storage, simLeftQueue, resolvePath("$HOME/iis_robot_sw/iis_catkin_ws/src/hangl_2016_tro/experiment/movements/push_fin/kuka_lwr_real_left_arm_0"));
    getchar();

    vector<KUKADU_SHARED_PTR<ControlQueue> > queueVectors;
    queueVectors.push_back(simLeftQueue);

    deleteDirectory(storeDir);

    simLeftQueue->stopCurrentMode();
    simLeftQueue->startQueue();
    simLeftQueue->switchMode(KukieControlQueue::KUKA_JNT_POS_MODE);

    /*
    // this is how to use the SensorStorageSingleton
    cout << "creating singleton" << endl;
    auto& storageSingleton = SensorStorageSingleton::get();

    cout << "registering queue" << endl;
    storageSingleton.registerHardware(simLeftQueue);

    cout << "initiating the storage" << endl;
    storageSingleton.initiateStorageAllRegistered();

    cout << "storing data" << endl;
    ros::Rate r(0.5);
    r.sleep();

    cout << "stopping the storage" << endl;
    storageSingleton.stopStorageAll();

    cout << "press key to continue" << endl;
    getchar();
    */

    string skillName = "";
    cout << "insert the desired name of the skill you want to create or load...";
    cin >> skillName;
    auto availableSkills = SkillFactory::get().listAvailableSkills();

    if(std::find(availableSkills.begin(), availableSkills.end(), skillName) == availableSkills.end()) {

        cout << "skill with this name does not exist yet --> creating it" << endl;

        simLeftQueue->jointPtp({-0.7, 0.7, 1.5, -1.74, -1.85, 1.27, 0.71});

        cout << "press enter to measure trajectory" << endl;
        getchar();

        cout << "starting measurement" << endl;
        SensorStorage sensorStorage(storage, queueVectors, std::vector<KUKADU_SHARED_PTR<GenericHand> >(), 1000);
        sensorStorage.setExportMode(SensorStorage::STORE_RBT_CART_POS | SensorStorage::STORE_RBT_JNT_POS);
        sensorStorage.startDataStorage(storeDir);
        cout << "measurement started" << endl;

        ros::Rate r(2); r.sleep();
        simLeftQueue->jointPtp({-1.5, 1.56, 2.33, -1.74, -1.85, 1.27, 0.71});

        sensorStorage.stopDataStorage();

        cout << "press enter to execute in simulation" << endl;
        getchar();

        KUKADU_SHARED_PTR<Dmp> sampleDmp;
        KUKADU_SHARED_PTR<SensorData> sampleData;
        KUKADU_SHARED_PTR<JointDMPLearner> sampleDmpLearner;

        arma::vec sampleTimes;
        sampleData = SensorStorage::readStorage(simLeftQueue, storeDir + string("/kuka_lwr_") + prefix + string("_left_arm_0"));
        sampleTimes = sampleData->getNormalizedTimeInSeconds();
        sampleDmpLearner = make_shared<JointDMPLearner>(az, bz, sampleTimes, sampleData->getJointPos());
        sampleDmp = sampleDmpLearner->fitTrajectories();

        DMPExecutor sampleExec(storage, sampleDmp, simLeftQueue);
        sampleExec.setExecutionMode(TrajectoryExecutor::EXECUTE_ROBOT);
        sampleExec.setAc(ac);

        sampleExec.createSkillFromThis(skillName);

        auto executionResult = sampleExec.execute();
        vec firstColSamples = sampleData->getJointPos().col(0);

        auto timesExecuted = executionResult->getTimes();
        auto firstColExecuted = executionResult->getYCol(0);

        Gnuplot plot;
        plot.plot_xy(armadilloToStdVec(sampleTimes), armadilloToStdVec(firstColSamples));
        plot.plot_xy(armadilloToStdVec(timesExecuted), armadilloToStdVec(firstColExecuted));

    } else {

        cout << "skill already exists --> loading it from database" << endl;

        auto firstDmp = KUKADU_DYNAMIC_POINTER_CAST<DMPExecutor>(SkillFactory::get().loadSkill(skillName, simLeftQueue));
        firstDmp->setExecutionMode(TrajectoryExecutor::EXECUTE_ROBOT);
        firstDmp->setAc(ac);
        firstDmp->execute();

    }

    getchar();

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
