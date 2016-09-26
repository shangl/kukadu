#ifndef CustomKinestheticTeacher_HPP
#define CustomKinestheticTeacher_HPP


#include <mutex>
#include <std_msgs/Float64MultiArray.h>
#include <cmath>
#include <eigen_conversions/eigen_msg.h>
#include <iostream>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <Eigen/Dense>
#include "kukadu/robot/sensors/AutoCompensatingFilter.hpp"
#include "kukadu/robot/arm/kukiecontrolqueue.hpp"
#include "kukadu/kinematics/moveitkinematics.hpp"
#include "kukadu/robot/sensorstorage.hpp"
#include "kukadu/robot/KinestheticTeacher.hpp"

namespace kukadu
{

class CustomKinestheticTeacher : KinestheticTeacher{

    static auto constexpr  FILTER_FREQ = 50.0;



    static auto constexpr FORCES_MOVING_MULTIPLIER = 0.14;
    static auto constexpr TORQUES_MOVING_MULTIPLIER = 0.21;

    static auto constexpr BASE_XY_MOVING_MULTIPLIER = 0.17;//0.6
    static auto constexpr BASE_Z_MOVING_MULTIPLIER = 0.6;//0.2
    static auto constexpr ARM_ALLJOINTS_MOVING_MULTIPLIER = 0.22;

    static auto constexpr MAXIMUM_JOINT_STEP = 0.1; //Movement cap
    static auto constexpr MAX_JOINT_MOVEMENT_ALLOWED_FOR_IK = 0.25; // IK solution acceptence threshold
    bool teacherRunning;
    bool filterRunning;
    bool firstReading;

    std::string recordingPath;
    std::vector<double> sensorVal;


    KUKADU_SHARED_PTR<kukadu::ControlQueue> robotinoQueue;
    std::shared_ptr<kukadu::MoveItKinematics> mvKin;
    std::shared_ptr<kukadu_thread> qThread;
    KUKADU_SHARED_PTR<kukadu_thread>recordingThread;
    std::shared_ptr<std::thread> moveThread;
    std::shared_ptr<std::thread> filterThread;



    std::shared_ptr<kukadu::SensorStorage> store;

    enum ControllerType {JACOBIAN,INVERSE,IK,HYBRID};

    Eigen::VectorXd stdToEigenVec(std::vector<double> myVec);
    std::vector<double> eigenToStdVec(Eigen::VectorXd myVec);

    std::vector<double> scaleForcesTorques(std::vector<double> myVec);
    std::vector<double> scaleJointCommands(std::vector<double> myVec);
    int sign(double x);
    bool isBigJump(std::vector<double> myVec);
    std::vector<double> capVec(std::vector<double> input, double maxCap);
    bool isColliding(std::vector<double> jointStates);
    bool isDifferentialCommandSafe(arma::vec diffCommand,arma::vec currentJointStates);
    arma::vec getNextDifferentialCommand(Eigen::MatrixXd jacobian,arma::vec currentJointState, ControllerType myType);
    void generateNextCommand();
    void teachingThreadHandler();
public:

    CustomKinestheticTeacher(KUKADU_SHARED_PTR<kukadu::ControlQueue> myQueue,KUKADU_SHARED_PTR<kukadu::MoveItKinematics> myKin,KUKADU_SHARED_PTR<kukadu::SensorStorage> myStore);

    void init();
    void startTeaching();
    void stopTeaching();
    void startRecording();
    void stopRecording();
    void quit();


};
}
#endif
