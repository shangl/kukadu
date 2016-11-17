#ifndef KUKADU_PLOTTINGCONTROLQUEUE_H
#define KUKADU_PLOTTINGCONTROLQUEUE_H

/**
 * @file   plottingcontrolqueue.hpp
 * @Author Simon Hangl (simon.hangl@uibk.ac.at)
 * @date   May, 2016
 * @brief  Contains an implementation of the control queue interface. The PlottingControlQueue
 * is not bound to a specific robot. It can be used to simulate a robot without any real
 * hardware available.
 *
 */

#include <queue>
#include <time.h>
#include <math.h>
#include <cstdlib>
#include <iostream>
#include <unistd.h>

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>

#include <iis_robot_dep/KukieError.h>
#include <iis_robot_dep/FriRobotData.h>
#include <iis_robot_dep/FriJointState.h>
#include <iis_robot_dep/FriJointCommand.h>
#include <iis_robot_dep/FriRobotJntData.h>
#include <iis_robot_dep/FriJointImpedance.h>
#include <iis_robot_dep/CartesianImpedance.h>

#include <kukadu/utils/types.hpp>
#include <kukadu/utils/utils.hpp>
#include <kukadu/types/kukadutypes.hpp>
#include <kukadu/robot/arm/controlqueue.hpp>
#include <kukadu/utils/destroyableobject.hpp>

namespace kukadu {

    /**
    * \class PlottingControlQueue
    *
    * \brief Contains an implementation of the control queue interface. The PlottingControlQueue
    * is not bound to a specific robot. It can be used to simulate a robot without any real
    * hardware available.
    * \ingroup Robot
    */
    class PlottingControlQueue : public ControlQueue {

    private:

        int impMode;
        int ptpReached;
        int monComMode;
        int currentMode;

        double currTime;

        arma::vec currJoints;
        arma::vec startJoints;

        geometry_msgs::Pose fakeCurrentPose;

        std::vector<std::string> jointNames;

        void construct(std::vector<std::string> jointNames, double timeStep);

    protected:

        virtual void startQueueHook();
        virtual void submitNextJointMove(arma::vec joints);
        virtual void submitNextCartMove(geometry_msgs::Pose pose);
        virtual void setCurrentControlTypeInternal(int controlType);

        virtual bool stopQueueWhilePtp();

    public:

        PlottingControlQueue(int degOfFreedom, double timeStep);
        PlottingControlQueue(std::vector<std::string> jointNames, double timeStep);

        void safelyDestroy();
        void setInitValues();
        void stopCurrentMode();
        void switchMode(int mode);
        void jointPtpInternal(arma::vec joints);
        void setJntPtpThresh(double thresh);
        void setStartingJoints(arma::vec joints);
        void addJointsPosToQueue(arma::vec joints);
        void cartPtpInternal(geometry_msgs::Pose pos, double maxForce);
        void addCartesianPosToQueue(geometry_msgs::Pose pose);
        void setAdditionalLoad(float loadMass, float loadPos);
        void synchronizeToControlQueue(int maxNumJointsInQueue);
        void setStiffness(float cpstiffnessxyz, float cpstiffnessabc, float cpdamping, float cpmaxdelta, float maxforce, float axismaxdeltatrq);

        virtual int getCurrentMode();

        long long int getCurrentTime();

        std::string getRobotName();
        std::string getRobotFileName();

        arma::vec getStartingJoints();
        arma::vec retrieveJointsFromRobot();

        std::vector<std::string> getJointNames();

        mes_result getCurrentJoints();
        mes_result getCurrentJntFrc();
        mes_result getCurrentCartesianPos();
        mes_result getCurrentCartesianFrcTrq();

        geometry_msgs::Pose getCurrentCartesianPose();

        virtual void rollBack(double time);
        virtual void stopJointRollBackMode();
        virtual void startJointRollBackMode(double possibleTime);

    };

}

#endif
