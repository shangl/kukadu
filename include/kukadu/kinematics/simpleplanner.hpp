#ifndef KUKADU_SIMPLEPLANNER_H
#define KUKADU_SIMPLEPLANNER_H

#include <vector>
#include <armadillo>
#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>
#include <geometry_msgs/Pose.h>
#include <kukadu/types/kukadutypes.hpp>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>
#include <kukadu/kinematics/kinematics.hpp>
#include <kukadu/kinematics/pathplanner.hpp>
#include <kukadu/robot/arm/controlqueue.hpp>

namespace kukadu {

    /**
     * \class SimplePlanner
     *
     * \brief
     * \ingroup Kinematics
     */
    class SimplePlanner : public PathPlanner {

    private:

        int degOfFreedom;

        double cycleTime;

        KUKADU_SHARED_PTR<Kinematics> kin;
        KUKADU_SHARED_PTR<ControlQueue> queue;

        RMLPositionFlags refFlags;
        ReflexxesAPI* refApi;
        RMLPositionInputParameters* refInputParams;
        RMLPositionOutputParameters* refOutputParams;

        bool checkRestrictions(const std::vector<arma::vec>& plan);
        bool checkPlanSmoothness(const std::vector<arma::vec>& plan);
        
        void initialize(double cycleTime, int degOfFreedom);

#ifdef CPP11SUPPORTED
        static constexpr int MAX_NUM_ATTEMPTS = 10;
        static constexpr double MAX_JNT_DIST = 0.2;
#else
        static const int MAX_NUM_ATTEMPTS = 10;
        static const double MAX_JNT_DIST = 0.2;
#endif

    public:

        SimplePlanner(KUKADU_SHARED_PTR<ControlQueue> queue, KUKADU_SHARED_PTR<Kinematics> kin);
        ~SimplePlanner();

        virtual std::vector<arma::vec> planJointTrajectory(std::vector<arma::vec> intermediateJoints);
        virtual std::vector<arma::vec> planCartesianTrajectory(std::vector<geometry_msgs::Pose> intermediatePoses, bool smoothCartesians = false, bool useCurrentRobotState = true);
        virtual std::vector<arma::vec> planCartesianTrajectory(arma::vec startJoints, std::vector<geometry_msgs::Pose> intermediatePoses, bool smoothCartesians = false, bool useCurrentRobotState = true);

    };

}

#endif
