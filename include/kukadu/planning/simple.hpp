#ifndef KUKADU_SIMPLEPLANNING_H
#define KUKADU_SIMPLEPLANNING_H

#include <vector>
#include <armadillo>
#include <Eigen/Core>
#include <ReflexxesAPI.h>
#include <geometry_msgs/Pose.h>
#include <kukadu/robot/queue.hpp>
#include <kukadu/planning/planning.hpp>
#include <kukadu/types/kukadutypes.hpp>

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
        double currentSpeedFactor;

        KUKADU_SHARED_PTR<Kinematics> kin;
        KUKADU_SHARED_PTR<ControlQueue> queue;

        RMLPositionFlags refFlags;
        ReflexxesAPI* refApi;
        RMLPositionInputParameters* refInputParams;
        RMLPositionOutputParameters* refOutputParams;

        bool checkRestrictions(const std::vector<arma::vec>& plan);
        bool checkPlanSmoothness(const std::vector<arma::vec>& plan);

        void initialize(double cycleTime, int degOfFreedom);

        static constexpr int MAX_NUM_ATTEMPTS = 10;
        static constexpr double MAX_JNT_DIST = 0.2;
        static constexpr auto MIN_VEL = 0.001;
        static constexpr auto MAX_VEL = 0.004;

    public:

        SimplePlanner(KUKADU_SHARED_PTR<ControlQueue> queue, KUKADU_SHARED_PTR<Kinematics> kin, std::vector<std::string> jointNames = std::vector<std::string>());
        ~SimplePlanner();

        virtual std::vector<arma::vec> planJointTrajectory(std::vector<arma::vec> intermediateJoints);
        virtual std::vector<arma::vec> planCartesianTrajectory(std::vector<geometry_msgs::Pose> intermediatePoses, bool smoothCartesians = false, bool useCurrentRobotState = true);
        virtual std::vector<arma::vec> planCartesianTrajectory(arma::vec startJoints, std::vector<geometry_msgs::Pose> intermediatePoses, bool smoothCartesians = false, bool useCurrentRobotState = true);

        virtual std::string getCartesianLinkName();
        virtual std::string getCartesianReferenceFrame();

        virtual geometry_msgs::Pose computeFk(std::vector<double> jointState);
        virtual bool isColliding(arma::vec jointState, geometry_msgs::Pose pose);
        virtual Eigen::MatrixXd getJacobian(std::vector<double> jointState = std::vector<double>());
        virtual std::vector<arma::vec> computeIk(std::vector<double> currentJointState, const geometry_msgs::Pose& goal);

        virtual void setSpeed(double speed);

    };
    
}

#endif
