#ifndef KUKADU_MOVEITKINEMATICS_H
#define KUKADU_MOVEITKINEMATICS_H

#include <vector>
#include <string>
#include <ros/ros.h>
#include <armadillo>
#include <geometry_msgs/Pose.h>
#include <kukadu/kinematics/pathplanner.hpp>
#include <kukadu/kinematics/simpleplanner.hpp>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <kukadu/kinematics/kinematics.hpp>
#include <kukadu/kinematics/constraints/constraints.hpp>
#include <kukadu/types/kukadutypes.hpp>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit/move_group_interface/move_group.h>

namespace kukadu {

    /**
     * \class MoveItKinematics
     *
     * \brief
     * \ingroup Kinematics
     */
    class MoveItKinematics : public PathPlanner, public Kinematics {

    private:

        bool avoidCollisions;

        int maxAttempts;
        int degOfFreedom;

        double timeOut;

        double planning_time_;
        int planning_attempts_;
        int max_traj_pts_;
        double goal_joint_tolerance_;
        double goal_position_tolerance_; // 0.1 mm
        double goal_orientation_tolerance_; // ~0.1 deg
        std::string planner_id_;

        std::vector<std::string> jointNames;

        std::string tipLink;
        std::string moveGroupName;

        ros::NodeHandle node;

        robot_model::RobotModelPtr robot_model_;
        robot_model_loader::RobotModelLoaderPtr rml_;
        planning_scene::PlanningScenePtr planning_scene_;

        ros::ServiceClient planning_client_;

        KUKADU_SHARED_PTR<Constraint> modelRestriction;
        robot_model::JointModelGroup* jnt_model_group;
        
        KUKADU_SHARED_PTR<SimplePlanner> simplePlanner;
        KUKADU_SHARED_PTR<ControlQueue> queue;

        void construct(KUKADU_SHARED_PTR<ControlQueue> queue, ros::NodeHandle node, std::string moveGroupName, std::vector<std::string> jointNames, std::string tipLink, bool avoidCollisions, int maxAttempts, double timeOut);

        bool collisionCheckCallback(moveit::core::RobotState* state, const moveit::core::JointModelGroup* joint_group, const double* solution);

    public:

        MoveItKinematics(KUKADU_SHARED_PTR<ControlQueue> queue, ros::NodeHandle node, std::string moveGroupName, std::vector<std::string> jointNames, std::string tipLink);
        MoveItKinematics(KUKADU_SHARED_PTR<ControlQueue> queue, ros::NodeHandle node, std::string moveGroupName, std::vector<std::string> jointNames, std::string tipLink, bool avoidCollisions, int maxAttempts, double timeOut);

        virtual geometry_msgs::Pose computeFk(std::vector<double> jointState);
        virtual std::vector<arma::vec> computeIk(std::vector<double> currentJointState, const geometry_msgs::Pose& goal);

        virtual std::vector<arma::vec> planJointTrajectory(std::vector<arma::vec> intermediateJoints);
        virtual std::vector<arma::vec> planCartesianTrajectory(std::vector<geometry_msgs::Pose> intermediatePoses, bool smoothCartesians = false, bool useCurrentRobotState = true);
        virtual std::vector<arma::vec> planCartesianTrajectory(arma::vec startJoints, std::vector<geometry_msgs::Pose> intermediatePoses, bool smoothCartesians = false, bool useCurrentRobotState = true);

        virtual Eigen::MatrixXd getJacobian(std::vector<double> jointState = std::vector<double>());

        bool isColliding(arma::vec jointState, geometry_msgs::Pose pose);

        KUKADU_SHARED_PTR<Constraint> getModelConstraint();

#ifdef CPP11SUPPORTED
        static constexpr int STD_MAX_ATTEMPTS = 5;
        static constexpr double STD_TIMEOUT = 0.01;
        static constexpr bool STD_AVOID_COLLISIONS = true;
#else
        static const int STD_MAX_ATTEMPTS = 5;
        static const double STD_TIMEOUT = 0.01;
        static const bool STD_AVOID_COLLISIONS = true;
#endif

    };

}

#endif
