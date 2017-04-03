#ifndef KUKADU_MOVEITKINEMATICS_H
#define KUKADU_MOVEITKINEMATICS_H

#include <string>
#include <armadillo>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <kukadu/planning/simple.hpp>
#include <kukadu/planning/planning.hpp>
#include <kukadu/types/kukadutypes.hpp>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene/planning_scene.h>
#ifdef ROSKINETIC
#include <moveit/move_group_interface/move_group_interface.h>
#else
#include <moveit/move_group_interface/move_group.h>
#endif
#include <moveit/robot_model_loader/robot_model_loader.h>

namespace kukadu {

    /**
     * \class MoveItConstraint
     *
     * \brief
     * \ingroup Kinematics
     */
    class MoveItConstraint: public Constraint {

    private:

        robot_model::RobotModelPtr robotModel;
        planning_scene::PlanningScenePtr planningScene;
        moveit::core::JointModelGroup* modelGroup;

    public:

        MoveItConstraint(robot_model::RobotModelPtr, planning_scene::PlanningScenePtr planningScene, moveit::core::JointModelGroup* modelGroup);

        virtual std::string getConstraintName();
        virtual bool stateOk(arma::vec joint, geometry_msgs::Pose cartPose);

    };

    /**
     * \class MoveItKinematics
     *
     * \brief
     * \ingroup Kinematics
     */
    class MoveIt : public PathPlanner {

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

#ifdef ROSKINETIC
        const moveit::planning_interface::MoveGroupInterface& moveGroup;
#else
        const moveit::planning_interface::MoveGroup& moveGroup;
#endif

        ros::ServiceClient planning_client_;

        KUKADU_SHARED_PTR<Constraint> modelRestriction;
        robot_model::JointModelGroup* jnt_model_group;
        
        KUKADU_SHARED_PTR<SimplePlanner> simplePlanner;
        KUKADU_SHARED_PTR<ControlQueue> queue;

        void construct(KUKADU_SHARED_PTR<ControlQueue> queue, ros::NodeHandle node, const std::string& moveGroupName, std::vector<std::string> jointNames, std::string tipLink, bool avoidCollisions, int maxAttempts, double timeOut);

        bool collisionCheckCallback(moveit::core::RobotState* state, const moveit::core::JointModelGroup* joint_group, const double* solution);

        KUKADU_SHARED_PTR<Constraint> getModelConstraint();

    public:

        MoveIt(KUKADU_SHARED_PTR<ControlQueue> queue, ros::NodeHandle node, const std::string& moveGroupName, std::vector<std::string> jointNames, std::string tipLink);
        MoveIt(KUKADU_SHARED_PTR<ControlQueue> queue, ros::NodeHandle node, const std::string& moveGroupName, std::vector<std::string> jointNames, std::string tipLink, bool avoidCollisions, int maxAttempts, double timeOut);

        virtual geometry_msgs::Pose computeFk(std::vector<double> jointState);
        virtual std::vector<arma::vec> computeIk(std::vector<double> currentJointState, const geometry_msgs::Pose& goal);

        virtual std::vector<arma::vec> planJointTrajectory(std::vector<arma::vec> intermediateJoints);
        virtual std::vector<arma::vec> planCartesianTrajectory(std::vector<geometry_msgs::Pose> intermediatePoses, bool smoothCartesians = false, bool useCurrentRobotState = true);
        virtual std::vector<arma::vec> planCartesianTrajectory(arma::vec startJoints, std::vector<geometry_msgs::Pose> intermediatePoses, bool smoothCartesians = false, bool useCurrentRobotState = true);

        virtual Eigen::MatrixXd getJacobian(std::vector<double> jointState = std::vector<double>());

        virtual bool isColliding(arma::vec jointState, geometry_msgs::Pose pose);

        virtual std::string getCartesianLinkName();
        virtual std::string getCartesianReferenceFrame();

        virtual void setSpeed(double speed);

#ifndef USEBOOST
        static constexpr int STD_MAX_ATTEMPTS = 5;
        static constexpr int STD_TIMEOUT = 10;
        static constexpr bool STD_AVOID_COLLISIONS = true;
#else
        #define STD_MAX_ATTEMPTS 5
        #define STD_TIMEOUT 10
        #define STD_AVOID_COLLISIONS true;
#endif

    };

}

#endif
