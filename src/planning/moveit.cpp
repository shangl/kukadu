#include <kukadu/utils/utils.hpp>
#include <kukadu/planning/moveit.hpp>
#include <moveit_msgs/GetMotionPlan.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#ifdef ROSKINETIC
#include <moveit/move_group_interface/move_group_interface.h>
#else
#include <moveit/move_group_interface/move_group.h>
#endif
#include <kukadu/storage/moduleusagesingleton.hpp>

using namespace ros;
using namespace std;
using namespace arma;

namespace kukadu {

    /****************** public functions *******************************/

    MoveIt::MoveIt(KUKADU_SHARED_PTR<ControlQueue> queue, NodeHandle node, const std::string& moveGroupName, std::vector<std::string> jointNames, std::string tipLink)
        : PathPlanner(jointNames), moveGroup(string(moveGroupName)) {

        construct(queue, node, moveGroupName, jointNames, tipLink, STD_AVOID_COLLISIONS, STD_MAX_ATTEMPTS, 1.0 / STD_TIMEOUT);

    }

    MoveIt::MoveIt(KUKADU_SHARED_PTR<ControlQueue> queue, ros::NodeHandle node, const std::string& moveGroupName, std::vector<std::string> jointNames, std::string tipLink, bool avoidCollisions, int maxAttempts, double timeOut)
        : PathPlanner(jointNames), moveGroup(string(moveGroupName)) {

        construct(queue, node, moveGroupName, jointNames, tipLink, avoidCollisions, maxAttempts, timeOut);

    }

    geometry_msgs::Pose MoveIt::computeFk(std::vector<double> jointState) {

        KUKADU_MODULE_START_USAGE();

        geometry_msgs::Pose retPose;

        moveit::core::RobotState state(robot_model_);
        state.setJointGroupPositions(jnt_model_group, jointState);

        const Eigen::Affine3d &eef_state = state.getGlobalLinkTransform(tipLink);
        tf::poseEigenToMsg(eef_state, retPose);

        return retPose;

    }

    std::vector<arma::vec> MoveIt::computeIk(std::vector<double> currentJointState, const geometry_msgs::Pose &goal) {

        KUKADU_MODULE_START_USAGE();

        vector<vec> retVec;
        vector<double> solution;

        // create robot state instance and set to initial values
        moveit::core::RobotState state(robot_model_);
        state.setToDefaultValues();

        // check, if seed state was provided and is valid
        if(!currentJointState.empty() && currentJointState.size() != jnt_model_group->getJointModelNames().size()) {
            ROS_ERROR("If a seed state is provided, it has to contain exactly one value per joint!");
            return retVec;
        }

        // set seed state if necessary
        if(!currentJointState.empty()) {
            state.setJointGroupPositions(jnt_model_group, currentJointState);
        }

        // compute result
        if(state.setFromIK(jnt_model_group, goal, tipLink, maxAttempts, timeOut, boost::bind(&MoveIt::collisionCheckCallback, this, _1, _2, _3))) {
            state.copyJointGroupPositions(jnt_model_group, solution);
            retVec.push_back(stdToArmadilloVec(solution));
        }

        KUKADU_MODULE_END_USAGE();

        return retVec;

    }

    std::vector<arma::vec> MoveIt::planJointTrajectory(std::vector<arma::vec> intermediateJoints) {

        KUKADU_MODULE_START_USAGE();

        sensor_msgs::JointState start_state;
        start_state.name = jointNames;
        start_state.position.insert(start_state.position.begin(), intermediateJoints.front().begin(), intermediateJoints.front().end());
        if (!planning_client_.exists()) {
            ROS_ERROR_STREAM("Unable to connect to planning service - ensure that MoveIt is launched!");
            throw(KukaduException("Unable to connect to planning service - ensure that MoveIt is launched!"));
        }

        moveit_msgs::GetMotionPlanRequest get_mp_request;
        moveit_msgs::MotionPlanRequest &request = get_mp_request.motion_plan_request;

        request.group_name = moveGroupName;
        request.num_planning_attempts = planning_attempts_;
        request.allowed_planning_time = planning_time_;
        request.planner_id = planner_id_;
        request.start_state.joint_state = start_state;

        vector<string> joint_names = jointNames;

        moveit_msgs::Constraints c;
        c.joint_constraints.resize(joint_names.size());

        auto jointPos = intermediateJoints.back();
        for (int j = 0; j < joint_names.size(); ++j) {
            moveit_msgs::JointConstraint& jc = c.joint_constraints.at(j);
            jc.joint_name = joint_names.at(j);
            jc.position = jointPos.at(j);
            jc.tolerance_above = 1e-3;
            jc.tolerance_below = 1e-3;
            jc.weight = 1.0;
        }
        request.goal_constraints.push_back(c);

        moveit_msgs::GetMotionPlanResponse get_mp_response;

        bool success = planning_client_.call(get_mp_request, get_mp_response);
        auto solution = get_mp_response.motion_plan_response;

        vector<vec> jointPath;
        if(success) {

            int pts_count = (int) solution.trajectory.joint_trajectory.points.size();
            int error_code = solution.error_code.val;

            for(auto jp : solution.trajectory.joint_trajectory.points)
                jointPath.push_back(stdToArmadilloVec(jp.positions));

            if(error_code != moveit_msgs::MoveItErrorCodes::SUCCESS) {
                stringstream s;
                s << "Planning failed with status code '" << solution.error_code.val << "'";
                ROS_DEBUG_STREAM(s.str());
                throw(KukaduException(s.str().c_str()));
            }

            if(pts_count > max_traj_pts_) {
                ROS_WARN("Valid solution found but contains to many points");
                throw("Valid solution found but contains to many points");
            }

        } else {
            stringstream s;
            s << "Planning failed with status code '" << solution.error_code.val << "'";
            ROS_DEBUG_STREAM(s.str());
            throw(KukaduException(s.str().c_str()));
        }

        KUKADU_MODULE_END_USAGE();

        return simplePlanner->planJointTrajectory(jointPath);

    }

    void MoveIt::setSpeed(double speed) {
        simplePlanner->setSpeed(speed);
    }

    std::vector<arma::vec> MoveIt::planCartesianTrajectory(std::vector<geometry_msgs::Pose> intermediatePoses, bool smoothCartesians, bool useCurrentRobotState) {

        KUKADU_MODULE_START_USAGE();

        vector<double> ikStart;
        for(int i = 0; i < jointNames.size(); ++i)
            ikStart.push_back(0.0);
        auto planRes = planCartesianTrajectory(computeIk(ikStart, intermediatePoses.front()).front(), intermediatePoses, smoothCartesians, useCurrentRobotState);

        KUKADU_MODULE_END_USAGE();

        return planRes;

    }

    std::vector<arma::vec> MoveIt::planCartesianTrajectory(arma::vec startJoints, std::vector<geometry_msgs::Pose> intermediatePoses, bool smoothCartesians, bool useCurrentRobotState) {

        KUKADU_MODULE_START_USAGE();

        sensor_msgs::JointState start_state;
        start_state.name = jointNames;
        auto startJointsVec = armadilloToStdVec(startJoints);
        start_state.position.insert(start_state.position.begin(), startJointsVec.begin(), startJointsVec.end());
        if (!planning_client_.exists()) {
            ROS_ERROR_STREAM("Unable to connect to planning service - ensure that MoveIt is launched!");
            throw(KukaduException("Unable to connect to planning service - ensure that MoveIt is launched!"));
        }

        moveit_msgs::GetMotionPlanRequest get_mp_request;
        moveit_msgs::MotionPlanRequest& request = get_mp_request.motion_plan_request;

        request.group_name = moveGroupName;
        request.num_planning_attempts = planning_attempts_;
        request.allowed_planning_time = planning_time_;
        request.planner_id = planner_id_;
        request.start_state.joint_state = start_state;
        request.goal_constraints.clear();

        ROS_DEBUG("Computing possible IK solutions for goal pose");

        vector<string> joint_names = jointNames;

        // compute a set of ik solutions and construct goal constraint
        for (int i = 0; i < 5; ++i) {

            auto values = computeIk(startJointsVec, intermediatePoses.back());

            if(values.size() > 0) {
                auto ikSol = values.front();
                moveit_msgs::Constraints c;
                for (int j = 0; j < joint_names.size(); ++j) {
                    moveit_msgs::JointConstraint jc;
                    jc.joint_name = joint_names.at(j);
                    jc.position = ikSol(j);
                    jc.tolerance_above = 1e-3;
                    jc.tolerance_below = 1e-3;
                    jc.weight = 1.0;
                    c.joint_constraints.push_back(jc);
                }
                request.goal_constraints.push_back(c);

            }

        }

        if(request.goal_constraints.size() == 0)
            throw KukaduException("no ik solutions found");

        moveit_msgs::GetMotionPlanResponse get_mp_response;

        bool success = planning_client_.call(get_mp_request, get_mp_response);
        auto solution = get_mp_response.motion_plan_response;

        vector<vec> jointPath;
        if(success) {

            int pts_count = (int) solution.trajectory.joint_trajectory.points.size();
            int error_code = solution.error_code.val;

            for(auto jp : solution.trajectory.joint_trajectory.points)
                jointPath.push_back(stdToArmadilloVec(jp.positions));

            if(error_code != moveit_msgs::MoveItErrorCodes::SUCCESS) {
                stringstream s;
                s << "Planning failed with status code '" << solution.error_code.val << "'";
                ROS_DEBUG_STREAM(s.str());
                throw(KukaduException(s.str().c_str()));
            }

            if(pts_count > max_traj_pts_) {
                ROS_WARN("Valid solution found but contains to many points");
                throw("Valid solution found but contains to many points");
            }

        } else {
            stringstream s;
            s << "Planning failed with status code '" << solution.error_code.val << "'";
            ROS_DEBUG_STREAM(s.str());
            throw(KukaduException(s.str().c_str()));
        }

        auto simplePlan = simplePlanner->planJointTrajectory(jointPath);

        KUKADU_MODULE_END_USAGE();

        return simplePlan;

    }

    Eigen::MatrixXd MoveIt::getJacobian(std::vector<double> jointState) {

        KUKADU_MODULE_START_USAGE();

        if(jointState.size() == 0)
            jointState = armadilloToStdVec(queue->getCurrentJoints().joints);

        Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
        Eigen::MatrixXd jacobian;
        moveit::core::RobotState state(robot_model_);
        state.setJointGroupPositions(jnt_model_group, jointState);
        state.getJacobian(jnt_model_group, state.getLinkModel(jnt_model_group->getLinkModelNames().back()), reference_point_position, jacobian);

        KUKADU_MODULE_END_USAGE();

        return jacobian;

    }

    bool MoveIt::isColliding(arma::vec jointState, geometry_msgs::Pose pose) {
        return !modelRestriction->stateOk(jointState, pose);
    }

    std::string MoveIt::getCartesianLinkName() {
        return tipLink;
    }

    std::string MoveIt::getCartesianReferenceFrame() {
        return moveGroup.getPlanningFrame();
    }

    /****************** private functions ******************************/

    void MoveIt::construct(KUKADU_SHARED_PTR<ControlQueue> queue, ros::NodeHandle node, const std::string& moveGroupName, std::vector<std::string> jointNames, std::string tipLink, bool avoidCollisions, int maxAttempts, double timeOut) {

        this->queue = queue;
        this->moveGroupName = moveGroupName;
        this->tipLink = tipLink;
        this->avoidCollisions = avoidCollisions;
        this->maxAttempts = maxAttempts;
        this->timeOut = timeOut;

        simplePlanner = make_shared<SimplePlanner>(queue, KUKADU_SHARED_PTR<Kinematics>());

        /* Load the robot model */
        rml_.reset(new robot_model_loader::RobotModelLoader("robot_description"));

        robot_model_ = rml_->getModel();
        if(!robot_model_) {
            string error = "(MoveIt) Unable to load robot model - make sure, that the robot_description is uploaded to the server";
            ROS_ERROR("(MoveIt) Unable to load robot model - make sure, that the robot_description is uploaded to the server");
            throw KukaduException(error.c_str());
        }

        // create instance of planning scene
        planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));

        jnt_model_group = robot_model_->getJointModelGroup(moveGroupName);

        if(!jnt_model_group) {
            ROS_ERROR("(MoveIt) unknown group name");
            return;
        }

        degOfFreedom = jnt_model_group->getJointModelNames().size();
        modelRestriction = KUKADU_SHARED_PTR<Constraint>(new MoveItConstraint(robot_model_, planning_scene_, jnt_model_group));

        if(avoidCollisions)
            addConstraint(modelRestriction);

        this->jointNames = jointNames;

        if(jnt_model_group->getJointModelNames().size() != jointNames.size()) {
            stringstream s;
            s << "number of specified joint names don't match the number of joints specified in the move group (movegroup: " << jnt_model_group->getJointModelNames().size() << " vs provided: " << jointNames.size() << ")";
            throw KukaduException(s.str().c_str());
        }

        // set some default values
        planning_time_ = 5.0;
        planning_attempts_ = 5;
        max_traj_pts_ = 50;
        goal_joint_tolerance_ = 1e-4;
        goal_position_tolerance_ = 1e-3; // 0.1 mm
        goal_orientation_tolerance_ = 1e-3; // ~0.1 deg
        planner_id_ = "RRTstarkConfigDefault";

        planning_client_ = node.serviceClient<moveit_msgs::GetMotionPlan>("/plan_kinematic_path");

    }

    bool MoveIt::collisionCheckCallback(moveit::core::RobotState *state, const moveit::core::JointModelGroup* joint_group, const double* solution) {

        if(avoidCollisions) {

            vec solVec(degOfFreedom);
            for(int i = 0; i < degOfFreedom; ++i)
                solVec(i) = solution[i];

            geometry_msgs::Pose pose = computeFk(armadilloToStdVec(solVec));

            return checkAllConstraints(solVec, pose);

        } else
            return true;

    }

    KUKADU_SHARED_PTR<Constraint> MoveIt::getModelConstraint() {
        return modelRestriction;
    }

    MoveItConstraint::MoveItConstraint(robot_model::RobotModelPtr robotModel, planning_scene::PlanningScenePtr planningScene, robot_model::JointModelGroup* modelGroup) {

        this->planningScene = planningScene;
        this->modelGroup = modelGroup;
        this->robotModel = robotModel;

    }

    bool MoveItConstraint::stateOk(arma::vec joint, geometry_msgs::Pose cartPose) {

        moveit::core::RobotState state(robotModel);
        double jointStateArray[joint.n_elem];
        for(int i = 0; i < joint.n_elem; ++i)
            jointStateArray[i] = joint[i];
        state.setJointGroupPositions(modelGroup, jointStateArray);
        state.update();

        if(planningScene->isStateColliding(state, modelGroup->getName())) {
            vector<string> collidingLinks;
            collision_detection::CollisionRequest requ;
            collision_detection::CollisionResult res;
            requ.contacts = true;
            requ.max_contacts = 10000;
            planningScene->getCollidingLinks(collidingLinks, state);
            planningScene->checkCollision(requ, res, state);

            cout << cartPose << endl;

            for(collision_detection::CollisionResult::ContactMap::iterator it = res.contacts.begin(); it != res.contacts.end(); ++it) {
                cout << "collission: " << it->first.first << " " << it->first.second << endl;
            }

            cout << "colliding links: ";
            for(int i = 0; i < collidingLinks.size(); ++i) {
                cout << collidingLinks.at(i) << endl;
            }

        }

        auto isOk = !planningScene->isStateColliding(state, modelGroup->getName());
        if(!isOk)
            planningScene->isStateColliding(state, modelGroup->getName(), true);

        return isOk;

    }

    std::string MoveItConstraint::getConstraintName() {
        return string("MoveItConstraint");
    }

    /****************** end ********************************************/

}
