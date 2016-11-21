#ifndef KUKADU_CONSTRAINT_H
#define KUKADU_CONSTRAINT_H

#include <armadillo>
#include <geometry_msgs/Pose.h>
#include <kukadu/types/kukadutypes.hpp>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene/planning_scene.h>

namespace kukadu {

    /**
     * \class Constraint
     *
     * \brief
     * \ingroup Kinematics
     */
    class Constraint {

    private:

    public:

        virtual std::string getConstraintName() = 0;

        virtual bool stateOk(arma::vec joint, geometry_msgs::Pose cartPose) = 0;

    };

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
     * \class TableConstraint
     *
     * \brief
     * \ingroup Kinematics
     */
    class TableConstraint: public Constraint {

    public:

        virtual std::string getConstraintName();
        virtual bool stateOk(arma::vec joint, geometry_msgs::Pose cartPose);

    };

}

#endif
