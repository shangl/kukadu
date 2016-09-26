#ifndef KUKADU_KINEMATICS_H
#define KUKADU_KINEMATICS_H

#include <vector>
#include <ros/ros.h>
#include <armadillo>
#include <geometry_msgs/Pose.h>
#include <kukadu/kinematics/constraints/constraints.hpp>
#include <kukadu/types/kukadutypes.hpp>

namespace kukadu {

    /**
     * \defgroup Kinematics
     * The kinematics module provides interfaces for motion planning
     * and kinematics (forward and inverse). Further, it contains
     * different implementations to path planners such as MoveIt
     * or KOMO
     */

    /**
     * \class Kinematics
     *
     * \brief
     * \ingroup Kinematics
     */
    class Kinematics {

    private:

        std::vector<KUKADU_SHARED_PTR<Constraint> > Constraints;

        std::vector<std::string> jointNames;

    protected:

        static std::vector<std::string> generateDefaultJointNames(int jointCount);

    public:

        Kinematics(std::vector<std::string> jointNames);

        void addConstraint(KUKADU_SHARED_PTR<Constraint> Constraint);
        void removeConstraint(KUKADU_SHARED_PTR<Constraint> Constraint);

        int getConstraintsCount();
        int  getConstraintIdx(KUKADU_SHARED_PTR<Constraint> Constraint);

        KUKADU_SHARED_PTR<Constraint> getConstraintByIdx(int idx);

        bool checkAllConstraints(arma::vec currentState, geometry_msgs::Pose pose);

        virtual std::vector<arma::vec> computeIk(arma::vec currentJointState, const geometry_msgs::Pose& goal);
        virtual std::vector<arma::vec> computeIk(std::vector<double> currentJointState, const geometry_msgs::Pose& goal) = 0;

        virtual geometry_msgs::Pose computeFk(std::vector<double> jointState) = 0;

        virtual void setJointNames(std::vector<std::string> jointNames);
        virtual std::vector<std::string> getJointNames();

    };

}

#endif
