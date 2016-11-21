#ifndef KUKADU_PLANNING_H
#define KUKADU_PLANNING_H

#include <vector>
#include <armadillo>
#include <Eigen/Core>
#include <ReflexxesAPI.h>
#include <geometry_msgs/Pose.h>
#include <kukadu/types/kukadutypes.hpp>
#include <kukadu/robot/arm/controlqueue.hpp>

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

        virtual bool isColliding(arma::vec jointState, geometry_msgs::Pose pose) = 0;
        virtual Eigen::MatrixXd getJacobian(std::vector<double> jointState = std::vector<double>()) = 0;

        virtual std::vector<arma::vec> computeIk(arma::vec currentJointState, const geometry_msgs::Pose& goal);
        virtual std::vector<arma::vec> computeIk(std::vector<double> currentJointState, const geometry_msgs::Pose& goal) = 0;

        virtual geometry_msgs::Pose computeFk(std::vector<double> jointState) = 0;

        virtual void setJointNames(std::vector<std::string> jointNames);
        virtual std::vector<std::string> getJointNames();

    };

    /**
     * \class PathPlanner
     *
     * \brief
     * \ingroup Kinematics
     */
    class PathPlanner {

    private:

        bool checkCollision;

    public:

        PathPlanner();

        void setCheckCollisions(bool collision);

        bool getCheckCollision();

        virtual std::vector<arma::vec> smoothJointPlan(std::vector<arma::vec> jointPlan);

        virtual std::vector<arma::vec> planJointTrajectory(std::vector<arma::vec> intermediateJoints) = 0;
        virtual std::vector<arma::vec> planCartesianTrajectory(std::vector<geometry_msgs::Pose> intermediatePoses, bool smoothCartesians, bool useCurrentRobotState) = 0;
        virtual std::vector<arma::vec> planCartesianTrajectory(arma::vec startJoints, std::vector<geometry_msgs::Pose> intermediatePoses, bool smoothCartesians = false, bool useCurrentRobotState = true) = 0;

#ifdef CPP11SUPPORTED
        static constexpr int RESULT_FAILED = 0;
        static constexpr int RESULT_SUCCESS = 1;
        static constexpr int RESULT_APPROXIMATE = 2;
#else
        static const int RESULT_FAILED = 0;
        static const int RESULT_SUCCESS = 1;
        static const int RESULT_APPROXIMATE = 2;
#endif

    };

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
