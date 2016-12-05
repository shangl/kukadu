#include <kukadu/utils/utils.hpp>
#include <kukadu/planning/simple.hpp>
#include <kukadu/planning/planning.hpp>

using namespace std;
using namespace arma;

namespace kukadu {

    bool TableConstraint::stateOk(arma::vec joint, geometry_msgs::Pose cartPose) {

        if(cartPose.position.z < 0) {
            if(cartPose.position.x > 0) {
                return false;
            }
        }

        return true;

    }

    std::string TableConstraint::getConstraintName() {
        return string("TableConstraint");
    }

    Kinematics::Kinematics(std::vector<std::string> jointNames) {
        Constraints.clear();
        this->jointNames = jointNames;
    }

    std::vector<std::string> Kinematics::generateDefaultJointNames(int jointCount)  {

        std::vector<std::string> jointNames;
        for(int i = 0; i < jointCount; ++i) {
            stringstream s;
            s << "joint" << i;
            jointNames.push_back(s.str());
        }

        return jointNames;

    }

    void Kinematics::setJointNames(std::vector<std::string> jointNames) {
        this->jointNames = jointNames;
    }

    std::vector<std::string> Kinematics::getJointNames() {
        return jointNames;
    }

    void Kinematics::addConstraint(KUKADU_SHARED_PTR<Constraint> Constraint) {
        Constraints.push_back(Constraint);
    }

    void Kinematics::removeConstraint(KUKADU_SHARED_PTR<Constraint> Constraint) {
        std::remove(Constraints.begin(), Constraints.end(), Constraint);
    }

    int Kinematics::getConstraintsCount() {
        return Constraints.size();
    }

    int  Kinematics::getConstraintIdx(KUKADU_SHARED_PTR<Constraint> Constraint) {
        return std::find(Constraints.begin(), Constraints.end(), Constraint) - Constraints.begin();
    }

    KUKADU_SHARED_PTR<Constraint> Kinematics::getConstraintByIdx(int idx) {
        return Constraints.at(idx);
    }

    std::vector<arma::vec> Kinematics::computeIk(arma::vec currentJointState, const geometry_msgs::Pose &goal) {
        return computeIk(armadilloToStdVec(currentJointState), goal);
    }

    bool Kinematics::checkAllConstraints(arma::vec currentState, geometry_msgs::Pose pose) {

        for(int i = 0; i < getConstraintsCount(); ++i) {

            KUKADU_SHARED_PTR<Constraint> currRest = getConstraintByIdx(i);
            if(!currRest->stateOk(currentState, pose))
                return false;

        }

        return true;

    }

    PathPlanner::PathPlanner(std::vector<std::string> jointNames) : Kinematics(jointNames) {
    }

    void PathPlanner::setCheckCollisions(bool collision) {
        checkCollision = collision;
    }

    bool PathPlanner::getCheckCollision() {
        return checkCollision;
    }

    std::vector<arma::vec> PathPlanner::smoothJointPlan(std::vector<arma::vec> jointPlan) {

        vector<vec> smoothedPlan;

        if(jointPlan.size()) {
            vec lastUsedJoints = jointPlan.at(0);
            smoothedPlan.push_back(lastUsedJoints);
            for(vec joints : jointPlan) {
                if(computeMaxJointDistance(lastUsedJoints, joints) > 0.001) {
                    lastUsedJoints = joints;
                    smoothedPlan.push_back(lastUsedJoints);
                }
            }
        }

        return smoothedPlan;

    }

    bool SimplePlanner::isColliding(arma::vec jointState, geometry_msgs::Pose pose) {
        throw KukaduException("(SimplePlanner) isColliding is not supported by SimplePlanner");
    }

    Eigen::MatrixXd SimplePlanner::getJacobian(std::vector<double> jointState) {
        throw KukaduException("(SimplePlanner) getJacobian is not supported by SimplePlanner");
    }

    std::vector<arma::vec> SimplePlanner::computeIk(std::vector<double> currentJointState, const geometry_msgs::Pose& goal) {
        throw KukaduException("(SimplePlanner) computeIk is not supported by SimplePlanner");
    }

    geometry_msgs::Pose SimplePlanner::computeFk(std::vector<double> jointState) {
        throw KukaduException("(SimplePlanner) computeFk is not supported by SimplePlanner");
    }

    std::string SimplePlanner::getCartesianLinkName() {
        return queue->getCartesianLinkName();
    }

    std::string SimplePlanner::getCartesianReferenceFrame() {
        return queue->getCartesianReferenceFrame();
    }

    void SimplePlanner::initialize(double cycleTime, int degOfFreedom) {

        refApi = new ReflexxesAPI(queue->getDegreesOfFreedom(), 1.0 / cycleTime);
        refInputParams = new RMLPositionInputParameters(queue->getDegreesOfFreedom());
        refOutputParams = new RMLPositionOutputParameters(queue->getDegreesOfFreedom());

        for(int i = 0; i < degOfFreedom; ++i) {
            // this seems to be not normal velocity but velocity normalized by time step
            refInputParams->MaxJerkVector->VecData[i] = 0.003 * cycleTime;
            refInputParams->MaxAccelerationVector->VecData[i] = 0.004 * cycleTime;
            refInputParams->MaxVelocityVector->VecData[i] = 0.002 * cycleTime;
            refInputParams->SelectionVector->VecData[i] = true;
        }

    }

    SimplePlanner::SimplePlanner(KUKADU_SHARED_PTR<ControlQueue> queue, KUKADU_SHARED_PTR<Kinematics> kin, std::vector<std::string> jointNames)
        : PathPlanner((jointNames.size()) ? jointNames : kin->getJointNames()) {

        this->queue = queue;
        this->kin = kin;

        cycleTime = queue->getCycleTime();
        degOfFreedom = queue->getDegreesOfFreedom();

        refApi = NULL;
        refInputParams = NULL;
        refOutputParams = NULL;
        initialize(cycleTime, degOfFreedom);

    }

    SimplePlanner::~SimplePlanner() {
        cout << "(SimplePlanner) memory leak here" << endl;
    }

    std::vector<arma::vec> SimplePlanner::planJointTrajectory(std::vector<arma::vec> intermediateJoints) {

        vector<vec> returnedTrajectory;

        int intermedJointsSize = intermediateJoints.size();
        int newDegOfFreedom = intermediateJoints.front().n_elem;
        int degOfFreedom = this->degOfFreedom;

        if(newDegOfFreedom != degOfFreedom) {
            initialize(cycleTime, newDegOfFreedom);
            degOfFreedom = newDegOfFreedom;
        }

        if(intermedJointsSize >= 1) {

            returnedTrajectory.push_back(intermediateJoints.at(0));

            bool firstTime = true;
            for(int j = 0; j + 1 < intermedJointsSize; ++j) {

                vec currentJointPos = intermediateJoints.at(j);
                vec nextJointPos = intermediateJoints.at(j + 1);

                for(int i = 0; i < degOfFreedom; ++i) {

                    refInputParams->TargetPositionVector->VecData[i] = nextJointPos(i);
                    refInputParams->CurrentPositionVector->VecData[i] = currentJointPos(i);

                    if(firstTime) {
                        refInputParams->CurrentVelocityVector->VecData[i] = 0.0;
                        refInputParams->CurrentAccelerationVector->VecData[i] = 0.0;
                        refInputParams->TargetVelocityVector->VecData[i] = 0.0;
                    } else {
                        // already set below
                    }

                }

                int result = ReflexxesAPI::RML_ERROR;
                while(result != ReflexxesAPI::RML_FINAL_STATE_REACHED) {

                    result = refApi->RMLPosition(*refInputParams, refOutputParams, refFlags);
                    refInputParams->CurrentPositionVector = refOutputParams->NewPositionVector;
                    refInputParams->CurrentVelocityVector = refOutputParams->NewVelocityVector;
                    refInputParams->CurrentAccelerationVector = refOutputParams->NewAccelerationVector;

                    vec next(degOfFreedom);
                    for(int k = 0; k < degOfFreedom; ++k)
                        next(k) = refOutputParams->NewPositionVector->VecData[k];

                    returnedTrajectory.push_back(next);

                }

            }

        }

        if(newDegOfFreedom != degOfFreedom)
            initialize(cycleTime, degOfFreedom);

        return returnedTrajectory;

    }

    std::vector<arma::vec> SimplePlanner::planCartesianTrajectory(arma::vec startJoints, std::vector<geometry_msgs::Pose> intermediatePoses, bool smoothCartesians, bool useCurrentRobotState) {

        for(int i = 0; i < MAX_NUM_ATTEMPTS; ++i) {

            vector<vec> retJoints;

            if(useCurrentRobotState)
                planJointTrajectory({queue->getCurrentJoints().joints, startJoints});

            vec currJoints = startJoints;
            retJoints.push_back(currJoints);

            int posesCount = intermediatePoses.size();
            for(int i = 0; i < posesCount; ++i) {
                vector<vec> nextIk = kin->computeIk(currJoints, intermediatePoses.at(i));
                if(nextIk.size()) {
                    currJoints = nextIk.at(0);
                    retJoints.push_back(nextIk.at(0));
                } else
                    break;
            }

            if(!smoothCartesians || checkPlanSmoothness(retJoints)) {
                vector<vec> plannedTrajectory = planJointTrajectory(retJoints);
                if(checkRestrictions(plannedTrajectory))
                    return plannedTrajectory;
                else
                    ROS_INFO("(SimplePlanner) restriction violation - replan");
            }

        }

        return vector<vec>();

    }

    std::vector<arma::vec> SimplePlanner::planCartesianTrajectory(std::vector<geometry_msgs::Pose> intermediatePoses, bool smoothCartesians, bool useCurrentRobotState) {

        return planCartesianTrajectory(queue->getCurrentJoints().joints, intermediatePoses, smoothCartesians, useCurrentRobotState);

    }

    bool SimplePlanner::checkRestrictions(const std::vector<arma::vec>& plan) {

        for(int i = 0; i < plan.size(); ++i) {
            vec nextPlanJoints = plan.at(i);
            if(!kin->checkAllConstraints(nextPlanJoints, kin->computeFk(armadilloToStdVec(nextPlanJoints))))
                return false;
        }
        return true;

    }

    bool SimplePlanner::checkPlanSmoothness(const std::vector<arma::vec>& plan) {

        for(int i = 0; i < plan.size() - 1; ++i) {
            vec curr = plan.at(i);
            vec next = plan.at(i + 1);
            vec res = (curr - next).t() * (curr - next);
            double dist = sqrt(res(0));
            if(dist > MAX_JNT_DIST)
                return false;
        }

        return true;

    }

}
