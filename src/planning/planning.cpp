#include <kukadu/utils/utils.hpp>
#include <kukadu/planning/simple.hpp>
#include <kukadu/planning/planning.hpp>
#include <kukadu/storage/moduleusagesingleton.hpp>

using namespace std;
using namespace arma;

namespace kukadu {

    /****************** public functions *******************************/

    bool TableConstraint::stateOk(arma::vec joint, geometry_msgs::Pose cartPose) {

        KUKADU_MODULE_START_USAGE();

        if(cartPose.position.z < 0) {
            if(cartPose.position.x > 0) {
                return false;
            }
        }

        KUKADU_MODULE_END_USAGE();

        return true;

    }

    std::string TableConstraint::getConstraintName() {
        KUKADU_MODULE_START_USAGE();
        auto retVal = string("TableConstraint");
        KUKADU_MODULE_END_USAGE();
        return retVal;
    }

    PathPlanner::PathPlanner(std::vector<std::string> jointNames) : Kinematics(jointNames) {
    }

    Kinematics::Kinematics(std::vector<std::string> jointNames) {
        Constraints.clear();
        this->jointNames = jointNames;
    }

    void Kinematics::addConstraint(KUKADU_SHARED_PTR<Constraint> Constraint) {
        KUKADU_MODULE_START_USAGE();
        Constraints.push_back(Constraint);
        KUKADU_MODULE_END_USAGE();
    }

    void Kinematics::removeConstraint(KUKADU_SHARED_PTR<Constraint> Constraint) {
        KUKADU_MODULE_START_USAGE();
        std::remove(Constraints.begin(), Constraints.end(), Constraint);
        KUKADU_MODULE_END_USAGE();
    }

    int Kinematics::getConstraintsCount() {
        KUKADU_MODULE_START_USAGE();
        auto retVal = Constraints.size();
        KUKADU_MODULE_END_USAGE();
        return retVal;
    }

    int  Kinematics::getConstraintIdx(KUKADU_SHARED_PTR<Constraint> Constraint) {
        KUKADU_MODULE_START_USAGE();
        auto retVal = std::find(Constraints.begin(), Constraints.end(), Constraint) - Constraints.begin();
        KUKADU_MODULE_END_USAGE();
        return retVal;
    }

    bool Kinematics::checkAllConstraints(arma::vec currentState, geometry_msgs::Pose pose) {
        KUKADU_MODULE_START_USAGE();
        bool retVal = true;
        for(int i = 0; i < getConstraintsCount(); ++i) {

            KUKADU_SHARED_PTR<Constraint> currRest = getConstraintByIdx(i);
            if(!currRest->stateOk(currentState, pose))
                break;

        }
        KUKADU_MODULE_END_USAGE();
        return retVal;

    }

    std::vector<arma::vec> Kinematics::computeIk(arma::vec currentJointState, const geometry_msgs::Pose &goal) {
        KUKADU_MODULE_START_USAGE();
        auto retVal = computeIk(armadilloToStdVec(currentJointState), goal);
        KUKADU_MODULE_END_USAGE();
    }

    void Kinematics::setJointNames(std::vector<std::string> jointNames) {
        KUKADU_MODULE_START_USAGE();
        this->jointNames = jointNames;
        KUKADU_MODULE_END_USAGE();
    }

    std::vector<std::string> Kinematics::getJointNames() {
        KUKADU_MODULE_START_USAGE();
        KUKADU_MODULE_END_USAGE();
        return jointNames;
    }

    KUKADU_SHARED_PTR<Constraint> Kinematics::getConstraintByIdx(int idx) {
        KUKADU_MODULE_START_USAGE();
        auto retVal = Constraints.at(idx);
        KUKADU_MODULE_END_USAGE();
        return retVal;
    }

    void PathPlanner::setCheckCollisions(bool collision) {
        KUKADU_MODULE_START_USAGE();
        checkCollision = collision;
        KUKADU_MODULE_END_USAGE();
    }

    bool PathPlanner::getCheckCollision() {
        KUKADU_MODULE_START_USAGE();
        KUKADU_MODULE_END_USAGE();
        return checkCollision;
    }

    std::vector<arma::vec> PathPlanner::smoothJointPlan(std::vector<arma::vec> jointPlan) {

        KUKADU_MODULE_START_USAGE();

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

        KUKADU_MODULE_END_USAGE();

        return smoothedPlan;

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

    bool SimplePlanner::isColliding(arma::vec jointState, geometry_msgs::Pose pose) {
        KUKADU_MODULE_START_USAGE();
        throw KukaduException("(SimplePlanner) isColliding is not supported by SimplePlanner");
        KUKADU_MODULE_END_USAGE();
    }

    Eigen::MatrixXd SimplePlanner::getJacobian(std::vector<double> jointState) {
        KUKADU_MODULE_START_USAGE();
        throw KukaduException("(SimplePlanner) getJacobian is not supported by SimplePlanner");
        KUKADU_MODULE_END_USAGE();
    }

    std::vector<arma::vec> SimplePlanner::computeIk(std::vector<double> currentJointState, const geometry_msgs::Pose& goal) {
        KUKADU_MODULE_START_USAGE();
        throw KukaduException("(SimplePlanner) computeIk is not supported by SimplePlanner");
        KUKADU_MODULE_END_USAGE();
    }

    geometry_msgs::Pose SimplePlanner::computeFk(std::vector<double> jointState) {
        KUKADU_MODULE_START_USAGE();
        throw KukaduException("(SimplePlanner) computeFk is not supported by SimplePlanner");
        KUKADU_MODULE_END_USAGE();
    }

    std::string SimplePlanner::getCartesianLinkName() {
        KUKADU_MODULE_START_USAGE();
        auto retVal = queue->getCartesianLinkName();
        KUKADU_MODULE_END_USAGE();
        return retVal;
    }

    std::string SimplePlanner::getCartesianReferenceFrame() {
        KUKADU_MODULE_START_USAGE();
        auto retVal = queue->getCartesianReferenceFrame();
        KUKADU_MODULE_END_USAGE();
        return retVal;
    }

    SimplePlanner::~SimplePlanner() {
        cout << "(SimplePlanner) memory leak here" << endl;
    }

    std::vector<arma::vec> SimplePlanner::planJointTrajectory(std::vector<arma::vec> intermediateJoints) {

        KUKADU_MODULE_START_USAGE();

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

        KUKADU_MODULE_END_USAGE();

        return returnedTrajectory;

    }

    std::vector<arma::vec> SimplePlanner::planCartesianTrajectory(arma::vec startJoints, std::vector<geometry_msgs::Pose> intermediatePoses, bool smoothCartesians, bool useCurrentRobotState) {

        KUKADU_MODULE_START_USAGE();

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

        KUKADU_MODULE_END_USAGE();

        return vector<vec>();

    }

    std::vector<arma::vec> SimplePlanner::planCartesianTrajectory(std::vector<geometry_msgs::Pose> intermediatePoses, bool smoothCartesians, bool useCurrentRobotState) {

        KUKADU_MODULE_START_USAGE();

        auto retVal = planCartesianTrajectory(queue->getCurrentJoints().joints, intermediatePoses, smoothCartesians, useCurrentRobotState);
        return retVal;

        KUKADU_MODULE_END_USAGE();

    }

    /****************** private functions ******************************/

    std::vector<std::string> Kinematics::generateDefaultJointNames(int jointCount)  {

        std::vector<std::string> jointNames;
        for(int i = 0; i < jointCount; ++i) {
            stringstream s;
            s << "joint" << i;
            jointNames.push_back(s.str());
        }

        return jointNames;

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

    /****************** end ********************************************/

}
