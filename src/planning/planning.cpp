#include <sstream>
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

    std::vector<arma::vec> PathPlanner::smoothJointPlan(std::vector<arma::vec> jointPlan, arma::vec maxVelocities, double cycleTime) {

        return jointPlan;

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

    CachedPlanner::CachedPlanner(StorageSingleton& storage, KUKADU_SHARED_PTR<ControlQueue> queue, KUKADU_SHARED_PTR<PathPlanner> actualPlanner) :
        StorageHolder(storage),
        PathPlanner(actualPlanner->getJointNames()) {

        this->queue = queue;
        this->robotId = queue->getRobotId();
        this->actualPlanner = actualPlanner;

    }

    void CachedPlanner::setSpeed(double speed) {
        actualPlanner->setSpeed(speed);
    }

    void CachedPlanner::addConstraint(KUKADU_SHARED_PTR<Constraint> constraint) {
        actualPlanner->addConstraint(constraint);
    }

    void CachedPlanner::removeConstraint(KUKADU_SHARED_PTR<Constraint> constraint) {
        actualPlanner->removeConstraint(constraint);
    }

    int CachedPlanner::getConstraintsCount() {
        return actualPlanner->getConstraintsCount();
    }

    int CachedPlanner::getConstraintIdx(KUKADU_SHARED_PTR<Constraint> constraint) {
        return actualPlanner->getConstraintIdx(constraint);
    }

    KUKADU_SHARED_PTR<Constraint> CachedPlanner::getConstraintByIdx(int idx) {
        return actualPlanner->getConstraintByIdx(idx);
    }

    bool CachedPlanner::checkAllConstraints(arma::vec currentState, geometry_msgs::Pose pose) {
        return actualPlanner->checkAllConstraints(currentState, pose);
    }

    bool CachedPlanner::isColliding(arma::vec jointState, geometry_msgs::Pose pose) {
        return actualPlanner->isColliding(jointState, pose);
    }

    Eigen::MatrixXd CachedPlanner::getJacobian(std::vector<double> jointState) {
        return actualPlanner->getJacobian(jointState);
    }

    std::vector<arma::vec> CachedPlanner::computeIk(arma::vec currentJointState, const geometry_msgs::Pose& goal) {

        KUKADU_MODULE_START_USAGE();

        double xPosOffset = 0.02;
        double yPosOffset = 0.02;
        double zPosOffset = 0.02;
        double xOrOffset = 0.02;
        double yOrOffset = 0.02;
        double zOrOffset = 0.02;

        vec rpy = quatToRpy(goal.orientation);

        stringstream s;
        s << " robot_id = " << robotId;
        auto frameId = getStorage().getCachedLabelId("reference_frames", "frame_id", "frame_name", getCartesianReferenceFrame(), s.str());
        auto linkId = getStorage().getCachedLabelId("links", "link_id", "link_name", getCartesianLinkName(), s.str());

        s.str("");
        s << "select cm.time_stamp as t, cart_pos_x, cart_pos_y, cart_pos_z, cart_rot_x, cart_rot_y, cart_rot_z, " <<
             // approximate distance to improve performance
             "(abs(cart_pos_x - " << goal.position.x << ") + " <<
             "abs(cart_pos_y - " << goal.position.y << ") + " <<
             "abs(cart_pos_z - " << goal.position.z << ") + " <<
             "abs(cart_rot_x - " << rpy(0) << ") + " <<
             "abs(cart_rot_y - " << rpy(1) << ") + " <<
             "abs(cart_rot_z - " << rpy(2) << ")" <<
             ") as cart_dist" <<
             " from cart_mes_pos as cmp" <<
             " inner join cart_mes as cm on cmp.cart_mes_id = cm.cart_mes_id" <<
             " inner join joint_mes as jm on jm.time_stamp = cm.time_stamp and jm.robot_id = cm.robot_id" <<
             " inner join hardware_joints as hwj on hwj.joint_id = jm.joint_id and hwj.hardware_instance_id = cm.robot_id" <<
             " where reference_frame_id = " << frameId <<
             " and link_id = " << linkId <<
             " and cm.robot_id = " << robotId <<
             " and cart_pos_x >= " << (goal.position.x - xPosOffset) << " and cart_pos_x <= " << (goal.position.x + xPosOffset) <<
             " and cart_pos_y >= " << (goal.position.y - yPosOffset) << " and cart_pos_y <= " << (goal.position.y + yPosOffset) <<
             " and cart_pos_z >= " << (goal.position.z - zPosOffset) << " and cart_pos_z <= " << (goal.position.z + zPosOffset) <<
             " and cart_rot_x >= " << (rpy(0) - xOrOffset) << " and cart_rot_x <= " << (rpy(0) + xOrOffset) <<
             " and cart_rot_y >= " << (rpy(1) - yOrOffset) << " and cart_rot_y <= " << (rpy(1) + yOrOffset) <<
             " and cart_rot_z >= " << (rpy(2) - zOrOffset) << " and cart_rot_z <= " << (rpy(2) + zOrOffset) <<
             " and hwj.joint_name in (";

        bool first = true;
        auto jointNames = getJointNames();
        for(auto& jointName : jointNames) {

            if(!first)
                s << ", ";
            else
                first = false;

            s << "'" << jointName << "'";
        }
        s << ")";
        s << " order by cart_dist limit 0, 5";

        long long int resTimeStamp = -1;
        auto cacheRes = getStorage().executeQuery(s.str());
        if(cacheRes->next()) {

            cout << "(CachedPlanner) used cache ik" << endl;

            // getting the timestamp of a correct configuration
            resTimeStamp = cacheRes->getInt64("t");
            return {loadJointPosByTimestamp(getStorage(), robotId, loadJointIdsFromName(getStorage(), robotId, getJointNames()), resTimeStamp)};

        }

        KUKADU_MODULE_END_USAGE();

        return actualPlanner->computeIk(currentJointState, goal);

    }

    std::vector<arma::vec> CachedPlanner::computeIk(std::vector<double> currentJointState, const geometry_msgs::Pose& goal) {
        return actualPlanner->computeIk(currentJointState, goal);
    }

    geometry_msgs::Pose CachedPlanner::computeFk(std::vector<double> jointState) {
        return actualPlanner->computeFk(jointState);
    }

    void CachedPlanner::setJointNames(std::vector<std::string> jointNames) {
        actualPlanner->setJointNames(jointNames);
    }

    std::vector<std::string> CachedPlanner::getJointNames() {
        return actualPlanner->getJointNames();
    }

    std::string CachedPlanner::getCartesianLinkName() {
        return actualPlanner->getCartesianLinkName();
    }

    std::string CachedPlanner::getCartesianReferenceFrame() {
        return actualPlanner->getCartesianReferenceFrame();
    }

    void CachedPlanner::setCheckCollisions(bool collision) {
        actualPlanner->setCheckCollisions(collision);
    }

    bool CachedPlanner::getCheckCollision() {
        return actualPlanner->getCheckCollision();
    }

    std::vector<arma::vec> CachedPlanner::smoothJointPlan(std::vector<arma::vec> jointPlan, arma::vec maxVelocities, double cycleTime) {
        return actualPlanner->smoothJointPlan(jointPlan, maxVelocities, cycleTime);
    }

    std::vector<arma::vec> CachedPlanner::planJointTrajectory(std::vector<arma::vec> intermediateJoints) {
        return actualPlanner->planJointTrajectory(intermediateJoints);
    }

    std::vector<arma::vec> CachedPlanner::planCartesianTrajectory(std::vector<geometry_msgs::Pose> intermediatePoses,
                                                           bool smoothCartesians, bool useCurrentRobotState) {

        KUKADU_MODULE_START_USAGE();

        auto res = planCartesianTrajectory(computeIk(queue->getCurrentJoints().joints, intermediatePoses.front()).front(), intermediatePoses,
                                       smoothCartesians, useCurrentRobotState);

        KUKADU_MODULE_END_USAGE();

        return res;

    }

    std::vector<arma::vec> CachedPlanner::planCartesianTrajectory(arma::vec startJoints, std::vector<geometry_msgs::Pose> intermediatePoses,
                                                           bool smoothCartesians, bool useCurrentRobotState) {

        KUKADU_MODULE_START_USAGE();

        try {

            if(intermediatePoses.size() > 0) {

                vector<vec> ikSolutions;
                vec startingJoints;
                if(useCurrentRobotState)
                    startingJoints = queue->getCurrentJoints().joints;

                ikSolutions.push_back(startingJoints);
                ikSolutions.push_back(startJoints);

                auto prevIkSolution = computeIk(queue->getCurrentJoints().joints, intermediatePoses.at(0)).front();
                ikSolutions.push_back(prevIkSolution);
                for(int i = 1; i < intermediatePoses.size(); ++i) {
                    auto currentIkSolution = computeIk(prevIkSolution, intermediatePoses.at(i)).front();
                    ikSolutions.push_back(currentIkSolution);
                    prevIkSolution = currentIkSolution;
                }

                cout << "(CachedPlanner) used cached intermediate points" << endl;

                return smoothJointPlan(planJointTrajectory(ikSolutions), {0.0}, queue->getCycleTime());

            }
        } catch(KukaduException& ex) {
            // caching didnt work --> roll back to original planner
        }

        auto res = actualPlanner->planCartesianTrajectory(startJoints, intermediatePoses, smoothCartesians, useCurrentRobotState);

        KUKADU_MODULE_END_USAGE();

        return res;

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
            refInputParams->MaxJerkVector->VecData[i] = 0.001 * cycleTime;
            refInputParams->MaxAccelerationVector->VecData[i] = 0.001 * cycleTime;
            refInputParams->MaxVelocityVector->VecData[i] = (MIN_VEL + (MAX_VEL - MIN_VEL) / 2.0) * cycleTime;
            refInputParams->SelectionVector->VecData[i] = true;
        }

        setSpeed(0.5);

    }

    void SimplePlanner::setSpeed(double speed) {
        currentSpeedFactor = speed;
        for(int i = 0; i < degOfFreedom; ++i)
            refInputParams->MaxVelocityVector->VecData[i] = (MIN_VEL + (MAX_VEL - MIN_VEL) * speed) * cycleTime;
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
