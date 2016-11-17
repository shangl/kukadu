#include <kukadu/robot/arm/plottingcontrolqueue.hpp>

using namespace std;
using namespace arma;

namespace kukadu {

    PlottingControlQueue::PlottingControlQueue(int degOfFreedom, double timeStep) : ControlQueue(degOfFreedom, timeStep) {

        vector<string> jntNames;
        for(int i = 0; i < degOfFreedom; ++i) {
            stringstream s;
            s << i;
            jntNames.push_back(string("joint_") + s.str());
        }
        construct(jntNames, timeStep);

    }

    PlottingControlQueue::PlottingControlQueue(std::vector<std::string> jointNames, double timeStep) : ControlQueue(jointNames.size(), timeStep) {

        construct(jointNames, timeStep);

    }

    void PlottingControlQueue::construct(std::vector<std::string> jointNames, double timeStep) {

        this->jointNames = jointNames;
        setInitValues();

    }

    void PlottingControlQueue::startQueueHook() {

    }

    void PlottingControlQueue::setInitValues() {

        set_ctrlc_exit_handler();
        currTime = getCurrentTime();
        currJoints = arma::vec(1);

    }

    std::string PlottingControlQueue::getRobotFileName() {
        return string("simulation_plotting_control_queue");
    }

    std::string PlottingControlQueue::getRobotName() {
        return string("Simulation (PlottingControlQueue)");
    }

    std::vector<std::string> PlottingControlQueue::getJointNames() {
        return jointNames;
    }

    void PlottingControlQueue::rollBack(double time) {
    //    throw KukaduException("(PlottingControlQueue) roll back mode not supported");
    }

    void PlottingControlQueue::stopJointRollBackMode() {
    //    throw KukaduException("(PlottingControlQueue) roll back mode not supported");
    }

    void PlottingControlQueue::startJointRollBackMode(double possibleTime) {
    //    throw KukaduException("(PlottingControlQueue) roll back mode not supported");
    }

    void PlottingControlQueue::setJntPtpThresh(double thresh) {

    }

    long long int PlottingControlQueue::getCurrentTime() {
        return currTime;
    }

    mes_result PlottingControlQueue::getCurrentCartesianFrcTrq() {

        mes_result ret;
        vec frcTrq(6);
        frcTrq.fill(0.0);

        ret.joints = frcTrq;
        ret.time = getCurrentTime();

        return ret;

    }

    mes_result PlottingControlQueue::getCurrentJntFrc() {

        mes_result ret;
        vec frcTrq(getMovementDegreesOfFreedom());
        frcTrq.fill(0.0);

        ret.joints = frcTrq;
        ret.time = getCurrentTime();

        return ret;

    }

    arma::vec PlottingControlQueue::getStartingJoints() {
        return startJoints;
    }

    void PlottingControlQueue::addJointsPosToQueue(arma::vec joints) {
        currJoints = joints;
        currTime += (long) (getCycleTime() * 1e3);
    }

    void PlottingControlQueue::addCartesianPosToQueue(geometry_msgs::Pose pose) {
        fakeCurrentPose = pose;
    }

    void PlottingControlQueue::switchMode(int mode) {

    }

    void PlottingControlQueue::submitNextJointMove(arma::vec joints) {

    }

    void PlottingControlQueue::submitNextCartMove(geometry_msgs::Pose pose) {

    }

    bool PlottingControlQueue::stopQueueWhilePtp() {
        return false;
    }

    void PlottingControlQueue::setCurrentControlTypeInternal(int controlType) {

    }

    void PlottingControlQueue::stopCurrentMode() {
    }

    void PlottingControlQueue::synchronizeToControlQueue(int maxNumJointsInQueue) {
    }

    void PlottingControlQueue::setStartingJoints(arma::vec joints) {
        currJoints = joints;
        startJoints = joints;
    }

    void PlottingControlQueue::cartPtpInternal(geometry_msgs::Pose pos, double maxForce) {
        fakeCurrentPose = pos;
    }

    void PlottingControlQueue::jointPtpInternal(arma::vec joints) {
        currJoints = joints;
    }

    int PlottingControlQueue::getCurrentMode() {
        return CONTROLQUEUE_JNT_POS_MODE;
    }

    void PlottingControlQueue::setAdditionalLoad(float loadMass, float loadPos) {
    }

    void PlottingControlQueue::setStiffness(float cpstiffnessxyz, float cpstiffnessabc, float cpdamping, float cpmaxdelta, float maxforce, float axismaxdeltatrq) {
    }

    mes_result PlottingControlQueue::getCurrentCartesianPos() {
        mes_result res;
        res.time = 0.0;
        res.joints = stdToArmadilloVec(createJointsVector(7, fakeCurrentPose.position.x, fakeCurrentPose.position.y, fakeCurrentPose.position.z,
                                                          fakeCurrentPose.orientation.x, fakeCurrentPose.orientation.y, fakeCurrentPose.orientation.z, fakeCurrentPose.orientation.w));
        return res;
    }

    geometry_msgs::Pose PlottingControlQueue::getCurrentCartesianPose() {
        return fakeCurrentPose;
    }

    mes_result PlottingControlQueue::getCurrentJoints() {
        mes_result res;
        res.time = getCurrentTime();
        res.joints = currJoints;
        return res;
    }

    void PlottingControlQueue::safelyDestroy() {
    }

}
