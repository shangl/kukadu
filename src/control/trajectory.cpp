#include <kukadu/control/trajectory.hpp>
#include <kukadu/utils/conversion_utils.hpp>
#include <kukadu/storage/moduleusagesingleton.hpp>

using namespace std;
using namespace arma;

namespace kukadu {

    /****************** public functions *******************************/

    Trajectory::Trajectory(const Trajectory& copy) {
        throw KukaduException("(Trajectory) copy constructor not supported yet");
    }

    Trajectory::Trajectory() { }

    int Trajectory::operator==(Trajectory const& comp) const {
        throw KukaduException("(Trajectory) == operator not supported yet");
    }

    SingleSampleTrajectory::SingleSampleTrajectory(arma::vec supervisedTs, std::vector<arma::vec> sampleYs) {
        this->supervisedTs = supervisedTs;
        this->sampleYs = sampleYs;
    }

    SingleSampleTrajectory::SingleSampleTrajectory(const SingleSampleTrajectory& copy) {
        this->supervisedTs = copy.supervisedTs;
        this->sampleYs = copy.sampleYs;
    }

    SingleSampleTrajectory::SingleSampleTrajectory() {
    }

    int SingleSampleTrajectory::getDegreesOfFreedom() const {
        return sampleYs.size();
    }

    int SingleSampleTrajectory::getDataPointsNum() {
        return supervisedTs.n_elem;
    }

    double SingleSampleTrajectory::getDataPoint(int freedomIdx, int ptIdx) {
        arma::vec sample =  getSampleYByIndex(freedomIdx);
        return sample(ptIdx);
    }

    double SingleSampleTrajectory::getT(int ptIdx) {
        return supervisedTs(ptIdx);
    }

    void SingleSampleTrajectory::setSupervisedTs(arma::vec supervisedTs) {
        this->supervisedTs = supervisedTs;
    }

    void SingleSampleTrajectory::setSampleYs(std::vector<arma::vec> sampleYs) {
        this->sampleYs = sampleYs;
    }

    arma::vec SingleSampleTrajectory::getSupervisedTs() {
        return supervisedTs;
    }

    arma::vec SingleSampleTrajectory::getSampleYByIndex(int idx) {
        return sampleYs.at(idx);
    }

    int SingleSampleTrajectory::operator==(SingleSampleTrajectory const& comp) const {

        return compareArmadilloVec(supervisedTs, comp.supervisedTs) && compareVectorOfArmadillos(sampleYs, comp.sampleYs);

    }

    arma::vec SingleSampleTrajectory::getStartingPos() {

        vec ret(getDegreesOfFreedom());

        for(int i = 0; i < getDegreesOfFreedom(); ++i) {
            ret(i) = getDataPoint(i, 0);
        }

        return ret;

    }

    std::vector<arma::vec> SingleSampleTrajectory::getSampleYs() {
        return sampleYs;
    }

    TrajectoryGenerator::TrajectoryGenerator() { }

    TrajectoryExecutor::TrajectoryExecutor(StorageSingleton& dbStorage, KUKADU_SHARED_PTR<ControlQueue> usedQueue, KUKADU_SHARED_PTR<Trajectory> trajectory) :
        Controller(dbStorage, "TrajectoryExecutor", {usedQueue},  0.0) {
        this->trajectory = trajectory;
        this->tEnd = trajectory->getTmax();
        this->executionMode = SIMULATE_ROBOT;
    }

    KUKADU_SHARED_PTR<Trajectory> TrajectoryExecutor::getTrajectory() {
        return trajectory;
    }

    void TrajectoryExecutor::setExecutionMode(enum execution_modes mode) {
        this->executionMode = mode;
    }

    void TrajectoryExecutor::setTStart(double tStartInSeconds) {
        this->tStart = tStartInSeconds;
    }

    void TrajectoryExecutor::setTEnd(double tEndInSeconds) {
        this->tEnd = tEndInSeconds;
    }

    double TrajectoryExecutor::getTEnd() {
        return tEnd;
    }

    double TrajectoryExecutor::getTStart() {
        return tStart;
    }

    TrajectoryExecutor::execution_modes TrajectoryExecutor::getExecutionMode() {
        return executionMode;
    }

    KUKADU_SHARED_PTR<ControllerResult> TrajectoryExecutor::executeInternal() {
        KUKADU_MODULE_START_USAGE();

        KUKADU_SHARED_PTR<ControllerResult> retVal;
        if(executionMode == EXECUTE_ROBOT)
            retVal = executeTrajectory();
        else if(executionMode == SIMULATE_ROBOT)
            retVal = simulateTrajectory();
        else
            throw KukaduException("(TrajectoryExecutor) execution mode is not supported");
        KUKADU_MODULE_END_USAGE();
        return retVal;
    }

    PolyTrajectoryGenerator::PolyTrajectoryGenerator(int basisFunctionCount) {
        this->basisFunctionCount = basisFunctionCount;
    }

    double PolyTrajectoryGenerator::evaluateBasisFunction(double x, int fun) {
        return pow(x, fun);
    }

    double PolyTrajectoryGenerator::evaluateByCoefficientsSingle(double x, vec coeff) {
        double ret = 0.0;
        int coeffDegree = this->getBasisFunctionCount();
        for(int i = 0; i < coeffDegree; ++i) ret += coeff(i) * pow(x, i + 1);
        return ret;
    }

    vec PolyTrajectoryGenerator::evaluateByCoefficientsMultiple(vec x, int sampleCount, vec coeff) {
        int coeffDegree = this->getBasisFunctionCount();
        vec evals(sampleCount);
        for(int i = 0; i < sampleCount; ++i) {
            evals(i) = evaluateByCoefficientsSingle(x(i), coeff);
        }
        return evals;
    }

    int PolyTrajectoryGenerator::getBasisFunctionCount() {
        return basisFunctionCount;
    }

    string PolyTrajectoryGenerator::getTrajectoryType() {
        return "polynomial";
    }

    /****************** private functions ******************************/

    /****************** end ********************************************/

}
