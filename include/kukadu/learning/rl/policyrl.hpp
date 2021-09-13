#ifndef KUKADU_POLICYRL_H
#define KUKADU_POLICYRL_H

#include <vector>
#include <armadillo>
#include <kukadu/robot/queue.hpp>
#include <kukadu/types/kukadutypes.hpp>
#include <kukadu/control/trajectory.hpp>

namespace kukadu {

    /** \brief Interface for reinforcement learning cost function computation used by DMPReinforcer
     *
     * This class provides the necessary interfaces for the cost function computation
     * \ingroup ControlPolicyFramework
     */
    class CostComputer {

    private:

    public:

        /**
         * \brief computes cost for a given dmp execution
         * \param results measured results of the last dmp execution
         */
        virtual double computeCost(KUKADU_SHARED_PTR<ControllerResult> results) = 0;

    };

    class TrajectoryBasedReward : public CostComputer {

    private:

        int degOfFreedom;

        double step;
        double tmax;
        double slope;

        arma::vec rewardsWeights;

    public:

        TrajectoryBasedReward(int degOfFreedom, double tmax, double timeStep);
        TrajectoryBasedReward(int degOfFreedom, arma::vec rewardsWeights, double tmax, double timeStep);

        void writeToFile(std::string file, double tStart, double tEnd, double stepSize);

        double computeCost(KUKADU_SHARED_PTR<ControllerResult> results);

        KUKADU_SHARED_PTR<ControllerResult> getOptimalTraj();
        KUKADU_SHARED_PTR<ControllerResult> getOptimalTraj(double tmax);
        KUKADU_SHARED_PTR<ControllerResult> getOptimalTraj(double tmax, int freedomIdx);
        KUKADU_SHARED_PTR<ControllerResult> getOptimalTraj(double tmin, double tmax, int freedomIdx);

        std::vector<arma::vec> computeFun(arma::vec t);

        virtual arma::vec computeFun(double t) = 0;

    };

    class SampleRewardComputer : public TrajectoryBasedReward {

    private:

        double tmax;
        double slope;

    public:

        SampleRewardComputer(double slope, int degOfFreedom, double tmax, double stepSize);

        arma::vec computeFun(double t);

    };

    /** \brief The TerminalCostComputer implements the CostComputer interface
     *
     * This class implements the CostComputer in a simple way. The cost of the last rollout is inserted manually by the user to the console.
     * This method can be used for very low dimensional reinforcement learning as there a low number of rollouts is needed.
     * \ingroup ControlPolicyFramework
     */
    class TerminalCostComputer : public CostComputer {

    private:

    public:

        double computeCost(KUKADU_SHARED_PTR<ControllerResult> results);

    };

    /** \brief The GeneralReinforcer provides a general framework for reinforcement learning.
     *
     * It is an abstract class that implements elementary functionality that should be in common for all reinforcement learning methods (such as rollout execution).
     * A concrete implementation of the DMPReinforcer has to implement the missing parts such as the methods getInitialRollout and computeRolloutParamters.
     * \ingroup ControlPolicyFramework
     */
    class GeneralReinforcer {

    private:

        bool isFirstIteration;

        double ac;
        double tolAbsErr;
        double tolRelErr;
        double dmpStepSize;
        double lastUpdateCost;

        KUKADU_SHARED_PTR<CostComputer> cost;
        KUKADU_SHARED_PTR<Trajectory> lastUpdate;
        KUKADU_SHARED_PTR<TrajectoryExecutor> trajEx;
        KUKADU_SHARED_PTR<ControlQueue> executionQueue;
        KUKADU_SHARED_PTR<ControlQueue> simulationQueue;
        KUKADU_SHARED_PTR<ControllerResult> lastUpdateRes;

        std::vector<double> lastCost;
        std::vector<KUKADU_SHARED_PTR<Trajectory> > rollout;
        std::vector<KUKADU_SHARED_PTR<ControllerResult> > dmpResult;

    public:

        /**
         * \brief constructor
         * \param cost CostComputer instance that computes the cost of a given rollout
         * \param movementQueue ControlQueue instance for robot execution
         * \param ac dmp phase stopping parameter
         * \param dmpStepSize step size for dmp execution
         * \param tolAbsErr absolute tolerated error for numerical approximation
         * \param tolRelErr relative tolerated error for numerical approximation
         */
        GeneralReinforcer(KUKADU_SHARED_PTR<TrajectoryExecutor> trajEx, KUKADU_SHARED_PTR<CostComputer> cost, KUKADU_SHARED_PTR<ControlQueue> simulationQueue, KUKADU_SHARED_PTR<ControlQueue> executionQueue);

        /**
         * \brief returns true if the first iteration has not been performed yet
         */
        bool getIsFirstIteration();

        /**
         * \brief executes rollout. first, the trajectory is simulated and the user is asked, whether the trajectory really should be executed
         * \param doSimulation flag whether trajectory should be simulated
         * \param doExecution flag whether trajectory should be executed at robot
         */
        void performRollout(int doSimulation, int doExecution);
        void setLastUpdate(KUKADU_SHARED_PTR<Trajectory> lastUpdate);

        double getLastUpdateReward();

        KUKADU_SHARED_PTR<Trajectory> getLastUpdate();
        KUKADU_SHARED_PTR<ControllerResult> getLastUpdateRes();

        /**
         * \brief returns cost for the last executed rollout
         */
        std::vector<double> getLastRolloutCost();
        std::vector<KUKADU_SHARED_PTR<Trajectory> > getLastRolloutParameters();
        std::vector<KUKADU_SHARED_PTR<ControllerResult> > getLastExecutionResults();

        virtual KUKADU_SHARED_PTR<Trajectory> updateStep() = 0;

        /**
         * \brief returns the first rollout of the reinforcement learning algorithm
         */
        virtual std::vector<KUKADU_SHARED_PTR<Trajectory> > getInitialRollout() = 0;

        /**
         * \brief computes the dmp parameters for the next rollout
         */
        virtual std::vector<KUKADU_SHARED_PTR<Trajectory> > computeRolloutParamters() = 0;

    };

    class PoWER : public GeneralReinforcer {

    private:

        int updatesPerRollout;
        int importanceSamplingCount;

        double explorationSigma;

        KUKADU_SHARED_PTR<TrajectoryExecutor> trajEx;

        std::vector<double> sigmas;
        std::vector<kukadu_normal_distribution> normals;
        std::vector<KUKADU_SHARED_PTR<Trajectory> > initDmp;
        std::vector<std::pair <double, KUKADU_SHARED_PTR<Trajectory> > > sampleHistory;

        kukadu_mersenne_twister generator;

        void construct(std::vector<KUKADU_SHARED_PTR<Trajectory> > initDmp, std::vector<double> explorationSigmas, int updatesPerRollout, int importanceSamplingCount, KUKADU_SHARED_PTR<CostComputer> cost, KUKADU_SHARED_PTR<ControlQueue> simulationQueue, KUKADU_SHARED_PTR<ControlQueue> executionQueue, double ac, double dmpStepSize, double tolAbsErr, double tolRelErr);

    public:

        PoWER(KUKADU_SHARED_PTR<TrajectoryExecutor> trajEx, std::vector<KUKADU_SHARED_PTR<Trajectory> > initDmp, double explorationSigma, int updatesPerRollout, int importanceSamplingCount, KUKADU_SHARED_PTR<CostComputer> cost, KUKADU_SHARED_PTR<ControlQueue> simulationQueue, KUKADU_SHARED_PTR<ControlQueue> executionQueue, double ac, double dmpStepSize, double tolAbsErr, double tolRelErr, unsigned seed);
        PoWER(KUKADU_SHARED_PTR<TrajectoryExecutor> trajEx, std::vector<KUKADU_SHARED_PTR<Trajectory> > initDmp, std::vector<double> explorationSigmas, int updatesPerRollout, int importanceSamplingCount, KUKADU_SHARED_PTR<CostComputer> cost, KUKADU_SHARED_PTR<ControlQueue> simulationQueue, KUKADU_SHARED_PTR<ControlQueue> executionQueue, double ac, double dmpStepSize, double tolAbsErr, double tolRelErr, unsigned seed);

        KUKADU_SHARED_PTR<Trajectory> updateStep();

        std::vector<KUKADU_SHARED_PTR<Trajectory> > getInitialRollout();
        std::vector<KUKADU_SHARED_PTR<Trajectory> > computeRolloutParamters();

    };

    class GradientDescent : public GeneralReinforcer {

    private:

        int updateNum;
        int updatesPerRollout;
        int importanceSamplingCount;

        double explorationSigma;

        TrajectoryExecutor* trajEx;

        kukadu_mersenne_twister generator;

        std::vector<double> sigmas;
        std::vector<kukadu_normal_distribution> normals;
        std::vector<KUKADU_SHARED_PTR<Trajectory> > initDmp;
        std::vector<std::pair <double, KUKADU_SHARED_PTR<Trajectory> > > sampleHistory;

        void construct(std::vector<KUKADU_SHARED_PTR<Trajectory> > initDmp, std::vector<double> explorationSigmas, int updatesPerRollout, int importanceSamplingCount, KUKADU_SHARED_PTR<CostComputer> cost, KUKADU_SHARED_PTR<ControlQueue> simulationQueue, KUKADU_SHARED_PTR<ControlQueue> executionQueue, double ac, double dmpStepSize, double tolAbsErr, double tolRelErr);

    public:

        GradientDescent(KUKADU_SHARED_PTR<TrajectoryExecutor> trajEx, std::vector<KUKADU_SHARED_PTR<Trajectory> > initDmp, double explorationSigma, int updatesPerRollout, int importanceSamplingCount, KUKADU_SHARED_PTR<CostComputer> cost, KUKADU_SHARED_PTR<ControlQueue> simulationQueue, KUKADU_SHARED_PTR<ControlQueue> executionQueue, double ac, double dmpStepSize, double tolAbsErr, double tolRelErr);
        GradientDescent(KUKADU_SHARED_PTR<TrajectoryExecutor> trajEx, std::vector<KUKADU_SHARED_PTR<Trajectory> > initDmp, std::vector<double> explorationSigmas, int updatesPerRollout, int importanceSamplingCount, KUKADU_SHARED_PTR<CostComputer> cost, KUKADU_SHARED_PTR<ControlQueue> simulationQueue, KUKADU_SHARED_PTR<ControlQueue> executionQueue, double ac, double dmpStepSize, double tolAbsErr, double tolRelErr);

        KUKADU_SHARED_PTR<Trajectory> updateStep();

        std::vector<KUKADU_SHARED_PTR<Trajectory> > getInitialRollout();
        std::vector<KUKADU_SHARED_PTR<Trajectory> > computeRolloutParamters();

    };

}

#endif
