#ifndef KUKADU_DMPREINFORCER_H
#define KUKADU_DMPREINFORCER_H

#include <vector>
#include <iostream>
#include <armadillo>
#include <kukadu/control/dmp.hpp>
#include <kukadu/types/kukadutypes.hpp>
#include <kukadu/robot/arm/controlqueue.hpp>
#include <kukadu/learning/rl/costcomputer.hpp>

namespace kukadu {

    /** \brief The DMPReinforcer provides a general framework for reinforcement learning.
     *
     * It is an abstract class that implements elementary functionality that should be in common for all reinforcement learning methods (such as rollout execution).
     * A concrete implementation of the DMPReinforcer has to implement the missing parts such as the methods getInitialRollout and computeRolloutParamters.
     * \ingroup ControlPolicyFramework
     */
    class DMPReinforcer {

    private:

        bool isFirstIteration;

        double ac;
        double tolAbsErr;
        double tolRelErr;

        CostComputer* cost;

        KUKADU_SHARED_PTR<Dmp> lastUpdate;
        KUKADU_SHARED_PTR<ControlQueue> movementQueue;
        KUKADU_SHARED_PTR<ControllerResult> lastUpdateRes;

        std::vector<double> lastCost;
        std::vector<KUKADU_SHARED_PTR<Dmp> > rollout;
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
        DMPReinforcer(CostComputer* cost, KUKADU_SHARED_PTR<ControlQueue> movementQueue, double ac, double tolAbsErr, double tolRelErr);

        void setLastUpdate(KUKADU_SHARED_PTR<Dmp> lastUpdate);
        /**
         * \brief executes rollout. first, the trajectory is simulated and the user is asked, whether the trajectory really should be executed
         * \param doSimulation flag whether trajectory should be simulated
         * \param doExecution flag whether trajectory should be executed at robot
         */
        void performRollout(int doSimulation, int doExecution);

        /**
         * \brief returns true if the first iteration has not been performed yet
         */
        bool getIsFirstIteration();

        /**
         * \brief returns simulation tolerated absolute error
         */
        double getTolAbsErr();

        /**
         * \brief returns simulation tolerated relative error
         */
        double getTolRelErr();

        KUKADU_SHARED_PTR<Dmp> getLastUpdate();
        KUKADU_SHARED_PTR<ControllerResult> getLastUpdateRes();

        virtual KUKADU_SHARED_PTR<Dmp> updateStep() = 0;

        std::vector<KUKADU_SHARED_PTR<Dmp> > getLastRolloutParameters();
        /**
         * \brief returns the first rollout of the reinforcement learning algorithm
         */
        virtual std::vector<KUKADU_SHARED_PTR<Dmp> > getInitialRollout() = 0;
        /**
         * \brief computes the dmp parameters for the next rollout
         */
        virtual std::vector<KUKADU_SHARED_PTR<Dmp> > computeRolloutParamters() = 0;

        /**
         * \brief returns cost for the last executed rollout
         */
        std::vector<double> getLastRolloutCost();
        std::vector<KUKADU_SHARED_PTR<ControllerResult> > getLastExecutionResults();

    };

}

#endif
