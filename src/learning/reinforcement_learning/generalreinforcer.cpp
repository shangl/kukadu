#include <kukadu/learning/rl/generalreinforcer.hpp>
#include <kukadu/learning/rl/dmpreinforcer.hpp>
#include <kukadu/utils/gnuplot.hpp>

using namespace std;
using namespace arma;

namespace kukadu {

    GeneralReinforcer::GeneralReinforcer(KUKADU_SHARED_PTR<TrajectoryExecutor> trajEx, KUKADU_SHARED_PTR<CostComputer> cost, KUKADU_SHARED_PTR<ControlQueue> simulationQueue, KUKADU_SHARED_PTR<ControlQueue> executionQueue) {

        this->trajEx = trajEx;
        this->cost = cost;
        this->simulationQueue = simulationQueue;
        this->executionQueue = executionQueue;
        this->isFirstIteration = true;
        this->lastCost.push_back(-1.0);
        this->lastUpdateCost = 0.0;

    }

    bool GeneralReinforcer::getIsFirstIteration() {
        return isFirstIteration;
    }

    std::vector<double> GeneralReinforcer::getLastRolloutCost() {
        return lastCost;
    }

    std::vector<KUKADU_SHARED_PTR<Trajectory> > GeneralReinforcer::getLastRolloutParameters() {
        return rollout;
    }

    std::vector<KUKADU_SHARED_PTR<ControllerResult> > GeneralReinforcer::getLastExecutionResults() {
        return dmpResult;
    }

    KUKADU_SHARED_PTR<ControllerResult> GeneralReinforcer::getLastUpdateRes() {
        return lastUpdateRes;
    }

    void GeneralReinforcer::performRollout(int doSimulation, int doExecution) {

        char cont = 'n';

        if(isFirstIteration) {

            rollout = getInitialRollout();

        }
        else {

            rollout = computeRolloutParamters();

        }

        lastCost.clear();
        dmpResult.clear();

        int degFreedom = rollout.at(0)->getDegreesOfFreedom();
        arma::vec startingJoints = arma::vec(degFreedom);

        for(int k = 0; k < rollout.size(); ++k) {

            vec startingPos = rollout.at(k)->getStartingPos();
            startingJoints = startingPos;

            KUKADU_SHARED_PTR<ControllerResult> simRes;
            if(doSimulation) {

                simulationQueue->jointPtp(startingJoints);
                trajEx->setTrajectory(rollout.at(k));

                simRes = trajEx->simulateTrajectory();

                if(!doExecution) {

                    if(isFirstIteration)
                        lastUpdateRes = simRes;
                }

            }

            bool useRollout = true;
            if(doExecution) {

                cout << "(GeneralReinforcer) do you want to execute this trajectory? (y/N) ";
                cin >> cont;

                if(cont == 'y' || cont == 'Y') {

                    cout << "(GeneralReinforcer) executing rollout" << endl;

                    simulationQueue->jointPtp(startingJoints);

                    trajEx->setTrajectory(rollout.at(k));
                    simRes = trajEx->executeTrajectory();
                    useRollout = true;

                } else {
                    useRollout = false;
                }

            }

            if(doSimulation || doExecution) {

                dmpResult.push_back(simRes);
                KUKADU_SHARED_PTR<ControllerResult> resK = simRes;
                double delta = cost->computeCost(resK);
                lastCost.push_back(delta);

                if(isFirstIteration) {
                    lastUpdateRes = resK;
                    lastUpdateCost = delta;
                }

            }

            if(doSimulation)
                simulationQueue->jointPtp(startingJoints);
            if(doExecution)
                executionQueue->jointPtp(startingJoints);

            if(doExecution) {
                cout << "(GeneralReinforcer) press a key to perform next rollout (rollout number " << (k + 2) << ")" << endl;
                getchar();
                getchar();
            }

        }

        double tmpCost = lastUpdateCost;
        KUKADU_SHARED_PTR<Trajectory> tmpUpdate = lastUpdate->copy();
        KUKADU_SHARED_PTR<ControllerResult> tmpRes = lastUpdateRes;
        lastUpdate = updateStep();

        trajEx->setTrajectory(lastUpdate);

        if(!isFirstIteration) {

            cout << "(GeneralReinforcer) performing newest update" << endl;

            KUKADU_SHARED_PTR<ControllerResult> simRes;
            if(doSimulation) {
                cout << "(DMPReinforcer) simulating update" << endl;
                simulationQueue->jointPtp(startingJoints);
                simRes = trajEx->simulateTrajectory();

                if(!doExecution) {
                    lastUpdateRes = simRes;
                }
            }

            if(doExecution) {

                cout << "(DMPReinforcer) do you want to execute this update? (y/N) ";
                cin >> cont;

                if(cont == 'y' || cont == 'Y') {

                    cout << "(DMPReinforcer) executing update" << endl;

                    simulationQueue->jointPtp(startingJoints);
                    simRes = trajEx->executeTrajectory();

                } else {
                }

            }

            if(doSimulation || ( doExecution && (cont == 'y' || cont == 'Y') )) {
                lastUpdateRes = simRes;
                lastUpdateCost = cost->computeCost(lastUpdateRes);
            }

        }

        // TODO: this is a hack!!!! repair it (power cannot directly be applied to metric learning) --> results can get worse instead of better
        if(lastUpdateCost < tmpCost) {

            lastUpdateCost = tmpCost;
            lastUpdate = tmpUpdate;
            lastUpdateRes = tmpRes;

            // get best reward
            for(int i = 0; i < lastCost.size(); ++i) {
                if(lastCost.at(i) > tmpCost) {
                    lastUpdateCost = lastCost.at(i);
                    lastUpdate = rollout.at(i);
                    lastUpdateRes = dmpResult.at(i);
                }
            }

        }


        isFirstIteration = false;

        cout << "(DMPReinforcer) last update reward/cost: " << lastUpdateCost << endl;

    }

    double GeneralReinforcer::getLastUpdateReward() {
        return lastUpdateCost;
    }

    KUKADU_SHARED_PTR<Trajectory> GeneralReinforcer::getLastUpdate() {

        return lastUpdate;

    }

    void GeneralReinforcer::setLastUpdate(KUKADU_SHARED_PTR<Trajectory> lastUpdate) {
        this->lastUpdate = lastUpdate;
    }

}
