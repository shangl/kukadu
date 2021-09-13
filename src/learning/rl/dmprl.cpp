#include <kukadu/robot/queue.hpp>
#include <kukadu/utils/gnuplot.hpp>
#include <kukadu/types/sensordata.hpp>
#include <kukadu/learning/rl/dmprl.hpp>
#include <kukadu/storage/sensorstorage.hpp>
#include <kukadu/storage/moduleusagesingleton.hpp>

using namespace std;
using namespace arma;

namespace kukadu {

    DMPReinforcer::DMPReinforcer(StorageSingleton& dbStorage, CostComputer* cost, KUKADU_SHARED_PTR<ControlQueue> movementQueue, double ac, double tolAbsErr, double tolRelErr) : storage(dbStorage) {

        this->cost = cost;
        this->movementQueue = movementQueue;
        this->ac = ac;
        this->tolAbsErr = tolAbsErr;
        this->tolRelErr = tolRelErr;
        this->isFirstIteration = true;
        this->lastCost.push_back(-1.0);

    }

    bool DMPReinforcer::getIsFirstIteration() {
        return isFirstIteration;
    }

    std::vector<double> DMPReinforcer::getLastRolloutCost() {
        return lastCost;
    }

    std::vector<KUKADU_SHARED_PTR<Dmp> > DMPReinforcer::getLastRolloutParameters() {
        return rollout;
    }

    std::vector<KUKADU_SHARED_PTR<ControllerResult> > DMPReinforcer::getLastExecutionResults() {
        return dmpResult;
    }

    double DMPReinforcer::getTolAbsErr() {
        return tolAbsErr;
    }

    double DMPReinforcer::getTolRelErr() {
        return tolRelErr;
    }

    KUKADU_SHARED_PTR<ControllerResult> DMPReinforcer::getLastUpdateRes() {
        return lastUpdateRes;
    }

    void DMPReinforcer::performRollout(int doSimulation, int doExecution) {

        KUKADU_MODULE_START_USAGE();

        char cont = 'y';
        vector<Gnuplot*> gs;
        Gnuplot* g1 = NULL;

        if(isFirstIteration) {

            rollout = getInitialRollout();
            isFirstIteration = false;

            DMPExecutor dmpSim(storage, rollout.at(0), movementQueue);

            // TODO: switch this to new class scheme (not explicetely use DMPExecutor, but trajectory executor)
            dmpSim.setExecutionMode(TrajectoryExecutor::SIMULATE_ROBOT);
            lastUpdateRes = dmpSim.execute();

        }
        else {

            rollout = computeRolloutParamters();

        }

        lastCost.clear();
        dmpResult.clear();

        for(int k = 0; k < rollout.size(); ++k) {

            DMPExecutor dmpsim(storage, rollout.at(k), movementQueue);

            if(doSimulation) {

                dmpsim.setExecutionMode(TrajectoryExecutor::SIMULATE_ROBOT);
                KUKADU_SHARED_PTR<ControllerResult> simRes = dmpsim.execute();
                dmpResult.push_back(simRes);

            }

            if(doExecution) {

                cout << "(DMPReinforcer) do you want to execute this trajectory? (y/N) ";
                cin >> cont;

                if(doExecution && (cont == 'y' || cont == 'Y')) {

                    cout << "(DMPReinforcer) executing rollout" << endl;

                    arma::vec startingJoints = rollout.at(k)->getY0();

                    movementQueue->setStartingJoints(startingJoints);
                    movementQueue->setStiffness(2200, 300, 1.0, 15000, 150, 2.0);
                    KUKADU_SHARED_PTR<kukadu_thread> thr = movementQueue->startQueue();

                    dmpsim.setExecutionMode(TrajectoryExecutor::EXECUTE_ROBOT);
                    dmpResult.push_back(dmpsim.execute());

                    movementQueue->stopQueue();

                    if(thr && thr->joinable())
                        thr->join();

                }

            }

            double delta = cost->computeCost(dmpResult.at(k));
            lastCost.push_back(delta);

        }

        lastUpdate = updateStep();
        DMPExecutor dmpsim(storage, lastUpdate, movementQueue);
        dmpsim.setExecutionMode(TrajectoryExecutor::SIMULATE_ROBOT);
        lastUpdateRes = dmpsim.execute();

        double lastUpdateCost = cost->computeCost(lastUpdateRes);

        this->lastUpdate = lastUpdate;

        cout << "(DMPReinforcer) last update reward/cost: " << lastUpdateCost << endl << endl;

        KUKADU_MODULE_END_USAGE();

    }

    KUKADU_SHARED_PTR<Dmp> DMPReinforcer::getLastUpdate() {

        return lastUpdate;

    }

    void DMPReinforcer::setLastUpdate(KUKADU_SHARED_PTR<Dmp> lastUpdate) {
        this->lastUpdate = lastUpdate;
    }

    DmpRewardComputer::DmpRewardComputer(StorageSingleton& storage, string robotName, string file, double az, double bz, double timeStep, int degOfFreedom, double tmax, double step) : TrajectoryBasedReward(degOfFreedom, tmax, step) {

        this->file = file;
        this->az = az;
        this->bz = bz;
        this->timeStep = timeStep;
        KUKADU_SHARED_PTR<ControlQueue> pcq = make_shared<PlottingControlQueue>(storage, robotName, degOfFreedom, "origin", "hand_link", timeStep);

        cout << "(DmpRewardComputer) starting execution of sample trajectory with timeStep size " << timeStep << endl;
        KUKADU_SHARED_PTR<SensorData> data = SensorStorage::readStorage(pcq, file);
        auto timesInMilliseconds = data->getTimeInMilliSeconds();
        auto times = convertAndRemoveOffset(timesInMilliseconds);
        KUKADU_SHARED_PTR<JointDMPLearner> dmpLearner = make_shared<JointDMPLearner>(az, bz, times, data->getJointPos());
        KUKADU_SHARED_PTR<Dmp> finalDmp = dmpLearner->fitTrajectories();
        DMPExecutor execDmp(storage, finalDmp, pcq);
        execDmp.setExecutionMode(TrajectoryExecutor::EXECUTE_ROBOT);
        executionResult = execDmp.execute();

    }

    arma::vec DmpRewardComputer::computeFun(double t) {

        vec time = executionResult->getTimes();
        vec retVec(executionResult->getYs().size());

        if(t >= time(time.n_elem - 1)) {
            for(int i = 0; i < retVec.n_elem; ++i)
                    retVec(i) = executionResult->getYs().at(i)(time.n_elem - 1);

        } else {

            int tIdx = binaryTimeSearch(time, t);
            double firstT = time(tIdx);
            double secondT = time(tIdx + 1);
            double firstDist = timeStep - t + firstT;
            double secondDist = timeStep - secondT + t;
            cout << "(DmpRewardComputer) here is an interpolation bug" << endl;
            for(int i = 0; i < retVec.n_elem; ++i)
                    retVec(i) = (firstDist * executionResult->getYs().at(i)(tIdx) + secondDist * executionResult->getYs().at(i)(tIdx + 1)) / timeStep;

        }

        return retVec;

    }

    int DmpRewardComputer::binaryTimeSearch(arma::vec times, double t) {

        int start = 0;
        int end = times.n_elem - 1;
        int middle = (start + end) / 2;

        if(t >= times(end))
            return end;
        else if(t <= times(start))
            return start;

        while(start != end && (start + 1) != end) {

            if(t >= times(middle) && t < (times(middle) + timeStep))
                return middle;

            if(times(middle) > t)
                end = middle;
            else
                start = middle;

            middle = (start + end) / 2;


        }

        return (start + 1);

    }

}
