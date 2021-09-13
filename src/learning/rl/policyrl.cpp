#include <kukadu/learning/rl/policyrl.hpp>
#include <kukadu/storage/moduleusagesingleton.hpp>

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

        KUKADU_MODULE_START_USAGE();

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

                trajEx->setExecutionMode(TrajectoryExecutor::SIMULATE_ROBOT);
                simRes = trajEx->execute();

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
                    trajEx->setExecutionMode(TrajectoryExecutor::EXECUTE_ROBOT);
                    simRes = trajEx->execute();
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
                trajEx->setExecutionMode(TrajectoryExecutor::SIMULATE_ROBOT);
                simRes = trajEx->execute();

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
                    trajEx->setExecutionMode(TrajectoryExecutor::EXECUTE_ROBOT);
                    simRes = trajEx->execute();

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

        KUKADU_MODULE_END_USAGE();

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

    GradientDescent::GradientDescent(KUKADU_SHARED_PTR<TrajectoryExecutor> trajEx, std::vector<KUKADU_SHARED_PTR<Trajectory> > initDmp, double explorationSigma, int updatesPerRollout, int importanceSamplingCount, KUKADU_SHARED_PTR<CostComputer> cost, KUKADU_SHARED_PTR<ControlQueue> simulationQueue, KUKADU_SHARED_PTR<ControlQueue> executionQueue, double ac, double dmpStepSize, double tolAbsErr, double tolRelErr) : GeneralReinforcer(trajEx, cost, simulationQueue, executionQueue) {

        throw KukaduException("(GradientDescent) currently broken");

        vector<double> intSigmas;

        // init sampler
        for(int i = 0; i < initDmp.at(0)->getCoefficients().at(0).n_elem; ++i) {
            vec currCoeff = initDmp.at(0)->getCoefficients().at(0);
            kukadu_normal_distribution normal(0, explorationSigma);
            normals.push_back(normal);
            intSigmas.push_back(abs(explorationSigma));
        }

        construct(initDmp, intSigmas, updatesPerRollout, importanceSamplingCount, cost, simulationQueue, executionQueue, ac, dmpStepSize, tolAbsErr, tolRelErr);

    }

    GradientDescent::GradientDescent(KUKADU_SHARED_PTR<TrajectoryExecutor> trajEx, std::vector<KUKADU_SHARED_PTR<Trajectory> > initDmp, vector<double> explorationSigmas, int updatesPerRollout, int importanceSamplingCount, KUKADU_SHARED_PTR<CostComputer> cost, KUKADU_SHARED_PTR<ControlQueue> simulationQueue, KUKADU_SHARED_PTR<ControlQueue> executionQueue, double ac, double dmpStepSize, double tolAbsErr, double tolRelErr) : GeneralReinforcer(trajEx, cost, simulationQueue, executionQueue) {

        throw KukaduException("(GradientDescent) currently broken");

        // init sampler
        for(int i = 0; i < initDmp.at(0)->getCoefficients().at(0).n_elem; ++i) {

            vec currCoeff = initDmp.at(0)->getCoefficients().at(0);
            kukadu_normal_distribution normal(0, explorationSigmas.at(i));
            normals.push_back(normal);

        }

        construct(initDmp, explorationSigmas, updatesPerRollout, importanceSamplingCount, cost, simulationQueue, executionQueue, ac, dmpStepSize, tolAbsErr, tolRelErr);

    }

    void GradientDescent::construct(std::vector<KUKADU_SHARED_PTR<Trajectory> > initDmp, vector<double> explorationSigmas, int updatesPerRollout, int importanceSamplingCount, KUKADU_SHARED_PTR<CostComputer> cost, KUKADU_SHARED_PTR<ControlQueue> simulationQueue, KUKADU_SHARED_PTR<ControlQueue> executionQueue, double ac, double dmpStepSize, double tolAbsErr, double tolRelErr) {

        setLastUpdate(initDmp.at(0));

        this->initDmp = initDmp;
        this->explorationSigma = explorationSigma;
        this->sigmas = explorationSigmas;
        this->updatesPerRollout = updatesPerRollout;
        this->importanceSamplingCount = importanceSamplingCount;
        this->updateNum = 0;

    }

    std::vector<KUKADU_SHARED_PTR<Trajectory> > GradientDescent::getInitialRollout() {
        vector<KUKADU_SHARED_PTR<Trajectory> > ret;
        ret.push_back(initDmp.at(0));
        return ret;
    }

    std::vector<KUKADU_SHARED_PTR<Trajectory> > GradientDescent::computeRolloutParamters() {

        KUKADU_MODULE_START_USAGE();

        KUKADU_SHARED_PTR<Trajectory> lastUp = getLastUpdate();
        vector<vec> dmpCoeffs = lastUp->getCoefficients();
        vector<KUKADU_SHARED_PTR<Trajectory> > nextCoeffs;

        for(int k = 0; k < updatesPerRollout; ++k) {

            for(int i = 0; i < dmpCoeffs.size(); ++i) {

                vec currCoeff = dmpCoeffs.at(i);

                for(int j = 0; j < currCoeff.n_elem; ++j) {

                    kukadu_normal_distribution normal = normals.at(j);
                    double eps = normal(generator);

                    currCoeff(j) += eps;

                }

                dmpCoeffs[i] = currCoeff;

            }

            KUKADU_SHARED_PTR<Trajectory> nextUp = lastUp->copy();
            nextUp->setCoefficients(dmpCoeffs);

            nextCoeffs.push_back(nextUp);

        }

        KUKADU_MODULE_END_USAGE();

        return nextCoeffs;

    }

    KUKADU_SHARED_PTR<Trajectory> GradientDescent::updateStep() {

        KUKADU_MODULE_START_USAGE();

        ++updateNum;
        KUKADU_SHARED_PTR<Trajectory> lastUp = getLastUpdate();
        KUKADU_SHARED_PTR<Trajectory> newUp = KUKADU_SHARED_PTR<Trajectory>();

        vector<KUKADU_SHARED_PTR<Trajectory> > lastDmps = getLastRolloutParameters();
        vector<double> lastRewards = getLastRolloutCost();

        vec lastEstimate = lastDmps.at(0)->getCoefficients().at(0);

        if(lastDmps.size() > 1) {

            // add rollouts to history
            for(int i = 0; i < lastRewards.size(); ++i) {
                pair <double, KUKADU_SHARED_PTR<Trajectory> > p(lastRewards.at(i), lastDmps.at(i));
                sampleHistory.push_back(p);
            }

            double lastUpdateRew = getLastUpdateReward();
            mat deltaTheta(lastDmps.at(0)->getCoefficients().at(0).n_elem, lastRewards.size());
            vec deltaJ(lastRewards.size());
            for(int i = 0; i < deltaTheta.n_cols; ++i) {
                KUKADU_SHARED_PTR<Trajectory> currDmp = lastDmps.at(i);
                vector<vec> currCoeffs = currDmp->getCoefficients();
                deltaJ(i) = lastRewards.at(i) - lastUpdateRew;
                for(int j = 0; j < deltaTheta.n_rows; ++j) {
                    vec currCoeffsVec = currCoeffs.at(0);
                    deltaTheta(j, i) = currCoeffsVec(j);
                }

            }

            vec gradEstimate = inv(deltaTheta * deltaTheta.t()) * deltaTheta * deltaJ;
            vec newEstimate = lastEstimate + gradEstimate / updateNum;
            cout << "last estimate: " << lastEstimate.t();
            cout << "gradient estimate" << gradEstimate.t() / updateNum << endl << endl;
            vector<vec> newCoeffs;
            newCoeffs.push_back(newEstimate);

            newUp = lastUp->copy();
            newUp->setCoefficients(newCoeffs);

        } else
            newUp = lastDmps.at(0);

        KUKADU_MODULE_END_USAGE();

        return newUp;

    }

    SampleRewardComputer::SampleRewardComputer(double slope, int degOfFreedom, double tmax, double stepSize) : TrajectoryBasedReward(degOfFreedom, tmax, stepSize) {
        this->slope = slope;
    }

    arma::vec SampleRewardComputer::computeFun(double t) {

        arma::vec retVec(1);
        double val = 0.0;

        retVec(0) = val = sin(t * 1.5);
        return retVec;

    }

    double TerminalCostComputer::computeCost(KUKADU_SHARED_PTR<ControllerResult> results) {

        KUKADU_MODULE_START_USAGE();

        double delta = 0.0;
        cout << "(TerminalCostComputer) Enter the deviation in query space (also be aware of the sign; e.g. qmeasured +/- cost = qdesired)...";
        cin >> delta;

        KUKADU_MODULE_END_USAGE();

        return delta;

    }

    TrajectoryBasedReward::TrajectoryBasedReward(int degOfFreedom, double tmax, double step) {

        this->step = step;
        this->tmax = tmax;
        this->degOfFreedom = degOfFreedom;
        rewardsWeights = vec(degOfFreedom);
        rewardsWeights.fill(1.0);

    }

    TrajectoryBasedReward::TrajectoryBasedReward(int degOfFreedom, arma::vec rewardsWeights, double tmax, double step) {

        this->step = step;
        this->degOfFreedom = degOfFreedom;
        this->rewardsWeights = rewardsWeights;
        this->tmax = tmax;

    }

    double TrajectoryBasedReward::computeCost(KUKADU_SHARED_PTR<ControllerResult> results) {

        KUKADU_MODULE_START_USAGE();

        int tCount = results->getTimes().n_elem;
        tmax = results->getTimes()(tCount - 1);

        double reward = 0.0;

        vector<vec> funVals = computeFun(results->getTimes());
        for(int i = 0; i < degOfFreedom; ++i) {
            vec y = results->getYs().at(i);

            vec diffVec = funVals.at(i) - y;
            vec rewardVec = diffVec.t() * diffVec;
            reward += rewardsWeights(i) *  rewardVec(0);

        }

        reward = reward / (tCount * sum(rewardsWeights));
        reward = 1.0 / exp(sqrt(reward));

        KUKADU_MODULE_END_USAGE();

        return reward;

    }

    void TrajectoryBasedReward::writeToFile(std::string file, double tStart, double tEnd, double stepSize) {

        ofstream outFile;
        outFile.open(file.c_str());

        for(; tStart < tEnd; tStart += stepSize) {
            arma::vec currentY = computeFun(tStart);
            outFile << tStart << "\t" << currentY.t() << endl;
        }

        outFile.close();

    }

    KUKADU_SHARED_PTR<ControllerResult> TrajectoryBasedReward::getOptimalTraj() {
        return getOptimalTraj(tmax);
    }

    KUKADU_SHARED_PTR<ControllerResult> TrajectoryBasedReward::getOptimalTraj(double tmax) {

        double tmin = 0;

        int size = (int) ( (double) (tmax - tmin) / (double) step);

        vec retT = vec(size);
        int i = 0;

        for(double t = tmin; i < size; t = t + step, ++i) {
            retT(i) = t - tmin;
        }

        return KUKADU_SHARED_PTR<ControllerResult>(new ControllerResult(retT, computeFun(retT), true));

    }

    KUKADU_SHARED_PTR<ControllerResult> TrajectoryBasedReward::getOptimalTraj(double tmax, int freedomIdx) {
        return getOptimalTraj(0, tmax, freedomIdx);
    }

    KUKADU_SHARED_PTR<ControllerResult> TrajectoryBasedReward::getOptimalTraj(double tmin, double tmax, int freedomIdx) {


        //TODO: revise this
        double step = 0.1;

        int size = (int) ( (double) (tmax - tmin) / (double) step);

        vec ys = vec(size);
        vec retT = vec(size);
        int i = 0;

        for(double t = tmin; i < size; t = t + step, ++i) {
            retT(i) = t - tmin;
            ys(i) = computeFun(t).at(freedomIdx);
        }

        vector<vec> yss;
        yss.push_back(ys);
        return KUKADU_SHARED_PTR<ControllerResult>(new ControllerResult(retT, yss, true));

    }

    std::vector<arma::vec> TrajectoryBasedReward::computeFun(arma::vec t) {

        vector<vec> retYs;
        for(int i = 0; i < degOfFreedom; ++i)
            retYs.push_back(vec(t.n_elem));

        for(int i = 0; i < t.n_elem; ++i) {
            vec funVal = computeFun(t(i));
            // very inefficient --> switch to arma::mat
            for(int j = 0; j < degOfFreedom; ++j) {
                vec yVec = retYs.at(j);
                yVec(i) = funVal(j);
                retYs.at(j) = yVec;
            }
        }

        return retYs;

    }

}
