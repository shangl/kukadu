#include <kukadu/control/dmp.hpp>
#include <kukadu/types/sensordata.hpp>
#include <kukadu/storage/sensorstorage.hpp>
#include <kukadu/learning/rl/dmprewardcomputer.hpp>

using namespace std;
using namespace arma;

namespace kukadu {

    DmpRewardComputer::DmpRewardComputer(string file, double az, double bz, double timeStep, int degOfFreedom, double tmax, double step) : TrajectoryBasedReward(degOfFreedom, tmax, step) {

        this->file = file;
        this->az = az;
        this->bz = bz;
        this->timeStep = timeStep;
        KUKADU_SHARED_PTR<ControlQueue> pcq = KUKADU_SHARED_PTR<ControlQueue>(new PlottingControlQueue(degOfFreedom, timeStep));

        cout << "(DmpRewardComputer) starting execution of sample trajectory with timeStep size " << timeStep << endl;
        KUKADU_SHARED_PTR<SensorData> data = SensorStorage::readStorage(pcq, file);
        auto timesInMilliseconds = data->getTimeInMilliSeconds();
        auto times = convertAndRemoveOffset(timesInMilliseconds);
        KUKADU_SHARED_PTR<JointDMPLearner> dmpLearner = KUKADU_SHARED_PTR<JointDMPLearner>(new JointDMPLearner(az, bz, times, data->getJointPos()));
        KUKADU_SHARED_PTR<Dmp> finalDmp = dmpLearner->fitTrajectories();
        DMPExecutor execDmp(finalDmp, pcq);
        executionResult = execDmp.executeTrajectory(0, 0, finalDmp->getTmax(), 0.1, 0.1);

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
