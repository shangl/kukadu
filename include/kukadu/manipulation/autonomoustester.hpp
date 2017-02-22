#ifndef KUKADU_AUTONOMOUS_TEST_H
#define KUKADU_AUTONOMOUS_TEST_H

#include <map>
#include <vector>
#include <utility>
#include <armadillo>
#include <kukadu/types/kukadutypes.hpp>
#include <kukadu/control/controller.hpp>

namespace kukadu {

    class AutonomousTester : public StorageHolder {

        long long int windowTime;

        std::map<std::string, long long int> timeSteps;
        std::map<std::string, KUKADU_SHARED_PTR<kukadu::Controller> > storedSkills;
        std::map<std::string, std::pair<std::vector<int>, std::vector<arma::mat> > > skillsData;

        std::map<std::string, std::vector<long long int> > skillSampleStartTimes;
        std::map<std::string, std::vector<long long int> > skillSampleEndTimes;

        double computeBestDistance(std::string skill, arma::mat& execution);
        double computeBestDistance(std::string skill, int prevEndTimeStep, int desiredEndTimeStep, arma::vec& prevDistances, arma::mat& execution);

        std::string generateFunctionQuery(long long int skillStartTime, long long int skillEndTime, long long int windowStartTimeStamp, long long int windowEndTimeStamp);

        std::vector<std::pair<int, int> > loadRunningFunctions(long long int startTimeStep, long long int endTimeStep, long long skillStartTime, long long skillEndTime);

        arma::mat computeFingerPrint(long long int startTime, long long int endTime, long long int timeCount, long long int deltaT);

    public:

        AutonomousTester(kukadu::StorageSingleton& storage, std::string skill, std::vector<KUKADU_SHARED_PTR<kukadu::Hardware> > availableHardware, std::pair<std::vector<int>, std::vector<arma::mat> >& skillData,
                         std::vector<long long int> sampleStartTimes, std::vector<long long int> sampleEndTimes, long long int timeStep);

        void testSkill(std::string id);

        virtual std::vector<std::pair<int, double> > computeFailureProb(std::string skill, arma::mat& execution);

    };

}

#endif
