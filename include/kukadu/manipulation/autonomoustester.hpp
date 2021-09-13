#ifndef KUKADU_AUTONOMOUS_TEST_H
#define KUKADU_AUTONOMOUS_TEST_H

#include <map>
#include <mutex>
#include <random>
#include <vector>
#include <utility>
#include <armadillo>
#include <kukadu/types/kukadutypes.hpp>
#include <kukadu/control/controller.hpp>

namespace kukadu {

    class AutonomousTester : public StorageHolder {

        bool simulate;

        int simlatedIdxWindow;
        long long int windowTime;

        std::mutex momCallerMutex;

        std::map<int, int> functionIdsToRows;
        std::map<int, int> functionRowsToIds;

        std::map<std::string, long long int> timeSteps;
        std::map<std::string, int> fingerPrintColCounts;
        std::map<std::string, arma::mat> skillMedianFingerPrints;
        std::map<std::string, arma::mat> skillFingerPrintStdDeviations;
        std::map<std::string, std::vector<arma::mat> > skillFingerPrints;
        std::map<std::string, KUKADU_SHARED_PTR<kukadu::Controller> > storedSkills;
        std::map<std::string, std::pair<std::vector<int>, std::vector<arma::mat> > > skillsData;

        std::vector<int> simulatedFaultyFunctionsGroundTruth;
        std::map<std::string, std::vector<int> > skillSimulatedFunctionRows;
        std::map<std::string, std::vector<double> > skillSimulatedFunctionMeans;
        std::map<std::string, std::vector<double> > skillSimulatedFunctionStdDevs;

        std::default_random_engine generator;

        std::map<std::string, std::vector<long long int> > skillSampleStartTimes;
        std::map<std::string, std::vector<long long int> > skillSampleEndTimes;

        double computeBestDistance(std::string skill, arma::mat& execution);
        double computeBestDistance(std::string skill, int prevEndTimeStep, int desiredEndTimeStep, arma::vec& prevDistances, arma::mat& execution);

        std::string generateFunctionQuery(long long int skillStartTime, long long int skillEndTime, long long int windowStartTimeStamp, long long int windowEndTimeStamp);

        std::vector<std::pair<int, int> > loadRunningFunctions(long long int startTimeStep, long long int endTimeStep, long long skillStartTime, long long skillEndTime);

        // returns the integrated fingerprint and the standardDeviation according to the single variances and Gaussian propagation of uncertainty
        std::pair<arma::vec, arma::vec> integrateFingerPrint(arma::mat& fingerprint, arma::mat& stdDev, int startIdx, int endIdx, bool computeStdDeviation = false);
        arma::mat computeFingerPrint(long long int startTime, long long int endTime, long long skillStartTime, long long skillEndTime, long long int timeCount, long long int deltaT);

        std::pair<std::map<int, int>, std::map<int, int> > mapFunctionIdToFingerPrintRow();

        // extracts and sorts the functions that deviate from the execution (returns the deviation in multiples of the standard deviation and the corresponding function id)
        std::vector<std::pair<int, double> > extractDeviatingFunctions(arma::vec& executedPrint, arma::vec& dataPrint, arma::vec& dataStdDev);

        arma::vec computeObservationLikelihood(std::string skillId, bool success, arma::mat& executedObservation, int windowStartIdx, int windowEndIdx, bool printFeedback = false);

        void loadSkillFingerPrintDb(std::string skill, std::vector<arma::mat>& fingerPrints);

        void construct(bool simulate, std::vector<int> simulatedFaultyFunctionsGroundTruth);

        arma::mat generateSimulatedSample(std::vector<int> usedFunctionRows, std::vector<double> functionMeans, std::vector<double> functionStdDev, int durationIndexCount);

        bool simulateSuccess(std::string skill);

        arma::vec computeBayesianUpdate(arma::vec likelihood, arma::vec prior);

        double computeInformatioinGain(std::string skill, arma::vec& currentBlameProbability);

        std::pair<std::string, std::map<std::string, double> > maximizeInformationGains(arma::vec& currentBlameProbability);

    public:

        AutonomousTester(kukadu::StorageSingleton& storage, bool simulate = false, std::vector<int> simulatedFaultyFunctionsGroundTruth = {});

        // executes (or simulates) on skill and returns the observation likelihood
        arma::vec testSkill(std::string id);

        virtual std::vector<std::pair<int, double> > computeFailureProb(std::string skill, arma::mat& execution);

        void setSimulate(bool simulate);

        void addSkill(std::string skill, KUKADU_SHARED_PTR<kukadu::Controller> skillController, std::pair<std::vector<int>, std::vector<arma::mat> >& skillData,
                      std::vector<long long int> sampleStartTimes, std::vector<long long int> sampleEndTimes, long long int timeStep);

        void addSimulatedSkill(std::string skillName, std::vector<int> usedFunctionRows, std::vector<double> functionMeans, std::vector<double> functionVariances, int numberOfSamples, int durationIndexCount);

        std::vector<int> testRobot(std::string firstSkill = "");

    };

}

#endif
