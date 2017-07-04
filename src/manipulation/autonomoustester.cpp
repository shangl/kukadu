#include <list>
#include <limits>
#include <sstream>
#include <json.hpp>
#include <armadillo>
#include <curl/curl.h>
#include <boost/progress.hpp>
#include <kukadu/utils/utils.hpp>
#include <kukadu/statistics/statistics.hpp>
#include <kukadu/manipulation/skillfactory.hpp>
#include <kukadu/manipulation/skillexporter.hpp>
#include <kukadu/manipulation/autonomoustester.hpp>

using namespace std;
using namespace arma;

using json = nlohmann::json;

namespace kukadu {

    AutonomousTester::AutonomousTester(kukadu::StorageSingleton& storage, bool simulate, std::vector<int> simulatedFaultyFunctionsGroundTruth) : StorageHolder(storage) {

        construct(simulate, simulatedFaultyFunctionsGroundTruth);

    }

    void AutonomousTester::construct(bool simulate, std::vector<int> simulatedFaultyFunctionsGroundTruth) {

        this->simulate = simulate;

        // set sliding window to 2 seconds
        windowTime = 2000;
        simlatedIdxWindow = 100;

        this->simulatedFaultyFunctionsGroundTruth = simulatedFaultyFunctionsGroundTruth;

        auto mappings = mapFunctionIdToFingerPrintRow();
        functionIdsToRows = mappings.first;
        functionRowsToIds = mappings.second;

    }

    void AutonomousTester::setSimulate(bool simulate) {
        this->simulate = simulate;
    }

    arma::mat AutonomousTester::generateSimulatedSample(std::vector<int> usedFunctionRows, std::vector<double> functionMeans, std::vector<double> functionStdDev, int durationIndexCount) {

        mat newSample(functionIdsToRows.size(), durationIndexCount);
        newSample.fill(0.0);

        for(int j = 0; j < usedFunctionRows.size(); ++j) {

            normal_distribution<double> distribution(functionMeans.at(j), functionStdDev.at(j));

            for(int k = 0; k < durationIndexCount; ++k) {
                double randomVal = distribution(generator);
                newSample(usedFunctionRows.at(j), k) = randomVal;
            }

        }

        return newSample;

    }

    bool AutonomousTester::simulateSuccess(std::string skill) {

        for(auto& groundTruthFunctionIdx : simulatedFaultyFunctionsGroundTruth) {

            // if the ground truth function is executed in the simulated skill --> return failure
            if(std::find(skillSimulatedFunctionRows[skill].begin(), skillSimulatedFunctionRows[skill].end(), groundTruthFunctionIdx) != skillSimulatedFunctionRows[skill].end())
                return false;

        }

        return true;

    }

    void AutonomousTester::addSimulatedSkill(std::string skillName, std::vector<int> usedFunctionRows, std::vector<double> functionMeans, std::vector<double> functionVariances, int numberOfSamples, int durationIndexCount) {

        if(usedFunctionRows.size() != functionMeans.size() || functionMeans.size() != functionVariances.size())
            throw KukaduException("(AutonomousTester) simulation data not complete");

        vector<mat> skillData;
        vector<int> skillSuccess;
        cout << "generating data for simulated skill " << skillName << endl;
        boost::progress_display show_progress(numberOfSamples);
        for(int i = 0; i < numberOfSamples; ++i) {

            // assume all samples are successful
            skillSuccess.push_back(1);

            skillData.push_back(generateSimulatedSample(usedFunctionRows, functionMeans, functionVariances, durationIndexCount));
            ++show_progress;

        }

        skillSimulatedFunctionRows[skillName] = usedFunctionRows;
        skillSimulatedFunctionMeans[skillName] = functionMeans;
        skillSimulatedFunctionStdDevs[skillName] = functionVariances;

        skillsData[skillName] = {skillSuccess, skillData};

        loadSkillFingerPrintDb(skillName, skillData);

    }

    void AutonomousTester::loadSkillFingerPrintDb(std::string skill, std::vector<arma::mat>& fingerPrints) {

        bool firstFingerPrint = true;

        mat skillFingerPrint;
        mat skillFingerPrintStdDev;
        mat varTerm1;
        int successfulExecutionsCount = 0;
        for(auto& currentFingerPrint : fingerPrints) {

            if(firstFingerPrint) {
                skillFingerPrint = currentFingerPrint;
                varTerm1 = arma::pow(currentFingerPrint, 2.0);
            } else {
                skillFingerPrint += currentFingerPrint;
                varTerm1 += arma::pow(currentFingerPrint, 2.0);
            }

            firstFingerPrint = false;
            ++successfulExecutionsCount;

            skillFingerPrints[skill].push_back(currentFingerPrint);

        }

        fingerPrintColCounts[skill] = skillFingerPrint.n_cols;

        skillFingerPrint /= successfulExecutionsCount;
        skillMedianFingerPrints[skill] = skillFingerPrint;

        skillFingerPrintStdDev = arma::sqrt(varTerm1 / successfulExecutionsCount - arma::pow(skillFingerPrint, 2.0));
        skillFingerPrintStdDeviations[skill] = skillFingerPrintStdDev;

    }

    void AutonomousTester::addSkill(std::string skill, KUKADU_SHARED_PTR<kukadu::Controller> skillController, std::pair<std::vector<int>, std::vector<arma::mat> >& skillData,
                  std::vector<long long int> sampleStartTimes, std::vector<long long int> sampleEndTimes, long long int timeStep) {

        if(sampleStartTimes.size() != sampleEndTimes.size() || sampleStartTimes.size() != skillData.first.size()
                || sampleStartTimes.size() != skillData.second.size())
                throw KukaduException("(AutonomousTester) add skill function needs the same dimensions for all samples");

        if(!simulate)
            storedSkills[skill] = skillController;

        skillsData[skill] = skillData;

        timeSteps[skill] = timeStep;
        skillSampleStartTimes[skill] = sampleStartTimes;
        skillSampleEndTimes[skill] = sampleEndTimes;

        cout << "loading fingerprint for skill " << skill << endl;
        boost::progress_display show_progress(skillData.first.size());

        long long int maxTimeCount = skillData.second.front().n_cols;

        vector<mat> prints;
        mat currentFingerPrint;
        for(int i = 0; i < sampleStartTimes.size(); ++i) {

            if(sampleEndTimes.at(i)) {

                currentFingerPrint = computeFingerPrint(sampleStartTimes.at(i), sampleEndTimes.at(i), sampleStartTimes.at(i), sampleEndTimes.at(i), maxTimeCount, timeStep);
                prints.push_back(currentFingerPrint);

            }

            ++show_progress;

        }

        loadSkillFingerPrintDb(skill, prints);

    }

    std::pair<std::map<int, int>, std::map<int, int> > AutonomousTester::mapFunctionIdToFingerPrintRow() {

        map<int, int> functionIdsToRows;
        map<int, int> functionRowsToIds;

        int rowIdx = 0;
        auto functionIdQuery = getStorage().executeQuery("select distinct(id) from software_functions order by id");
        while(functionIdQuery->next()) {
            int functionId = functionIdQuery->getInt("id");
            functionIdsToRows[functionId] = rowIdx;
            functionRowsToIds[rowIdx] = functionId;
            ++rowIdx;
        }

        return {functionIdsToRows, functionRowsToIds};

    }

    arma::mat AutonomousTester::computeFingerPrint(long long int startTime, long long int endTime,
                                                   long long int skillStartTime, long long int skillEndTime,
                                                   long long int timeCount, long long int deltaT) {

        auto& storage = getStorage();
        auto runningFunctions = generateFunctionQuery(skillStartTime, skillEndTime, startTime, endTime);

        map<int, int>& mapFunctionIdToIdx = functionIdsToRows;
        int functionCount = mapFunctionIdToIdx.size();

        arma::mat statisticsData(functionCount, timeCount);
        statisticsData.fill(0.0);

        auto statisticsQuery = storage.executeQuery(runningFunctions);
        while(statisticsQuery->next()) {

            // load current function call
            int currentFunctionId = statisticsQuery->getInt("function_id");
            int currentFunctionIdx = -1;

            if(mapFunctionIdToIdx.find(currentFunctionId) != mapFunctionIdToIdx.end()) {

                currentFunctionIdx = mapFunctionIdToIdx[currentFunctionId];

                long long int currentStartTime = statisticsQuery->getInt64("start_timestamp");
                long long int currentEndTime = statisticsQuery->getInt64("end_timestamp");
                int callCount = statisticsQuery->getInt("cnt");

                if(!currentEndTime)
                    currentEndTime = skillEndTime;

                long long int normalizedStartTime = currentStartTime - skillStartTime;
                long long int normalizedEndTime = currentEndTime - skillStartTime;

                long long int delta = normalizedEndTime - normalizedStartTime;

                // another sanity check
                if(normalizedStartTime > normalizedEndTime && normalizedEndTime > 0)
                    throw KukaduException("(AutonomousTester) database inconsistency --> start time of function call is bigger than end time");

                // compute index where the data belongs to
                int currentStartIndex = (double) normalizedStartTime / deltaT;
                int correctionIndex = abs(min(0, currentStartIndex));
                currentStartIndex = max(0, currentStartIndex);

                // compute on how many cells the calls are distributed
                int distributionCount = max(1.0, ceil((double) delta / deltaT));

                // for fingerprinting --> do not distribute them, but just sum them up
                double distributedCount = callCount;

                // add the distributed count of the function call to the cell range (many calls can be there at the same time --> sum them up)
                for(int currentIndex = currentStartIndex, i = correctionIndex; i < distributionCount && currentIndex < statisticsData.n_cols; ++currentIndex, ++i)
                    statisticsData(currentFunctionIdx, currentIndex) += distributedCount;

            } else
                throw KukaduException("(AutonomousTester) precondition violated - bug in code");

        }

        if(armadilloHasNan(statisticsData))
            throw KukaduException("(AutonomousTester) post condition violated - check for bug in code");

        return statisticsData;

    }

    std::pair<arma::vec, arma::vec> AutonomousTester::integrateFingerPrint(arma::mat& fingerprint, arma::mat& stdDev, int startIdx, int endIdx, bool computeStdDeviation) {

        vec dataPrint;
        vec standardDeviation;
        bool firstRun = true;
        for(int colIdx = startIdx; colIdx <= endIdx; ++colIdx) {
            if(firstRun) {
                dataPrint = fingerprint.col(colIdx);
                if(computeStdDeviation)
                    standardDeviation = arma::pow(stdDev.col(colIdx), 2.0);
                else
                    standardDeviation = vec(dataPrint.n_elem);
            } else {
                dataPrint += fingerprint.col(colIdx);
                if(computeStdDeviation)
                    standardDeviation += arma::pow(stdDev.col(colIdx), 2.0);
            }
            firstRun = false;
        }

        return {dataPrint / (endIdx - startIdx), arma::sqrt(standardDeviation)};

    }

    double AutonomousTester::computeInformatioinGain(std::string skill, arma::vec& currentBlameProbability) {

        auto& collectedFingerPrints = skillFingerPrints[skill];

        double currentEntropy = computeEntropy(currentBlameProbability);

        double expectedEntropy = 0.0;
        int entropiesCount = 0;
        for(auto& fingerPrintSample : collectedFingerPrints) {

            // only run once for all samples (doesn't change anything for simulation)
            if(simulate) {

                // compute for unsuccessful
                vec likelihood = computeObservationLikelihood(skill, false, fingerPrintSample, 0, fingerPrintSample.n_cols - 1, false);
                vec posterior = computeBayesianUpdate(likelihood, currentBlameProbability);

                expectedEntropy += computeEntropy(posterior);
                ++entropiesCount;

                // compute for successful
                likelihood = computeObservationLikelihood(skill, true, fingerPrintSample, 0, fingerPrintSample.n_cols - 1, false);
                posterior = computeBayesianUpdate(likelihood, currentBlameProbability);

                expectedEntropy += computeEntropy(posterior);
                ++entropiesCount;

            } else {

                for(int i = 0; i < fingerPrintSample.n_cols; i += 200) {

                    // vary over all times and success
                    vec likelihood = computeObservationLikelihood(skill, false, fingerPrintSample, 0, fingerPrintSample.n_cols - 1, false);
                    vec posterior = computeBayesianUpdate(likelihood, currentBlameProbability);

                    expectedEntropy += computeEntropy(posterior);
                    ++entropiesCount;

                    likelihood = computeObservationLikelihood(skill, true, fingerPrintSample, 0, fingerPrintSample.n_cols - 1, false);
                    posterior = computeBayesianUpdate(likelihood, currentBlameProbability);

                    expectedEntropy += computeEntropy(posterior);
                    ++entropiesCount;

                }

            }

        }

        expectedEntropy /= entropiesCount;

        return currentEntropy - expectedEntropy;

    }

    std::pair<std::string, std::map<string, double> > AutonomousTester::maximizeInformationGains(arma::vec& currentBlameProbability) {

        map<string, double> informationGains;
        cout << "computing expected information gains" << endl;
        boost::progress_display show_progress(skillMedianFingerPrints.size());
        for(auto& skillPrint : skillMedianFingerPrints) {
            informationGains[skillPrint.first] = computeInformatioinGain(skillPrint.first, currentBlameProbability);
            ++show_progress;
        }

        auto maxGain = std::max_element(informationGains.begin(), informationGains.end(),
            [](const pair<std::string, double>& p1, const pair<std::string, double>& p2) {
                return p1.second < p2.second; });

        return {maxGain->first, informationGains};

    }

    arma::vec AutonomousTester::computeBayesianUpdate(arma::vec likelihood, arma::vec prior) {

        // Bayesian update
        prior %= likelihood;

        // renormalize
        prior /= sum(prior);

        return prior;

    }

    vector<int> AutonomousTester::testRobot(std::string firstSkill) {

        int speedUpBias = 10;

        vec pBlame(functionIdsToRows.size());
        pBlame.fill(1.0 / functionIdsToRows.size());

        ofstream pBlameStream;
        pBlameStream.open("/tmp/tester-pblame");

        ofstream igStream;
        igStream.open("/tmp/tester-igs");

ofstream skillStream;
skillStream.open("/tmp/tester-skills");

        // get ordered list of all skills (not sure if foreach ensures this for every run)
        vector<string> orderedSkillsList;
        cout << "skills used for testing:" << endl;
        skillStream << "skills used for testing:" << endl;
        for(auto& s : skillMedianFingerPrints) {
            orderedSkillsList.push_back(s.first);
            cout << s.first << endl;
            skillStream << s.first << endl;
        }

        std::pair<std::string, std::map<string, double> > igs;
        string maxGainSkill;
        double maxGainValue;

        string selectedSkill;
        if(firstSkill != "") {
            selectedSkill = firstSkill;
        } else {

            igs = maximizeInformationGains(pBlame);
            maxGainSkill = igs.first;
            maxGainValue = igs.second[igs.first];

            for(auto& skillName : orderedSkillsList) {
                auto ig = igs.second[skillName];
                igStream << ig << "\t";
            }
            igStream << endl;

            selectedSkill = maxGainSkill;

skillStream << selectedSkill << ":" << endl;
cout  << selectedSkill << " (" << maxGainValue << "): " << endl;

        }

        int runCount = 100;
        for(int i = 0; i < runCount; ++i) {

            auto observationLikelihood = testSkill(selectedSkill);
            for(int k = 0; k < speedUpBias; ++k) {

                pBlame = computeBayesianUpdate(observationLikelihood, pBlame);

                if(armadilloHasNan(pBlame)) {
                    cout << "schon wieder nan" << endl;
                    getchar();
                }

            }

            igs = maximizeInformationGains(pBlame);
            maxGainSkill = igs.first;
            maxGainValue = igs.second[igs.first];

            for(auto& skillName : orderedSkillsList) {
                auto ig = igs.second[skillName];
                igStream << ig << "\t";
            }
            igStream << endl;

            selectedSkill = maxGainSkill;

            vector<pair<int, double> > functionRanking;
            for(int i = 0; i < pBlame.n_elem; ++i)
                functionRanking.push_back({i, pBlame(i)});

            std::sort(functionRanking.begin(), functionRanking.end(), [](const pair<int, double>& a, const pair<int, double>& b) -> bool {
                return abs(a.second) > abs(b.second);
            });

            cout << "i think, the following functions may be broken:" << endl;
            for(int i = 0; i < 5; ++i) {
                auto functionName = getStorage().getCachedLabel("software_functions",
                                                                "id", "name",
                                                                functionRowsToIds[functionRanking.at(i).first]);
                cout << functionRowsToIds[functionRanking.at(i).first] << " (row " << functionRanking.at(i).first << ")"
                                                                       << " - " << functionName <<
                                                                    " (" << functionRanking.at(i).second <<
                                                                    ")" << endl;
            }

pBlameStream << pBlame.t();
pBlameStream.flush();
skillStream << selectedSkill << ":" << endl;
cout  << selectedSkill << " (" << maxGainValue << "): " << endl;

        }

pBlameStream.close();
skillStream.close();
igStream.close();

        return {};

    }

    arma::vec AutonomousTester::testSkill(std::string id) {

        if(skillFingerPrints.find(id) != skillFingerPrints.end()) {

            int simulatedId = 42;
            bool successful = false;

            long long int executedStartTime = 0;
            long long int executedEndTime = 0;
            if(simulate) {
                // if the system is simulated --> retrieve if the skill was executed successfully
                successful = simulateSuccess(id);
            } else {

                cout << "are you ready to execute skill " << id << "?" << endl;
                string s;
                cin >> s;
                storedSkills[id]->prepare();

                TimedObject t;
                executedStartTime = t.getCurrentTime();
                storedSkills[id]->execute();
                executedEndTime = t.getCurrentTime();
                cout << "was the skill successful? (0 = no, 1 = yes)" << endl;
                cin >> successful;

            }

            std::vector<std::pair<int, double> > failurePlaces;
            if(successful) {

                auto& skillData = skillsData[id];

                // the "failure place is the last index
                failurePlaces = {{skillData.second.front().n_cols - 1, 1.0}};

            } else {

                mat simulationFakeDataMat;
                if(!simulate)
                    failurePlaces = computeFailureProb(id, skillsData[id].second.at(simulatedId));
                else
                    failurePlaces = computeFailureProb(id, simulationFakeDataMat);

            }

            // if the mom doesnt see a failure, we assume its the last index
            if(!failurePlaces.size()) {
                failurePlaces = {{skillsData[id].second.front().n_cols - 1, 1.0}};
            }

            cout << "uses failure index " << failurePlaces.front().first << " which corresponds " <<
                 "time " << failurePlaces.front().first * timeSteps[id] / 1000 << " secnods" << endl;

            long long int skillStartTime = 0;
            long long int skillEndTime = 0;

            // only required if skill is actually executed
            if(!simulate) {
                skillStartTime = skillSampleStartTimes[id].at(simulatedId);
                skillEndTime = skillSampleEndTimes[id].at(simulatedId);
            } else {

                skillStartTime = executedStartTime;
                skillEndTime = executedEndTime;

            }

            int windowStartIdx = 0;
            int windowEndIdx = 0;

            mat executionPrint;
            if(!simulate) {
                executionPrint = computeFingerPrint(skillStartTime, skillEndTime, skillStartTime, skillEndTime, fingerPrintColCounts[id], timeSteps[id]);
            } else {
                executionPrint = generateSimulatedSample(skillSimulatedFunctionRows[id], skillSimulatedFunctionMeans[id], skillSimulatedFunctionStdDevs[id], skillMedianFingerPrints[id].n_cols);
            }

            // only considere the first on - break below
            for(auto& failure : failurePlaces) {

                int failureTimeIdx = failure.first;
                double failureProb = 1.0 - failure.second;

                if(!simulate) {

                    long long int windowStartTime = skillStartTime + timeSteps[id] * (failureTimeIdx + 1) - windowTime;

                    if(windowStartTime < skillStartTime)
                        windowStartTime = skillStartTime;

                    long long int windowEndTime = skillStartTime + timeSteps[id] * (failureTimeIdx + 1);

                    // auto runningFunctions = loadRunningFunctions(windowStartTime, windowEndTime, skillStartTime, skillEndTime);

                    auto normalizedWindowStartTime = windowStartTime - skillStartTime;
                    auto normalizedWindowEndTime = windowEndTime - skillStartTime;

                    if(!successful) {
                        // if skill was not successful, check the window before the found failure point
                        windowStartIdx = floor((double) normalizedWindowStartTime / timeSteps[id]);
                        windowEndIdx = ceil((double) normalizedWindowEndTime / timeSteps[id]);
                    } else {
                        // if the skill was successful --> compute the fingerprint for the whole skill
                        windowStartIdx = 0;
                        windowEndIdx = executionPrint.n_cols - 1;
                    }

                    if(windowEndIdx >= executionPrint.n_cols || windowStartIdx < 0)
                        throw KukaduException("(AutonomousTester) something is wrong with the window index computation");

                    if(!executionPrint.n_cols)
                        throw KukaduException("(AutonomousTester) desired window is too small");

                } else {

                    if(!successful) {
                        // if skill was not successful, check the window before the found failure point
                        windowStartIdx = max(0, windowEndIdx - simlatedIdxWindow);
                        windowEndIdx = failureTimeIdx;
                    } else {
                        // if the skill was successful --> compute the fingerprint for the whole skill
                        windowStartIdx = 0;
                        windowEndIdx = executionPrint.n_cols - 1;
                    }

                }

                break;

            }

            vec probObservationGivenFunctionFailure = computeObservationLikelihood(id, successful, executionPrint, windowStartIdx, windowEndIdx, true);

            if(armadilloHasInf(probObservationGivenFunctionFailure)) {
                cout << "lala" << endl;
            }

            return probObservationGivenFunctionFailure;

        } else
            throw KukaduException("(AutonomousTester) skill data not available");

    }

    arma::vec AutonomousTester::computeObservationLikelihood(std::string skillId, bool success, arma::mat& executedObservation, int windowStartIdx, int windowEndIdx, bool printFeedback) {

        double probThresh = 0.5;
        double minSuccObservationProb = 0.1;
        double minFailObservationProb = 0.6;
        double maxMultipleOfStdThresh = 7.0;

        vec likelihoodProbabilities(executedObservation.n_rows);

        // all functions that were not executed within the window yield an observation probability of 1 --> because they can't have anything to do with the observation
        likelihoodProbabilities.fill(0.5);

        // integrate the fingerprint of the current execution over the given window
        mat fakeStdDev;
        vec integratedExecutionPrint = integrateFingerPrint(executedObservation, fakeStdDev, windowStartIdx, windowEndIdx, false).first;

        auto dataFingerPrint = skillMedianFingerPrints[skillId];

        // integrate the database fingerprint over the given window
        auto integratedDataPrintWithStdDev = integrateFingerPrint(dataFingerPrint, skillFingerPrintStdDeviations[skillId], windowStartIdx, windowEndIdx, true);
        vec& integratedDataPrint = integratedDataPrintWithStdDev.first;
        vec& integratedDataPrintStdDev = integratedDataPrintWithStdDev.second;

        auto maliciousFunctions = extractDeviatingFunctions(integratedExecutionPrint, integratedDataPrint, integratedDataPrintStdDev);

        double maxDeviation = -1.0;
        for(auto& maliciousFunction : maliciousFunctions)
            maxDeviation = max(abs(maxDeviation), abs(maliciousFunction.second));

        bool printDebug = false;
        if(printFeedback)
            cout << "the following functions might have caused the problem" << endl;
        for(auto& maliciousFunction : maliciousFunctions) {

            auto functionName = getStorage().getCachedLabel("software_functions", "id", "name", maliciousFunction.first);
            auto& functionRow = functionIdsToRows[maliciousFunction.first];

            if(success) {

                // if successful - the likelihood of observing the data if the function is malicious should be low
               likelihoodProbabilities(functionRow) = min(probThresh, max(minSuccObservationProb, probThresh * (abs(maliciousFunction.second) / maxDeviation)));

            } else {

                // if not successful - the likelihood of observing the data if the function is malicious should be proportional to the deviation of the fingerprint (but at least probThresh)
                likelihoodProbabilities(functionRow) = min(1.0, max(minFailObservationProb, probThresh * (1.0 + abs(maliciousFunction.second) / maxMultipleOfStdThresh)));

            }

            if(printFeedback) {
                cout << "function (id, name, dev / stdDev";
                if(printDebug)
                    cout << ", fingerprint_row, average data, stddev data, average execution";
                cout << "): (" << maliciousFunction.first << ", " <<
                        functionName << ", " <<
                        maliciousFunction.second;
                if(printDebug)
                    cout << ", " << functionRow << integratedDataPrint(functionRow) << ", " << integratedDataPrintStdDev(functionRow) << ", " << integratedExecutionPrint(functionRow);
                cout << ")" << endl;
            }

        }

        return likelihoodProbabilities;

    }

    std::vector<std::pair<int, double> > AutonomousTester::extractDeviatingFunctions(arma::vec& executedPrint, arma::vec& dataPrint, arma::vec& dataStdDev) {

        vector<pair<int, double> > maliciousFunctions;

        vec diff = (executedPrint - dataPrint);
        vec fingerPrintDiff = diff / dataStdDev;

        double maxDev = 5.0;

        for(int i = 0; i < dataStdDev.n_elem; ++i) {

            // if nan
            if(fingerPrintDiff(i) != fingerPrintDiff(i))
                fingerPrintDiff(i) = maxDev;
            else if(fingerPrintDiff(i) < -maxDev)
                fingerPrintDiff(i) = -maxDev;
            else if(fingerPrintDiff(i) > maxDev)
                fingerPrintDiff(i) = maxDev;

        }

        for(int j = 0; j < fingerPrintDiff.n_elem; ++j) {

            if(diff(j) == 0.0)
                fingerPrintDiff(j) = 0.0;

            if(abs(fingerPrintDiff(j)) > 0.0) {
                int& functionId = functionRowsToIds[j];
                maliciousFunctions.push_back({functionId, fingerPrintDiff(j)});
            } else if(fingerPrintDiff(j) != fingerPrintDiff(j) && diff(j) > 0.0) {
                int& functionId = functionRowsToIds[j];
                maliciousFunctions.push_back({functionId, fingerPrintDiff(j)});
            }

        }

        std::sort(maliciousFunctions.begin(), maliciousFunctions.end(), [](const pair<int, double>& a, const pair<int, double>& b) -> bool {
            return abs(a.second) > abs(b.second);
        });

        return maliciousFunctions;

    }

    std::string AutonomousTester::generateFunctionQuery(long long int skillStartTime, long long int skillEndTime, long long int windowStartTimeStamp, long long int windowEndTimeStamp) {

        stringstream whereStr;
        whereStr << " where ";
        // function has to overlap with the window (greatest and least required because end_timestamp can be 0)
        whereStr << " ( greatest(start_timestamp, end_timestamp) >= " << windowStartTimeStamp << " and least(start_timestamp, greatest(start_timestamp, end_timestamp)) <= " << windowEndTimeStamp << ") ";
        // or it is started after the skill started and before the window ends and crashed (end_timestamp == 0)
        whereStr << " or ( start_timestamp >= " << skillStartTime << " and start_timestamp <= " << windowEndTimeStamp << " and (end_timestamp is null or end_timestamp = 0) ) ";

        stringstream s;
        //!!!!!!!!!!!!!!!! add crashed information (when end_timestamp is 0)
        s << "select function_id, start_timestamp, end_timestamp, cnt from " <<
                "(select function_id, start_timestamp, end_timestamp, 1 as cnt from software_statistics_mode0" <<
             whereStr.str() <<
              ") as mod1func";
        s <<
                " union (" <<
                "select function_id, start_timestamp, end_timestamp, cnt from software_statistics_mode1" <<
             whereStr.str() <<
            ")" <<
                " order by function_id asc, start_timestamp asc";

        return s.str();

    }

    std::vector<std::pair<int, int> > AutonomousTester::loadRunningFunctions(long long int startTimeStep, long long int endTimeStep, long long int skillStartTime, long long int skillEndTime) {

        auto& storage = getStorage();

        auto functionQuery = generateFunctionQuery(skillStartTime, skillEndTime, startTimeStep, endTimeStep);
        map<int, int> runningFunctions;
        auto queryRes = storage.executeQuery(functionQuery);
        while(queryRes->next()) {

            int currentFunctionId = queryRes->getInt("function_id");
            long long int currentStartTime = queryRes->getInt64("start_timestamp");
            long long int currentEndTime = queryRes->getInt64("end_timestamp");
            int currentCount = queryRes->getInt("cnt");

            // if function id is already included --> add up the counts
            if(runningFunctions.find(currentFunctionId) != runningFunctions.end())
                runningFunctions[currentFunctionId] += currentCount;
            // else, just set the value
            else
                runningFunctions[currentFunctionId] = currentCount;

        }

        // transform to a vector and sort it with the number of calls
        std::vector<std::pair<int, int> > retVec;
        for(auto& function : runningFunctions)
            retVec.push_back(function);

        std::sort(retVec.begin(), retVec.end(), [](const pair<int, int>& a, const pair<int, int>& b) -> bool {
            return a.first < b.first;
        });

        return retVec;

    }

    double AutonomousTester::computeBestDistance(std::string skill, int prevEndTimeStep, int desiredEndTimeStep, arma::vec& prevDistances, arma::mat& execution) {

        auto& skillData = skillsData[skill];

        double minDist = std::numeric_limits<double>::max();
        for(int i = prevEndTimeStep; i < execution.n_cols && i < desiredEndTimeStep; ++i) {

            vec currCol = execution.col(i);
            // compute all distances until the current time step
            for(int j = 0; j < skillData.first.size(); ++j) {
                // if that skill was successful
                if(!skillData.first.at(j)) {
                    auto diffVec = currCol - skillData.second.at(j).col(i);
                    vec distance = diffVec.t() * diffVec;
                    prevDistances(j) += distance(0);

                    if(i == (execution.n_cols - 1) || i == (desiredEndTimeStep - 1))
                        if(prevDistances(j) > 0.0 && prevDistances(j) < minDist)
                            minDist = prevDistances(j);

                }
            }
        }

        return minDist;

    }

    double AutonomousTester::computeBestDistance(std::string skill, arma::mat& execution) {

        auto& skillData = skillsData[skill];
        vec skillDistances(skillData.first.size());
        skillDistances.fill(0.0);

        double minDist = 0.0;
        for(int i = 0; i < execution.n_cols; ++i)
            minDist = computeBestDistance(skill, i, i + 1, skillDistances, execution);

        return minDist;

    }

    // horrible
    std::string momJsonResponse;
    void writefunction(void* ptr, size_t size, size_t nmemb, void *stream) {

        stringstream s;
        size_t totalSize = size * nmemb;
        char* castedPtr = (char*) ptr;
        for(size_t i = 0; i < totalSize; ++i)
            s << castedPtr[i];

        momJsonResponse = s.str();

    }

    std::vector<std::pair<int, double> > AutonomousTester::computeFailureProb(std::string skill, arma::mat& execution) {

        /*
        auto& skillData = skillsData[skill];
        vec skillDistances(skillData.first.size());
        skillDistances.fill(0.0);

        vector<double> distDevel;
        for(int i = 0; i < execution.n_cols; ++i)
            distDevel.push_back(computeBestDistance(skill, i, i + 1, skillDistances, execution));
        */

        if(simulate) {

            return { {execution.n_cols * 0.75, 0.02} };

        } else {

            momCallerMutex.lock();

            stringstream s;
            mat firstMom = execution.t();

            for(int i = 0; i < firstMom.n_rows; ++i){
                for(int j = 0; j < firstMom.n_cols; ++j)
                    s << firstMom(i, j) << " ";
                s << endl;
            }

            json j;
            j["skillname"] = skill;
            j["measurement_data"] = s.str();
            string requestData = j.dump();

            curl_global_init(CURL_GLOBAL_ALL);
            auto curl = curl_easy_init();
            if(curl) {

                struct curl_slist* headers = NULL;
                headers = curl_slist_append(headers, "Content-Type: application/json");

                /* pass our list of custom made headers */
                curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

                /* set the size of the postfields data */
                curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, requestData.size());

                /* First set the URL that is about to receive our POST. This URL can
                   just as well be a https:// URL if that is what should receive the
                   data. */
                curl_easy_setopt(curl, CURLOPT_URL, "http://138.232.64.154:9000");

                /* Now specify the POST data */
                curl_easy_setopt(curl, CURLOPT_POSTFIELDS, requestData.c_str());

                curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writefunction);

                /* Perform the request, res will get the return code */
                auto res = curl_easy_perform(curl);
                /* Check for errors */
                if(res != CURLE_OK)
                  cerr << "curl_easy_perform() failed: " << curl_easy_strerror(res) << endl;

                /* always cleanup */
                curl_easy_cleanup(curl);

                cout << "response received" << endl;

            }
            curl_global_cleanup();

            string resp = momJsonResponse;

            auto j2 = json::parse(resp);

            vector<int> indexes;
            if (j2.find("cuttimes") != j2.end()) {
                list<double> elements = j2["cuttimes"];
                for(auto& times : elements)
                    if(times > 100)
                        indexes.push_back(times);
                std::sort(indexes.begin(), indexes.end(), [](const int& a, const int& b) -> bool {
                    return a < b;
                });

            } else
                throw KukaduException("(AutnomousTester) something is wrong with keras");

            momCallerMutex.unlock();

            vector<pair<int, double> > finalResponse;
            for(auto& el : indexes)
                // no probabilities for now
                finalResponse.push_back({el, 1.0});

            return finalResponse;

        }

        return {};

    }

}
