#include <limits>
#include <sstream>
#include <armadillo>
#include <boost/progress.hpp>
#include <kukadu/utils/utils.hpp>
#include <kukadu/manipulation/skillfactory.hpp>
#include <kukadu/manipulation/skillexporter.hpp>
#include <kukadu/manipulation/autonomoustester.hpp>

using namespace std;
using namespace arma;

namespace kukadu {

    AutonomousTester::AutonomousTester(kukadu::StorageSingleton& storage, std::string skill, std::vector<KUKADU_SHARED_PTR<kukadu::Hardware> > availableHardware,
                                       std::pair<std::vector<int>, std::vector<arma::mat> >& skillData,
                                       std::vector<long long int> sampleStartTimes, std::vector<long long int> sampleEndTimes, long long int timeStep)
        : StorageHolder(storage) {

        auto& skillFactory = SkillFactory::get();

        try {
            if(availableHardware.size())
                storedSkills[skill] = skillFactory.loadSkill(skill, availableHardware.front());
            else
                cerr << "(AutonomousTester) no hardware passed" << endl;
        } catch(KukaduException& ex) {
            cerr << "(AutonomousTester) skill controller not registered" << endl;
        }

        skillsData[skill] = skillData;

        timeSteps[skill] = timeStep;
        skillSampleStartTimes[skill] = sampleStartTimes;
        skillSampleEndTimes[skill] = sampleEndTimes;

        // set sliding window to 2 seconds
        windowTime = 2000;

        SkillExporter exporter(getStorage());
        auto executionsList = exporter.getSkillExecutions(skill);

        cout << "loading fingerprint for skill " << skill << endl;
        boost::progress_display show_progress(executionsList.size());

        long long int maxDuration = 0;
        for(auto& execution : executionsList)
            if(maxDuration < (get<1>(execution) - get<0>(execution)))
                maxDuration = get<1>(execution) - get<0>(execution);

        long long int maxTimeCount = ceil((double) maxDuration / timeSteps[skill]);

        auto mappings = mapFunctionIdToFingerPrintRow();
        functionIdsToRows = mappings.first;
        functionRowsToIds = mappings.second;

        bool firstFingerPrint = true;

        mat skillFingerPrint;
        mat skillFingerPrintStdDev;
        mat varTerm1;
        int successfulExecutionsCount = 0;
        for(auto& execution : executionsList) {

            mat currentFingerPrint = computeFingerPrint(get<0>(execution), get<1>(execution), get<0>(execution), get<1>(execution), maxTimeCount, timeSteps[skill]);

            if(get<2>(execution)) {
                if(firstFingerPrint) {
                    skillFingerPrint = currentFingerPrint;
                    varTerm1 = arma::pow(currentFingerPrint, 2.0);
                } else {
                    skillFingerPrint += currentFingerPrint;
                    varTerm1 += arma::pow(currentFingerPrint, 2.0);
                }

                firstFingerPrint = false;
                ++successfulExecutionsCount;
            }
            ++show_progress;

        }

        fingerPrintColCounts[skill] = skillFingerPrint.n_cols;

        skillFingerPrint /= successfulExecutionsCount;
        skillFingerPrints[skill] = skillFingerPrint;

        skillFingerPrintStdDev = arma::sqrt(varTerm1 / successfulExecutionsCount - arma::pow(skillFingerPrint, 2.0));
        skillFingerPrintStdDeviations[skill] = skillFingerPrintStdDev;

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

        // sanity ensurance in case the window is too long
        endTime = min(endTime, skillEndTime);

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

        return {dataPrint, arma::sqrt(standardDeviation)};

    }

    void AutonomousTester::testSkill(std::string id) {

        if(skillsData.find(id) != skillsData.end()) {

            int simulatedId = 42;

            // replace skillsData[id].second.at(5) by actually executed data --> make sure that the timestep is the same
            auto failurePlaces = computeFailureProb(id, skillsData[id].second.at(simulatedId));

            // replace it by the real start time
            auto skillStartTime = skillSampleStartTimes[id].at(simulatedId);
            auto skillEndTime = skillSampleEndTimes[id].at(simulatedId);

            for(auto& failure : failurePlaces) {

                int failureTimeIdx = failure.first;
                double failureProb = 1.0 - failure.second;

                long long int windowStartTime = skillStartTime + timeSteps[id] * (failureTimeIdx + 1) - windowTime;

                if(windowStartTime < skillStartTime)
                    windowStartTime = skillStartTime;

                long long int windowEndTime = skillStartTime + timeSteps[id] * (failureTimeIdx + 1);

                auto executionPrint = computeFingerPrint(windowStartTime, windowEndTime, skillStartTime, skillEndTime, fingerPrintColCounts[id], timeSteps[id]);
                auto runningFunctions = loadRunningFunctions(windowStartTime, windowEndTime, skillStartTime, skillEndTime);

                auto normalizedWindowStartTime = windowStartTime - skillStartTime;
                auto normalizedWindowEndTime = windowEndTime - skillStartTime;

                int windowStartIdx = floor((double) normalizedWindowStartTime / timeSteps[id]);
                int windowEndIdx = ceil((double) normalizedWindowEndTime / timeSteps[id]);

                if(windowEndIdx >= executionPrint.n_cols)
                    throw KukaduException("(AutonomousTester) something is wrong with the window index computeation");

                auto dataFingerPrint = skillFingerPrints[id];

                if(!executionPrint.n_cols)
                    throw KukaduException("(AutonomousTester) desired window is too small");

                // integrate the fingerprint of the current execution over the given window
                mat fakeStdDev;
                vec integratedExecutionPrint = integrateFingerPrint(executionPrint, fakeStdDev, windowStartIdx, windowEndIdx, false).first;

                // integrate the database fingerprint over the given window
                auto integratedDataPrintWithStdDev = integrateFingerPrint(dataFingerPrint, skillFingerPrintStdDeviations[id], windowStartIdx, windowEndIdx, true);
                vec& integratedDataPrint = integratedDataPrintWithStdDev.first;
                vec& integratedDataPrintStdDev = integratedDataPrintWithStdDev.second;

                auto maliciousFunctions = extractDeviatingFunctions(integratedExecutionPrint, integratedDataPrint, integratedDataPrintStdDev);

                cout << "the following functions might have caused the problem" << endl;
                for(auto& maliciousFunction : maliciousFunctions) {
                    auto functionName = getStorage().getCachedLabel("software_functions", "id", "name", maliciousFunction.first);
                    cout << "function (id, name, count): (" << maliciousFunction.first << ", " <<
                            functionName << ", " <<
                            maliciousFunction.second << ")" << endl;
                }

            }

        } else
            throw KukaduException("(AutonomousTester) skill data not available");

    }

    std::vector<std::pair<int, double> > AutonomousTester::extractDeviatingFunctions(arma::vec& executedPrint, arma::vec& dataPrint, arma::vec& dataStdDev) {

        vector<pair<int, double> > maliciousFunctions;

        vec diff = (executedPrint - dataPrint);
        vec fingerPrintDiff = diff / dataStdDev;
        for(int j = 0; j < fingerPrintDiff.n_elem; ++j) {

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

    std::vector<std::pair<int, double> > AutonomousTester::computeFailureProb(std::string skill, arma::mat& execution) {

        auto& skillData = skillsData[skill];
        vec skillDistances(skillData.first.size());
        skillDistances.fill(0.0);

        vector<double> distDevel;
        for(int i = 0; i < execution.n_cols; ++i)
            distDevel.push_back(computeBestDistance(skill, i, i + 1, skillDistances, execution));

        //return { {1450, 0.02} };
        return { {500, 0.02} };

    }

}
