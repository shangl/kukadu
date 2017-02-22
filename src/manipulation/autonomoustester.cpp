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

        // set sliding window to 4 seconds
        windowTime = 4000;

        SkillExporter exporter(getStorage());
        auto executionsList = exporter.getSkillExecutions(skill);

        cout << "loading fingerprint for skill " << skill << endl;
        boost::progress_display show_progress(executionsList.size());

        long long int maxDuration = 0;
        for(auto& execution : executionsList)
            if(maxDuration < (get<1>(execution) - get<0>(execution)))
                maxDuration = get<1>(execution) - get<0>(execution);

        long long int maxTimeCount = ceil((double) maxDuration / timeSteps[skill]);

        bool firstFingerPrint = true;
        mat skillFingerPrint;
        for(auto& execution : executionsList) {
            if(get<2>(execution)) {
                if(firstFingerPrint)
                    skillFingerPrint = computeFingerPrint(get<0>(execution), get<1>(execution), maxTimeCount, timeSteps[skill]);
                else
                    skillFingerPrint += computeFingerPrint(get<0>(execution), get<1>(execution), maxTimeCount, timeSteps[skill]);
                firstFingerPrint = false;
            }
            ++show_progress;
        }

    }

    arma::mat AutonomousTester::computeFingerPrint(long long int startTime, long long int endTime, long long int timeCount, long long int deltaT) {

        auto& storage = getStorage();
        auto runningFunctions = generateFunctionQuery(startTime, endTime, startTime, endTime);

        stringstream functionStream;
        functionStream << "select id from software_functions order by id";
        auto queryRes = storage.executeQuery(functionStream.str());
        map<int, int> mapFunctionIdToIdx;
        for(int i = 0; queryRes->next(); ++i) {
            int currentId = queryRes->getInt("id");
            mapFunctionIdToIdx[currentId] = i;
        }

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
                long long int delta = endTime - startTime;

                long long int normalizedStartTime = currentStartTime - startTime;
                long long int normalizedEndTime = currentEndTime - startTime;

                // another sanity check
                if(normalizedStartTime > normalizedEndTime && normalizedEndTime >= 0)
                    throw KukaduException("(AutonomousTester) database inconsistency --> start time of function call is bigger than end time");

                // compute index where the data belongs to
                int currentStartIndex = normalizedStartTime / deltaT;

                // compute on how many cells the calls are distributed
                int distributionCount = max(1.0, ceil((double) delta / deltaT));
                double distributedCount = (double) callCount / distributionCount;

                // add the distributed count of the function call to the cell range (many calls can be there at the same time --> sum them up)
                for(int currentIndex = currentStartIndex, i = 0; i < distributionCount && currentIndex < statisticsData.n_cols; ++currentIndex)
                    statisticsData(currentFunctionIdx, currentIndex) += distributedCount;

            } else
                throw KukaduException("(AutonomousTester) precondition violated - bug in code");

        }

        if(armadilloHasNan(statisticsData))
            throw KukaduException("(AutonomousTester) post condition violated - check for bug in code");

        return statisticsData;

    }

    void AutonomousTester::testSkill(std::string id) {

        if(skillsData.find(id) != skillsData.end()) {

            // replace skillsData[id].second.at(5) by actually executed data --> make sure that the timestep is the same
            auto failurePlaces = computeFailureProb(id, skillsData[id].second.at(5));

            // replace it by the real start time
            auto skillStartTime = skillSampleStartTimes[id].at(42);
            auto skillEndTime = skillSampleEndTimes[id].at(42);

            for(auto& failure : failurePlaces) {

                int failureTimeIdx = failure.first;
                double failureProb = 1.0 - failure.second;

                long long int windowStartTime = skillStartTime + timeSteps[id] * (failureTimeIdx + 1) - windowTime;
                if(windowStartTime < skillStartTime)
                    windowStartTime = skillStartTime;

                long long int windowEndTime = skillStartTime + timeSteps[id] * (failureTimeIdx + 1);

                auto runningFunctions = loadRunningFunctions(windowStartTime, windowEndTime, skillStartTime, skillEndTime);

                cout << "the following functions might have caused the problem" << endl;
                for(auto& fun : runningFunctions)
                    cout << "function (id, name, count): (" << fun.first << ", " <<
                            getStorage().getCachedLabel("software_functions", "id", "name", fun.first) << ", " <<
                            fun.second << ")" << endl;

            }


        } else
            throw KukaduException("(AutonomousTester) skill data not available");

    }

    std::string AutonomousTester::generateFunctionQuery(long long int skillStartTime, long long int skillEndTime, long long int windowStartTimeStamp, long long int windowEndTimeStamp) {

        long long int duration = skillEndTime - skillStartTime;

        stringstream s;
        //!!!!!!!!!!!!!!!! add crashed information (when end_timestamp is 0)
        s << "select function_id, start_timestamp, end_timestamp, cnt from" <<
                "(select function_id, start_timestamp, end_timestamp, 1 as cnt from software_statistics_mode0 where " <<
             " (end_timestamp >=  " << windowStartTimeStamp << " and end_timestamp <= " << windowEndTimeStamp << ") or ";
             if(duration > 0)
                s << " ((start_timestamp + " << duration << ") >=  " << skillStartTime << " and (start_timestamp + " << duration << ") <= " << skillEndTime << ") or ";
        s << " (start_timestamp >= " << windowStartTimeStamp << " and start_timestamp <= " << windowEndTimeStamp << ")" <<
                " union " <<
                "select function_id, start_timestamp, end_timestamp, cnt from software_statistics_mode1 where ";
            if(duration > 0)
               s << " ((start_timestamp + " << duration << ") >=  " << windowStartTimeStamp << " and (start_timestamp + " << duration << ") <= " << windowEndTimeStamp << ") or ";
        s << " (end_timestamp >=  " << windowStartTimeStamp << " and end_timestamp <= " << windowEndTimeStamp << ") or " <<
             " (start_timestamp >= " << windowStartTimeStamp << " and start_timestamp <= " << windowEndTimeStamp << ")" <<
                ") as union_statistics" <<
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
            return a.second > b.second;
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

        return { {1450, 0.02} };

    }

}
