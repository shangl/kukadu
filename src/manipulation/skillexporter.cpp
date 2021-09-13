#include <limits>
#include <sstream>
#include <armadillo>
#include <kukadu/utils.hpp>
#include <boost/progress.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/bzip2.hpp>
#include <kukadu/manipulation/skillexporter.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <kukadu/storage/sensorstoragesingleton.hpp>

using namespace std;
using namespace arma;
using namespace boost::iostreams;

namespace kukadu {

    void storeExecution(vector<KUKADU_SHARED_PTR<Hardware> >& hardwareInstances,
                        map<int, vector<vector<std::pair<long long int, arma::vec> > > >& allSamples,
                        vector<long long int>& unnormalizedStartTimes,
                        vector<long long int>& unnormalizedEndTimes,
                        int& executionId,
                        long long int& maxDuration, long long int minDeltaT,
                        vector<int>& sensorDataLengthPerHardware,
                        std::string file,
                        bool exportModuleStatistics,
                        bool compress);

    int augmentWithFunctionStatistics(long long startTime, long long endTime, arma::mat& currentData, long long int deltaT, stringstream &ostr, int maxFunctionId = -1);

    SkillExporter::SkillExporter(StorageSingleton &storage) : StorageHolder(storage) {

    }

    std::vector<std::pair<int, std::string> > SkillExporter::getSkills() {

        vector<pair<int, string> > skillList;

        auto& st = getStorage();

        auto queryRes = st.executeQuery("select distinct(skill_id), label from skills");
        while(queryRes->next()) {
            int id = queryRes->getInt("skill_id");
            string label = queryRes->getString("label");
            skillList.push_back({id, label});
        }

        return skillList;

    }

    std::pair<std::vector<int>, std::vector<arma::mat> > SkillExporter::loadExecutions(std::string directory, std::vector<long long int>& startTimes, std::vector<long long int>& endTimes, long long int& timeStep) {

        string detailedLabels = directory + "/labels_detail";
        ifstream labelsFile;
        labelsFile.open(detailedLabels.c_str());

        vector<int> successLabels;
        vector<mat> skillData;

        startTimes.clear();
        endTimes.clear();
        timeStep = -1;

        string labelsLine;
        vector<string> skillLines;
        while(getline(labelsFile, labelsLine))
            skillLines.push_back(labelsLine);

        boost::progress_display show_progress(skillLines.size());
        for(int i = 0; i < skillLines.size(); ++i) {

            string& labelsLine = skillLines.at(i);

            KukaduTokenizer tok(labelsLine);
            string skillFileName = directory + "/" + tok.next();
            int successLabel = atoi(tok.next().c_str());
            ifstream skillFile(skillFileName.c_str(), ios_base::in | ios_base::binary);
            boost::iostreams::filtering_istreambuf in;
            in.push(boost::iostreams::bzip2_decompressor());
            in.push(skillFile);

            stringstream s;
            boost::iostreams::copy(in, s);

            mat skillMat;
            skillMat.load(s);

            skillFile.close();

            skillData.push_back(skillMat);
            successLabels.push_back(successLabel);

            stringstream timingFileName;
            timingFileName << skillFileName << "_timing";
            ifstream timingFile;
            timingFile.open(timingFileName.str().c_str());

            string timingLine;
            while(getline(timingFile, timingLine)) {
                KukaduTokenizer timingTok(timingLine, "=");
                string identifier = timingTok.next();
                if(identifier == "startTimeStamp")
                    startTimes.push_back(atoll(timingTok.next().c_str()));
                else if(identifier == "endTimeStamp")
                    endTimes.push_back(atoll(timingTok.next().c_str()));
                else if(identifier == "timeStep")
                    timeStep = atoll(timingTok.next().c_str());
            }

            ++show_progress;

        }

        return {successLabels, skillData};

    }

    double SkillExporter::computeDistance(arma::mat& m1, arma::mat& m2) {

        double totalDist = 0.0;
        for(int i = 0; i < m1.n_cols; ++i) {

            auto c1i = m1.col(i);
            auto c2i = m2.col(i);

            auto diff = c1i - c2i;
            vec dist = diff.t() * diff;

            totalDist += dist(0);

        }

        return totalDist;

    }

    arma::mat SkillExporter::computeAllDistances(std::vector<arma::mat>& skillData, bool giveFeedback) {

        if(!skillData.size())
            throw KukaduException("(SkillExporter) no data passed");

        int skillDataSize = skillData.size();

        boost::progress_display show_progress(skillDataSize * skillDataSize);

        mat distMat(skillDataSize, skillDataSize);
        distMat.fill(0.0);
        for(int i = 0; i < skillDataSize; ++i) {
            for(int j = 0; j < skillDataSize; ++j) {
                if(giveFeedback)
                    cout << "comparing skills " << i << " and " << j << endl;
                double distance = computeDistance(skillData.at(i), skillData.at(j));
                distMat(j, i) = distance;
                ++show_progress;
            }
        }

        return distMat;

    }

    std::vector<std::string> SkillExporter::getSkillHardware(int skillId) {

        vector<string> hardwareList;

        auto& st = getStorage();

        stringstream s;
        s << "select hi.instance_name as iname from skills_robot as sr " <<
             " inner join hardware_instances as hi on hi.instance_id = sr.hardware_instance_id where skill_id = " << skillId <<
             " order by hi.instance_name";

        auto queryRes = st.executeQuery(s.str());
        while(queryRes->next())
            hardwareList.push_back(queryRes->getString("iname"));

        return hardwareList;

    }

    std::vector<std::tuple<long long int, long long int, bool> > SkillExporter::getSkillExecutions(std::string skillName, long long int startTime, long long int endTime) {

        auto& storage = getStorage();
        int skillId = storage.getCachedLabelId("skills", "skill_id", "label", skillName);

        return getSkillExecutions(skillId, startTime, endTime);

    }

    std::vector<std::tuple<long long int, long long int, bool> > SkillExporter::getSkillExecutions(int skillId, long long startTime, long long endTime) {

        if(endTime == 0)
            endTime = std::numeric_limits<long long int>::max();

        std::vector<std::tuple<long long int, long long int, bool> > executionsList;

        auto& st = getStorage();

        stringstream s;
        s << "select start_timestamp, end_timestamp, successful from skill_executions where skill_id = " << skillId <<
             " and start_timestamp >= " << startTime << " and (end_timestamp <= " << endTime <<
             " or (end_timestamp is null and start_timestamp <= " << endTime << "))" <<
             " order by start_timestamp";
        auto queryRes = st.executeQuery(s.str());
        while(queryRes->next()) {
            long long int startTime = queryRes->getInt64("start_timestamp");
            long long int endTime = queryRes->getInt64("end_timestamp");
            bool succ = queryRes->getInt("successful");
            executionsList.push_back(std::tuple<long long int, long long int, bool>{startTime, endTime, succ});
        }

        return executionsList;

    }

    void normalizeTime(vector<std::pair<long long int, arma::vec> >& times) {

        if(times.size()) {
            long long int startTime = times.front().first;
            for(auto& t : times)
                t.first -= startTime;
        }

    }

    long long int computeAverageTimeDist(vector<std::pair<long long int, arma::vec> >& times) {

        long long int avgDist = 0;
        for(int i = 1; i < times.size(); ++i)
            avgDist += times.at(i).first - times.at(i - 1).first;

        return avgDist / times.size();

    }

    std::pair<int, int> msToMinSec(long long int time) {

        double totalSeconds = (double) time / 1000.0;
        int totalMins = totalSeconds / 60.0;
        int restSeconds = (totalSeconds - totalMins * 60.0);

        return {totalMins, restSeconds};

    }

    void SkillExporter::exportSkillExecutions(int skillId, long long int startTime, long long int endTime, std::string folder, bool exportModuleStatistics, bool compress) {

        if(fileExists(folder))
            throw KukaduException("(SkillExporter) target directory already exists");
        createDirectory(folder);

        auto& sensorSingleton = SensorStorageSingleton::get();

        auto hardware = getSkillHardware(skillId);

        vector<int> sensorDataLengthPerHardware;
        vector<KUKADU_SHARED_PTR<Hardware> > hardwareInstances;
        for(auto& hardwareName : hardware) {
            sensorDataLengthPerHardware.push_back(-1);
            hardwareInstances.push_back(sensorSingleton.getRegisteredHardware(hardwareName));
        }

        long long int maxDuration = 0;
        long long int minDeltaT = std::numeric_limits<long long int>::max();
        map<int, vector<vector<std::pair<long long int, arma::vec> > > > allSamples;
        for(auto& hw : hardwareInstances)
            allSamples[hw->getHardwareInstance()] = vector<vector<std::pair<long long int, arma::vec> > >{};

        int exportedExecutionCount = 0;
        auto executions = getSkillExecutions(skillId, startTime, endTime);

        if(executions.size()) {

            // contains the start data before normalization (after normalization all start times are 0)
            vector<long long int> unnormalizedStartTimes(executions.size(), std::numeric_limits<long long int>::max());
            vector<long long int> unnormalizedEndTimes(executions.size(), std::numeric_limits<long long int>::min());

            for(int i = 0; i < executions.size(); ++i) {

                auto& execution = executions.at(i);

                long long int startTime = get<0>(execution);
                long long int endTime = get<1>(execution);
                bool succ = get<2>(execution);

                auto convTime = msToMinSec(endTime - startTime);
                int totalMins = convTime.first;
                int restSeconds = convTime.second;

                cout << "exporting execution number " << ++exportedExecutionCount;
                if(endTime > 0)
                    cout << " and duration " << totalMins << "m:" << restSeconds << "s";
                cout << " which was " << ((succ) ? "successful" : "not successful" ) << endl;

                int hardwareId = 0;
                // load all the data and estimate the grid time minDeltaT
                for(auto& hw : hardwareInstances) {

                    // load all the data
                    allSamples[hw->getHardwareInstance()].push_back(hw->loadData(startTime, endTime));
                    auto& executionData = allSamples[hw->getHardwareInstance()].back();

                    unnormalizedStartTimes.at(i) = startTime;
                    unnormalizedEndTimes.at(i) = endTime;

                    if(executionData.size()) {

                        // store the sensor data length
                        if(sensorDataLengthPerHardware.at(hardwareId) == -1)
                            sensorDataLengthPerHardware.at(hardwareId) = executionData.front().second.n_elem;

                        for(auto& dataLine : executionData)
                            if(sensorDataLengthPerHardware.at(hardwareId) != dataLine.second.n_elem)
                                throw KukaduException("(SkillExporter) sensor data varies in lenght over time - currently not supported with this implementation");

                        // shift the time to 0
                        normalizeTime(executionData);

                        // get the duration of the execution
                        long long int& executionEndTime = executionData.back().first;

                        // find the maximum duration of all samples
                        maxDuration = (maxDuration > executionEndTime) ? maxDuration : executionEndTime;

                        // get the average clock cycle for the hardware
                        auto avgTime = computeAverageTimeDist(executionData);

                        // get the maximum average clock cycle for all hardware components
                        minDeltaT = (avgTime < minDeltaT) ? avgTime : minDeltaT;

                        cout << "data for " << hw->getHardwareInstanceName() << " has " << executionData.size() << " samples with an average delta t = " << avgTime << "ms" << endl;

                    } else
                        cout << "(SkillExporter) no data in the requested time period from " << startTime << " to " << endTime << endl;

                    hardwareId++;

                }

            }

            // compute the sum of all dimension
            int totalDimensionality = 0;
            for(auto& senorDataLength : sensorDataLengthPerHardware)
                totalDimensionality += senorDataLength;

            auto convTime = msToMinSec(maxDuration);
            int maxMins = convTime.first;
            int maxRestSeconds = convTime.second;

            cout << "statistics on data:" << endl;
            cout << "\t Maximal duration: " << maxMins << "m:" << maxRestSeconds << "s" << endl;
            cout << "\t Minimal delta t: " << minDeltaT << "ms" << endl;
            cout << "\t Total sensor dimensionality: " << totalDimensionality << endl;

            if(*(folder.end()) != '/')
                folder += "/";

            ofstream labelFile;
            labelFile.open(string(folder + "labels").c_str());

            ofstream labelDetailFile;
            labelDetailFile.open(string(folder + "labels_detail").c_str());

            for(int i = 0; i < executions.size(); ++i) {

                auto& execution = executions.at(i);
                bool succ = get<2>(execution);
                long long int endTime = get<1>(execution);

                int successMode = -1;

                if(succ)
                    successMode = 0;
                else if(endTime != 0)
                    successMode = 1;
                else
                    successMode = 2;

                stringstream s;
                s << "skill_" << i;

                if(compress)
                    s << ".bz2";

                labelFile << s.str() << "\t" << succ << endl;
                labelDetailFile << s.str() << "\t" << successMode << endl;

                storeExecution(hardwareInstances, allSamples,
                               unnormalizedStartTimes, unnormalizedEndTimes,
                               i, maxDuration, minDeltaT, sensorDataLengthPerHardware, folder + s.str(),
                               exportModuleStatistics,
                               compress);

            }

        } else {
            cout << "(SkillExporter) no skills executed in the requested time span" << endl;
        }


    }

    int augmentWithFunctionStatistics(long long int startTime, long long int endTime, arma::mat& currentData, long long int deltaT, stringstream& ostr, int maxFunctionId) {

        auto& storage = StorageSingleton::get();

        stringstream functionStream;
        functionStream << "select id from software_functions";
        if(maxFunctionId >= 0)
            functionStream << " where id <= " << maxFunctionId;
        functionStream << " order by id";
        auto queryRes = storage.executeQuery(functionStream.str());
        map<int, int> mapFunctionIdToIdx;
        for(int i = 0; queryRes->next(); ++i) {
            int currentId = queryRes->getInt("id");
            mapFunctionIdToIdx[currentId] = i;
            if(currentId > maxFunctionId)
                maxFunctionId = currentId;
        }

        int functionCount = mapFunctionIdToIdx.size();

        stringstream s;
        s << "select function_id, start_timestamp, end_timestamp, cnt from" <<
                "(select function_id, start_timestamp, end_timestamp, 1 as cnt from software_statistics_mode0 where start_timestamp >= " << startTime << " and end_timestamp <= " << endTime <<
                " union " <<
                "select function_id, start_timestamp, end_timestamp, cnt from software_statistics_mode1 where start_timestamp >= " << startTime << " and end_timestamp <= " << endTime <<
                ") as union_statistics" <<
                " where function_id <= " << maxFunctionId <<
                " order by function_id asc, start_timestamp asc";

        arma::mat statisticsData(functionCount, currentData.n_cols);
        statisticsData.fill(0.0);

        auto statisticsQuery = storage.executeQuery(s.str());
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
                if(normalizedStartTime > normalizedEndTime && normalizedEndTime >= 0) {
                    cout << normalizedStartTime << " " << normalizedEndTime << endl;
                    throw KukaduException("(SkillExporter) database inconsistency --> start time of function call is bigger than end time");
                }

                // compute index where the data belongs to
                int currentStartIndex = normalizedStartTime / deltaT;

                // compute on how many cells the calls are distributed
                int distributionCount = max(1.0, ceil((double) delta / deltaT));
                double distributedCount = (double) callCount / distributionCount;

                // add the distributed count of the function call to the cell range (many calls can be there at the same time --> sum them up)
                for(int currentIndex = currentStartIndex, i = 0; i < distributionCount && currentIndex < statisticsData.n_cols; ++currentIndex)
                    statisticsData(currentFunctionIdx, currentIndex) += distributedCount;

            } else
                throw KukaduException("(SkillExporter) precondition violated - bug in code");

        }

        if(armadilloHasNan(statisticsData))
            throw KukaduException("(SkillExporter) post condition violated - check for bug in code");

        ostr << statisticsData;

        int maxUsedFunctionId = 0;
        for(auto& functionIdPair : mapFunctionIdToIdx)
            maxUsedFunctionId = max(functionIdPair.first, maxUsedFunctionId);

        return maxUsedFunctionId;

    }

    void storeExecution(vector<KUKADU_SHARED_PTR<Hardware> >& hardwareInstances,
                        map<int, vector<vector<std::pair<long long int, arma::vec> > > >& allSamples,
                        vector<long long int>& unnormalizedStartTimes,
                        vector<long long int>& unnormalizedEndTimes,
                        int& executionId,
                        long long int& maxDuration, long long int minDeltaT,
                        vector<int>& sensorDataLengthPerHardware,
                        std::string file,
                        bool exportModuleStatistics,
                        bool compress) {

        int totalDimensionality = 0;
        for(auto& senorDataLength : sensorDataLengthPerHardware)
            totalDimensionality += senorDataLength;

        // create the storage grid
        // the grid resolution is the maximum frequency; the length is the maximal duration of all executions
        int maxTimeIdx = ceil((double) maxDuration / minDeltaT);
        mat storageGrid(totalDimensionality, maxTimeIdx);
        storageGrid.fill(datum::nan);

        stringstream completeFileStream;

        ofstream outFile(file.c_str(), ios_base::out | ios_base::binary);

        filtering_streambuf<output> out;

        if(compress)
            out.push(bzip2_compressor());

        out.push(outFile);

        vector<int> maxFilledColId(totalDimensionality, 0);

        // filling the grid
        // run through all the data for each hardware and fill it to the corresponding spot in the matrix
        int dimOffset = 0;
        for(int i = 0; i < hardwareInstances.size(); ++i) {

            auto& dataPerHardware = allSamples[hardwareInstances.at(i)->getHardwareInstance()];

            // go through all executions
            auto& executionPerHardware = dataPerHardware.at(executionId);

            // go through all sensor samples for each execution
            for(int m = 0; m < executionPerHardware.size(); ++m) {

                auto& dataLine = executionPerHardware.at(m);

                auto& currentTime = dataLine.first;
                auto& currentSensorSample = dataLine.second;

                // store the sample to the current position in the storage grid
                int currentColIdx = (double) currentTime / minDeltaT;

                for(int j = 0; j < sensorDataLengthPerHardware.at(i); ++j) {

                    maxFilledColId.at(dimOffset + j) = currentColIdx;
                    storageGrid(dimOffset + j, currentColIdx) = currentSensorSample(j);

                    // from the current time index go back until the last sample (all values until there are NaN) and interpolate
                    int k = 0;
                    for(k = currentColIdx - 1; k >= 0; --k) {
                        // check if value is nan --> if not, stop the loop, we reached the interpolation place
                        if(storageGrid(dimOffset + j, k) == storageGrid(dimOffset + j, k))
                            break;
                    }

                    // if k is smaller than 0, it means that all values before were nan --> no interpolation possible, just fill it up with the current value
                    if(k < 0)
                        for(int l = 0; l < currentColIdx; ++l)
                            storageGrid(dimOffset + j, l) = currentSensorSample(j);
                    else if(k != currentColIdx) {
                        double deltaX = (double) (storageGrid(dimOffset + j, currentColIdx) - storageGrid(dimOffset + j, k)) / (currentColIdx - k);
                        for(int l = k; l < currentColIdx; ++l)
                            storageGrid(dimOffset + j, l) = storageGrid(dimOffset + j, k) + (l - k + 1) * deltaX;
                    }

                }

            }

            dimOffset += sensorDataLengthPerHardware.at(i);

        }

        // compute the actual duration of all samples (including the ones that got cut off)
        long long int executionDuration = 0;
        for(int i = 0; i < maxFilledColId.size(); ++i) {
            long long int dimLength = (maxFilledColId.at(i) + 1) * minDeltaT;
            executionDuration = (executionDuration < dimLength) ? dimLength : executionDuration;
        }

        // at this stage the begin of the grid should be free of nans --> now fill up the back part with dummy data
        for(int i = 0; i < maxFilledColId.size(); ++i) {
            auto& currentMaxColId = maxFilledColId.at(i);
            if(currentMaxColId == 0)
                cout << "(SkillExporter) warning: not all hardware components delivered data (row: " << i << " was empty)" << endl;
            for(int j = currentMaxColId + 1; j < storageGrid.n_cols; ++j)
                storageGrid(i, j) = storageGrid(i, currentMaxColId);
        }

        if(exportModuleStatistics) {

            cout << "generating function statistics for execution number " << executionId;
            if(compress)
                cout << " and compress";
            cout << endl;
            int maxUsedFunctionId = augmentWithFunctionStatistics(unnormalizedStartTimes.at(executionId),
                                          unnormalizedStartTimes.at(executionId) + executionDuration, storageGrid, minDeltaT,
                                          completeFileStream);
            ofstream maxUsedFunctionFile;
            maxUsedFunctionFile.open(string(file + "_max_used_function").c_str());
            maxUsedFunctionFile << "maxUsedFunctionId=" << maxUsedFunctionId << endl;

        }

        ofstream timingFile;
        timingFile.open(string(file + "_timing").c_str());
        timingFile << "startTimeStamp=" << unnormalizedStartTimes.at(executionId) << endl;
        timingFile << "endTimeStamp=" << unnormalizedEndTimes.at(executionId) << endl;
        timingFile << "timeStep=" << minDeltaT << endl;
        timingFile.close();

        // check if matrix contains nan
        for(int i = 0; i < storageGrid.n_rows; ++i)
            for(int j = 0; j < storageGrid.n_cols; ++j)
                if(storageGrid(i, j) != storageGrid(i, j))
                    throw KukaduException("(SkillExporter) post condition violated - check for bug in code");

        completeFileStream << storageGrid << endl;

        // compressing
        copy(completeFileStream, out);
        outFile.close();

    }

}
