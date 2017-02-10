#include <limits>
#include <sstream>
#include <armadillo>
#include <kukadu/utils.hpp>
#include <kukadu/manipulation/skillexporter.hpp>
#include <kukadu/storage/sensorstoragesingleton.hpp>

using namespace std;
using namespace arma;

namespace kukadu {

    void storeExecution(vector<KUKADU_SHARED_PTR<Hardware> >& hardwareInstances,
                        map<int, vector<vector<std::pair<long long int, arma::vec> > > >& allSamples, int& executionId,
                        long long int& maxDuration, long long int& minDeltaT,
                        vector<int>& sensorDataLengthPerHardware,
                        std::string file);

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

    void SkillExporter::exportSkillExecutions(int skillId, long long int startTime, long long int endTime, std::string folder) {

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
        for(auto& execution : executions) {

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
            labelFile << s.str() << "\t" << succ << endl;
            labelDetailFile << s.str() << "\t" << successMode << endl;

            storeExecution(hardwareInstances, allSamples, i, maxDuration, minDeltaT, sensorDataLengthPerHardware, folder + s.str());

        }


    }

    void storeExecution(vector<KUKADU_SHARED_PTR<Hardware> >& hardwareInstances,
                        map<int, vector<vector<std::pair<long long int, arma::vec> > > >& allSamples, int& executionId,
                        long long int& maxDuration, long long int& minDeltaT,
                        vector<int>& sensorDataLengthPerHardware,
                        std::string file) {

        int totalDimensionality = 0;
        for(auto& senorDataLength : sensorDataLengthPerHardware)
            totalDimensionality += senorDataLength;

        // create the storage grid
        // the grid resolution is the maximum frequency; the length is the maximal duration of all executions
        int maxTimeIdx = ceil((double) maxDuration / minDeltaT);
        mat storageGrid(totalDimensionality, maxTimeIdx);
        storageGrid.fill(datum::nan);

        ofstream outFile;
        outFile.open(file.c_str());

        vector<int> maxFilledColId(totalDimensionality, 0);

        // filling the grid
        // run through all the data for each hardware and fill it to the corresponding spot in the matrix
        int dimOffset = 0;
        for(int i = 0; i < hardwareInstances.size(); ++i) {

            auto& dataPerHardware = allSamples[hardwareInstances.at(i)->getHardwareInstance()];

            // go through all executions
            auto& executionPerHardware = dataPerHardware.at(executionId);

            // go through all sensor samples for each execution
            for(auto& dataLine : executionPerHardware) {

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

        // at this stage the begin of the grid should be free of nans --> now fill up the back part with dummy data
        for(int i = 0; i < maxFilledColId.size(); ++i) {
            auto& currentMaxColId = maxFilledColId.at(i);
            if(currentMaxColId == 0)
                cout << "(SkillExporter) warning: not all hardware components delivered data (row: " << i << " was empty)" << endl;
            for(int j = currentMaxColId + 1; j < storageGrid.n_cols; ++j)
                storageGrid(i, j) = storageGrid(i, currentMaxColId);
        }

        outFile << storageGrid << endl;
        outFile.close();

    }

}
