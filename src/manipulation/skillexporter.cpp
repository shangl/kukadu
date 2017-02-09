#include <limits>
#include <sstream>
#include <armadillo>
#include <kukadu/manipulation/skillexporter.hpp>
#include <kukadu/storage/sensorstoragesingleton.hpp>

using namespace std;
using namespace arma;

namespace kukadu {

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

    void SkillExporter::exportSkillExecutions(int skillId, long long int startTime, long long int endTime, std::string folder) {

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
        vector<vector<std::pair<long long int, arma::vec> > > hardwareData;

        int exportedExecutionCount = 0;
        auto executions = getSkillExecutions(skillId, startTime, endTime);
        for(auto& execution : executions) {

            long long int startTime = get<0>(execution);
            long long int endTime = get<1>(execution);
            bool succ = get<2>(execution);

            double totalSeconds = (double) (endTime - startTime) / 1000.0;
            int totalMins = totalSeconds / 60.0;
            int restSeconds = (totalSeconds - totalMins * 60.0);

            cout << "exporting execution number " << ++exportedExecutionCount;
            if(endTime > 0)
                cout << " and duration " << totalMins << "m:" << restSeconds << "s";
            cout << " which was " << ((succ) ? "successful" : "not successful" ) << endl;

            int hardwareId = 0;
            // load all the data and estimate the grid time minDeltaT
            for(auto& hw : hardwareInstances) {

                // load all the data
                hardwareData.push_back(hw->loadData(startTime, endTime));
                auto& executionData = hardwareData.back();

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

            int totalDimensionality = 0;
            for(auto& senorDataLength : sensorDataLengthPerHardware)
                totalDimensionality += senorDataLength;

            cout << "statistcis on data:" << endl;
            cout << "\t Maximal duration: " << maxDuration << "ms" << endl;
            cout << "\t Minimal delta t: " << minDeltaT << "ms" << endl;
            cout << "\t Total sensor dimensionality: " << totalDimensionality << endl;

            // create the storage grid
            // the grid resolution is the maximum frequency; the length is the maximal duration of all executions
            int maxTimeIdx = ceil((double) maxDuration / minDeltaT);
            mat storageGrid(totalDimensionality, maxTimeIdx);

            /*
            // fill the storage grid
            for(auto& hwData : hardwareData) {

            }
            */

        }

    }

}
