#include <limits>
#include <sstream>
#include <kukadu/manipulation/skillexporter.hpp>
#include <kukadu/storage/sensorstoragesingleton.hpp>

using namespace std;

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

    void SkillExporter::exportSkillExecutions(int skillId, long long int startTime, long long int endTime, std::string folder) {

        auto& sensorSingleton = SensorStorageSingleton::get();

        auto hardware = getSkillHardware(skillId);

        vector<KUKADU_SHARED_PTR<Hardware> > hardwareInstances;
        for(auto& hardwareName : hardware)
            hardwareInstances.push_back(sensorSingleton.getRegisteredHardware(hardwareName));

        int exportedExecutionCount = 0;
        auto executions = getSkillExecutions(skillId, startTime, endTime);
        for(auto& execution : executions) {

            cout << "exporting execution number " << ++exportedExecutionCount << endl;

            long long int startTime = get<0>(execution);
            long long int endTime = get<1>(execution);
            bool succ = get<2>(execution);

            for(auto& hw : hardwareInstances) {
                auto executionData = hw->loadData(startTime, endTime);
                cout << "data for " << hw->getHardwareInstanceName() << " has " << executionData.size() << " samples" << endl;
            }

        }

    }

}
