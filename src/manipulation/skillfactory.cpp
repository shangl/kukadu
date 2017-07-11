#include <memory>
#include <utility>
#include <sstream>
#include <iostream>
#include <kukadu/manipulation/skillfactory.hpp>

// include all the controllers from which skills can be generated
#include <kukadu/control/dmp.hpp>

using namespace std;

namespace kukadu {

    std::map<std::string, std::function<KUKADU_SHARED_PTR<Controller>(StorageSingleton&, int, int, KUKADU_SHARED_PTR<Hardware>)> > SkillFactory::skillFactories{
        {
            "DMPExecutor", [](StorageSingleton& storage, int skillId, int controllerType, KUKADU_SHARED_PTR<Hardware> queue) {
                return make_shared<DMPExecutor>(storage, skillId, KUKADU_DYNAMIC_POINTER_CAST<ControlQueue>(queue));
            }
        },
        {
            "JointPtp", [](StorageSingleton& storage, int skillId, int controllerType, KUKADU_SHARED_PTR<Hardware> queue) {
                return make_shared<JointPtp>(storage, KUKADU_DYNAMIC_POINTER_CAST<ControlQueue>(queue));
            }
        },
        {
            "CartesianPtp", [](StorageSingleton& storage, int skillId, int controllerType, KUKADU_SHARED_PTR<Hardware> queue) {
                return make_shared<CartesianPtp>(storage, KUKADU_DYNAMIC_POINTER_CAST<ControlQueue>(queue));
            }
        }
    };

    SkillFactory::SkillFactory() : storage(StorageSingleton::get()) {

    }

    SkillFactory& SkillFactory::get() {
        static SkillFactory instance;
        return instance;
    }

    KUKADU_SHARED_PTR<Controller> SkillFactory::loadSkill(std::string skillName, std::vector<KUKADU_SHARED_PTR<Hardware> > hardwareComponents) {

        stringstream s;
        s << "select skr.skill_id as skid, controller_type from skills as ski" <<
             " inner join skills_robot skr on skr.skill_id = ski.skill_id" <<
             " where label = '" << skillName << "' and skr.hardware_instance_id = " << queue->getHardwareInstance();
        auto skillResult = storage.executeQuery(s.str());

        if(skillResult->next()) {

            long long int skillId = skillResult->getInt64("skid");
            int controllerType = skillResult->getInt("controller_type");

            auto controllerClassLabel = storage.getCachedLabel("controller_types", "controller_id", "controller_implementation_class", controllerType);
            if(skillFactories.find(controllerClassLabel) != skillFactories.end())
                return skillFactories[controllerClassLabel](storage, skillId, controllerType, queue);
            else
                throw KukaduException("(SkillFactory) automatic loading is not supported for the required controller");

        } else
            throw KukaduException("(SkillFactory) requested skill name does not exist in the data base or is not available for your robot");
        return nullptr;
    }

    std::vector<std::string> SkillFactory::listAvailableSkills() {
        vector<string> skills;
        auto skillsRes = storage.executeQuery("select skill_id, label from skills");
        while(skillsRes->next())
            skills.push_back(skillsRes->getString("label"));
        return skills;
    }

    bool SkillFactory::skillExists(std::string skillName) {
        auto skillsList = listAvailableSkills();
        if(std::find(skillsList.begin(), skillsList.end(), skillName) != skillsList.end())
            return true;
        return false;
    }

    std::string SkillFactory::getSkillController(std::string skillName) {
        auto controllerRes = storage.executeQuery("select controller_implementation_class from skills inner join controller_types on skills.controller_type = controller_types.controller_id where skills.label = '" + skillName + "'");
        if(controllerRes->next())
            return controllerRes->getString("controller_implementation_class").asStdString();
        else
            throw KukaduException("(SkillFactory) no such skill in database or controller is not known");
    }

    std::vector<std::string> SkillFactory::getSupportedRobots(std::string skillName) {

        stringstream s;
        s << "select skr.instance_name as rname from skills as ski" <<
             " inner join skills_robot skr on skr.skill_id = ski.skill_id" <<
             " inner join hardware_instances as hwi on hwi.instance_id = skr.hardware_instance_id "
             " where label = '" << skillName << "'";

        vector<string> supportedRobots;
        auto robotRes = storage.executeQuery(s.str());
        bool nothingFound = true;
        while(robotRes->next()) {
            if(nothingFound)
                nothingFound = false;
            supportedRobots.push_back(robotRes->getString("rname"));
        }
        if(nothingFound)
            throw KukaduException("(SkillFactory) no such skill in database or robot is not known anymore");
        return supportedRobots;
    }

}
