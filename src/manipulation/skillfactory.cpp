#include <memory>
#include <utility>
#include <sstream>
#include <iostream>
#include <kukadu/manipulation/skillfactory.hpp>

// include all the controllers from which skills can be generated
#include <kukadu/control/dmp.hpp>

using namespace std;

namespace kukadu {

    std::map<std::string, std::function<KUKADU_SHARED_PTR<Controller>(StorageSingleton&, int, int, KUKADU_SHARED_PTR<ControlQueue>)> > SkillFactory::skillFactories{
        {
            "DMPExecutor", [](StorageSingleton& storage, int skillId, int controllerType, KUKADU_SHARED_PTR<ControlQueue> queue) {
                return make_shared<DMPExecutor>(storage, skillId, queue);
            }
        }
    };

    SkillFactory::SkillFactory() : storage(StorageSingleton::get()) {

    }

    SkillFactory& SkillFactory::get() {
        static SkillFactory instance;
        return instance;
    }

    KUKADU_SHARED_PTR<Controller> SkillFactory::loadSkill(std::string skillName, KUKADU_SHARED_PTR<ControlQueue> queue) {

        stringstream s;
        s << "select skill_id, controller_type from skills where label = '" << skillName << "' and robot_id = " << queue->getRobotId();
        auto skillResult = storage.executeQuery(s.str());

        if(skillResult->next()) {

            long long int skillId = skillResult->getInt64("skill_id");
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

    std::string SkillFactory::getSkillController(std::string skillName) {
        auto controllerRes = storage.executeQuery("select controller_implementation_class from skills inner join controller_types on skills.controller_type = controller_types.controller_id where skills.label = '" + skillName + "'");
        if(controllerRes->next())
            return controllerRes->getString("controller_implementation_class").asStdString();
        else
            throw KukaduException("(SkillFactory) no such skill in database or controller is not known");
    }

    std::vector<std::string> SkillFactory::getSupportedRobots(std::string skillName) {
        vector<string> supportedRobots;
        auto robotRes = storage.executeQuery("select robot_name from skills inner join robot on skills.robot_id = robot.robot_id where skills.label = '" + skillName + "'");
        bool nothingFound = true;
        while(robotRes->next()) {
            if(nothingFound)
                nothingFound = false;
            supportedRobots.push_back(robotRes->getString("robot_name"));
        }
        if(nothingFound)
            throw KukaduException("(SkillFactory) no such skill in database or robot is not known anymore");
        return supportedRobots;
    }

}
