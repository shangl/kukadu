#include <memory>
#include <utility>
#include <sstream>
#include <iostream>
#include <kukadu/manipulation/skillfactory.hpp>

// include all the controllers from which skills can be generated
#include <kukadu/control/dmp.hpp>

using namespace std;

namespace kukadu {

    std::map<std::string, std::function<KUKADU_SHARED_PTR<Controller>(StorageSingleton&, int, int, std::vector<KUKADU_SHARED_PTR<Hardware> >)> > SkillFactory::skillFactories{
        {
            "DMPExecutor", [](StorageSingleton& storage, int skillId, int controllerType, std::vector<KUKADU_SHARED_PTR<Hardware> > hardwareComponents) {
                return make_shared<DMPExecutor>(storage, skillId, KUKADU_DYNAMIC_POINTER_CAST<ControlQueue>(hardwareComponents.front()));
            }
        },
        {
            "JointPtp", [](StorageSingleton& storage, int skillId, int controllerType, std::vector<KUKADU_SHARED_PTR<Hardware> > hardwareComponents) {
                return make_shared<JointPtp>(storage, KUKADU_DYNAMIC_POINTER_CAST<ControlQueue>(hardwareComponents.front()));
            }
        },
        {
            "CartesianPtp", [](StorageSingleton& storage, int skillId, int controllerType, std::vector<KUKADU_SHARED_PTR<Hardware> > hardwareComponents) {
                return make_shared<CartesianPtp>(storage, KUKADU_DYNAMIC_POINTER_CAST<ControlQueue>(hardwareComponents.front()));
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
        s << "SELECT DISTINCT skills_robot.robot_config_id as 'roboConfigId' FROM skills_robot INNER JOIN skills ON skills.skill_id=skills_robot.skill_id WHERE skills.label='" << skillName << "'";

        auto roboConfigResults = storage.executeQuery(s.str());

        int roboConfigIdInOrder = -1;
        int roboConfigIdSet = -1;
        int roboConfigIdSubset = -1;

        //find ids of configurations which match given hardwarecomponents. first match is taken and not overwritten, finishes when InOrderId was found
        while(roboConfigResults->next() && roboConfigIdInOrder == -1){
            int roboConfigId = roboConfigResults->getInt("roboConfigId");
            RobotConfiguration roboConfig(storage, roboConfigId);

            if(roboConfig.containsHardwareInOrder(hardwareComponents)){
                roboConfigIdInOrder = roboConfigId;
            } else if (roboConfigIdSet == -1 && roboConfig.containsHardwareAsSet(hardwareComponents)){
                roboConfigIdSet = roboConfigId;
            } else if (roboConfigIdSubset == -1 && roboConfig.containsHardware(hardwareComponents)){
                roboConfigIdSubset = roboConfigId;
            }
        }

        int configId = roboConfigIdInOrder != -1 ? roboConfigIdInOrder : roboConfigIdSet != -1 ? roboConfigIdSet : roboConfigIdSubset;

            s.str(std::string());
            s << "SELECT skills.skill_id as 'skillId', skills.controller_type as 'controllerId', skills_robot.robot_config_id as 'roboConfigId'" <<
                 "FROM skills_robot INNER JOIN skills ON skills.skill_id=skills_robot.skill_id " <<
                 "WHERE skills.label='"<< skillName <<"' AND skills_robot.robot_config_id=" << configId;

            auto skillResult = storage.executeQuery(s.str());

            if(skillResult->next()) {

                long long int skillId = skillResult->getInt64("skillId");
                int controllerType = skillResult->getInt("controllerId");

                auto controllerClassLabel = storage.getCachedLabel("controller_types", "controller_id", "controller_implementation_class", controllerType);
                if(skillFactories.find(controllerClassLabel) != skillFactories.end())
                    return skillFactories[controllerClassLabel](storage, skillId, controllerType, hardwareComponents);
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
