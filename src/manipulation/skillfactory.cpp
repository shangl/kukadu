#include <time.h>
#include <memory>
#include <utility>
#include <sstream>
#include <iostream>
#include <QtCore/QFile>
#include <QtCore/QTextStream>
#include <kukadu/utils/utils.hpp>
#include <kukadu/control/dmp.hpp>
#include <kukadu/generated_skills.hpp>
#include <kukadu/manipulation/skillfactory.hpp>

using namespace std;
namespace kukadu {

    std::map<std::string, std::function<KUKADU_SHARED_PTR<Controller>(
            StorageSingleton &, int, int, std::vector<KUKADU_SHARED_PTR<Hardware> >)> > SkillFactory::skillFactories{
            {"Nothing",                  [](StorageSingleton &storage, int skillId, int controllerType,
                                            std::vector<KUKADU_SHARED_PTR<Hardware> > hardwareComponents) {
                return make_shared<Nothing>(storage);
            }},
            {"DMPExecutor",              [](StorageSingleton &storage, int skillId, int controllerType,
                                            std::vector<KUKADU_SHARED_PTR<Hardware> > hardwareComponents) {
                return make_shared<DMPExecutor>(storage, skillId,
                                                KUKADU_DYNAMIC_POINTER_CAST<ControlQueue>(hardwareComponents.front()));
            }},
            {"JointPtp",                 [](StorageSingleton &storage, int skillId, int controllerType,
                                            std::vector<KUKADU_SHARED_PTR<Hardware> > hardwareComponents) {
                return make_shared<JointPtp>(storage,
                                             KUKADU_DYNAMIC_POINTER_CAST<JointHardware>(hardwareComponents.front()));
            }},
            {"CartesianPtp",             [](StorageSingleton &storage, int skillId, int controllerType,
                                            std::vector<KUKADU_SHARED_PTR<Hardware> > hardwareComponents) {
                return make_shared<CartesianPtp>(storage,
                                                 KUKADU_DYNAMIC_POINTER_CAST<ControlQueue>(hardwareComponents.front()));
            }},
            {"LocalizeObject",           [](StorageSingleton &storage, int skillId, int controllerType,
                                            std::vector<KUKADU_SHARED_PTR<Hardware> > hardwareComponents) {
                return make_shared<LocalizeObject>(storage,
                                                   KUKADU_DYNAMIC_POINTER_CAST<Kinect>(hardwareComponents.front()));
            }},
            {"KinestheticTeaching",      [](StorageSingleton &storage, int skillId, int controllerType,
                                            std::vector<KUKADU_SHARED_PTR<Hardware> > hardwareComponents) {
                return make_shared<KinestheticTeaching>(storage, KUKADU_DYNAMIC_POINTER_CAST<ControlQueue>(
                        hardwareComponents.front()));
            }},
            {"MoveHome",                 [](StorageSingleton &storage, int skillId, int controllerType,
                                            std::vector<KUKADU_SHARED_PTR<Hardware> > hardwareComponents) {
                return make_shared<MoveHome>(storage, hardwareComponents);
            }},
            {
             "OpenHand",                 [](StorageSingleton &storage, int skillId, int controllerType,
                                            std::vector<KUKADU_SHARED_PTR<Hardware> > hardwareComponents) {
                return make_shared<OpenHand>(storage, hardwareComponents);
            }
            },
            {
             "CloseHand",                [](StorageSingleton &storage, int skillId, int controllerType,
                                            std::vector<KUKADU_SHARED_PTR<Hardware> > hardwareComponents) {
                return make_shared<CloseHand>(storage, hardwareComponents);
            }
            },
            {
             "ChangeStiffness",          [](StorageSingleton &storage, int skillId, int controllerType,
                                            std::vector<KUKADU_SHARED_PTR<Hardware> > hardwareComponents) {
                return make_shared<ChangeStiffness>(storage, hardwareComponents);
            }
            },
            {
             "PushHandPos",              [](StorageSingleton &storage, int skillId, int controllerType,
                                            std::vector<KUKADU_SHARED_PTR<Hardware> > hardwareComponents) {
                return make_shared<PushHandPos>(storage, hardwareComponents);
            }
            },
            {
             "BlockingPos",              [](StorageSingleton &storage, int skillId, int controllerType,
                                            std::vector<KUKADU_SHARED_PTR<Hardware> > hardwareComponents) {
                return make_shared<BlockingPos>(storage, hardwareComponents);
            }
            },
            {
             "RightHandBlocking",        [](StorageSingleton &storage, int skillId, int controllerType,
                                            std::vector<KUKADU_SHARED_PTR<Hardware> > hardwareComponents) {
                return make_shared<RightHandBlocking>(storage, hardwareComponents);
            }
            },
            {
             "ChangeModeToIMP",          [](StorageSingleton &storage, int skillId, int controllerType,
                                            std::vector<KUKADU_SHARED_PTR<Hardware> > hardwareComponents) {
                return make_shared<ChangeModeToIMP>(storage, hardwareComponents);
            }
            },
            {
             "WaitForReached",           [](StorageSingleton &storage, int skillId, int controllerType,
                                            std::vector<KUKADU_SHARED_PTR<Hardware> > hardwareComponents) {
                return make_shared<WaitForReached>(storage, hardwareComponents);
            }
            },
            {
             "PushForward",              [](StorageSingleton &storage, int skillId, int controllerType,
                                            std::vector<KUKADU_SHARED_PTR<Hardware> > hardwareComponents) {
                return make_shared<PushForward>(storage, hardwareComponents);
            }
            },
            {
             "FinalPush",                [](StorageSingleton &storage, int skillId, int controllerType,
                                            std::vector<KUKADU_SHARED_PTR<Hardware> > hardwareComponents) {
                return make_shared<FinalPush>(storage, hardwareComponents);
            }
            },
            {
             "SimpleJointPtp",           [](StorageSingleton &storage, int skillId, int controllerType,
                                            std::vector<KUKADU_SHARED_PTR<Hardware> > hardwareComponents) {
                return make_shared<SimpleJointPtp>(storage, hardwareComponents);
            }
            },
            {
             "ShelfAlignment",           [](StorageSingleton &storage, int skillId, int controllerType,
                                            std::vector<KUKADU_SHARED_PTR<Hardware> > hardwareComponents) {
                return make_shared<ShelfAlignment>(storage, hardwareComponents);
            }
            },
            {
             "ShelfPlacementController", [](StorageSingleton &storage, int skillId, int controllerType,
                                            std::vector<KUKADU_SHARED_PTR<Hardware> > hardwareComponents) {
                return make_shared<ShelfPlacementController>(storage, hardwareComponents);
            }
            },
            {
             "DropInBoxController",      [](StorageSingleton &storage, int skillId, int controllerType,
                                            std::vector<KUKADU_SHARED_PTR<Hardware> > hardwareComponents) {
                return make_shared<DropInBoxController>(storage, hardwareComponents);
            }
            },
            {
             "PressButtonController",    [](StorageSingleton &storage, int skillId, int controllerType,
                                            std::vector<KUKADU_SHARED_PTR<Hardware> > hardwareComponents) {
                return make_shared<PressButtonController>(storage, hardwareComponents);
            }
            },
            {
             "SensingSlide",             [](StorageSingleton &storage, int skillId, int controllerType,
                                            std::vector<KUKADU_SHARED_PTR<Hardware> > hardwareComponents) {
                return make_shared<SensingSlide>(storage, hardwareComponents);
            }
            },
            {
             "SensingPoke",              [](StorageSingleton &storage, int skillId, int controllerType,
                                            std::vector<KUKADU_SHARED_PTR<Hardware> > hardwareComponents) {
                return make_shared<SensingPoke>(storage, hardwareComponents);
            }
            },
            {
             "PushTranslation",          [](StorageSingleton &storage, int skillId, int controllerType,
                                            std::vector<KUKADU_SHARED_PTR<Hardware> > hardwareComponents) {
                return make_shared<PushTranslation>(storage, hardwareComponents);
            }
            },
            {
             "BookGrasping",             [](StorageSingleton &storage, int skillId, int controllerType,
                                            std::vector<KUKADU_SHARED_PTR<Hardware> > hardwareComponents) {
                return make_shared<BookGrasping>(storage, hardwareComponents);
            }
            }


            //insertSkill
            //at this line further skills will be inserted automatically - do not remove it
    };

    SkillFactory::SkillFactory() : storage(StorageSingleton::get()) {}

    SkillFactory &SkillFactory::get() {
        static SkillFactory instance;
        return instance;
    }

    void SkillFactory::addSkill(std::string skillName) {

        std::string skillText = ",{\n\t\t\t\"" + skillName + "\"" +
                                ", [](StorageSingleton& storage, int skillId, int controllerType, std::vector<KUKADU_SHARED_PTR<Hardware> > hardwareComponents) {\n" +
                                "\t\t\t\treturn make_shared<" + skillName +
                                ">(storage, hardwareComponents);\n" + "\t\t\t}\n" +
                                "\t\t}\n";
        std::string skillFactoryFilePath = resolvePath("$KUKADU_HOME/src/manipulation/skillfactory.cpp");
        QFile skillFactoryFile(QString::fromStdString(skillFactoryFilePath));
        QFile newSkillFactoryFile(QString::fromStdString(skillFactoryFilePath + "1"));

        skillFactoryFile.open(QIODevice::ReadOnly);
        newSkillFactoryFile.open(QIODevice::WriteOnly);

        QTextStream skillFactoryStream(&skillFactoryFile);
        QTextStream newSkillFactoryStream(&newSkillFactoryFile);

        QString skillFactoryContent = "";
        std::string containedString = "/";
        containedString += "/insertSkill"; //keep this here like this to not have the text replaced here

        while (!skillFactoryStream.atEnd()) {
            QString line = skillFactoryStream.readLine();

            if (line.contains(QString::fromStdString(containedString))) {
                newSkillFactoryStream << QString::fromStdString(skillText) << "\n";
            }
            newSkillFactoryStream << line + "\n";
        }

        skillFactoryFile.close();
        newSkillFactoryFile.close();
        skillFactoryFile.remove();
        newSkillFactoryFile.rename(QString::fromStdString(skillFactoryFilePath));
    }

    void SkillFactory::removeSkill(std::string skillName) {
        std::string skillFactoryFilePath = resolvePath("$KUKADU_HOME/src/manipulation/skillfactory.cpp");
        QFile skillFactoryFile(QString::fromStdString(skillFactoryFilePath));
        QFile newSkillFactoryFile(QString::fromStdString(skillFactoryFilePath + "1"));

        skillFactoryFile.open(QIODevice::ReadOnly);
        newSkillFactoryFile.open(QIODevice::WriteOnly);

        QTextStream skillFactoryStream(&skillFactoryFile);
        QTextStream newSkillFactoryStream(&newSkillFactoryFile);

        bool skilldeleted = false;

        while (!skillFactoryStream.atEnd()) {
            QString line = skillFactoryStream.readLine();

            if (!skilldeleted) {
                if (line.contains("{")) {
                    std::string textToCheckStd = "{\"" + skillName + "\",";
                    QString textToCheck = QString::fromStdString(textToCheckStd);

                    if (line.contains(textToCheck)) {
                        line = skillFactoryStream.readLine();
                        line = skillFactoryStream.readLine();
                        line = skillFactoryStream.readLine();
                        line = skillFactoryStream.readLine();
                        skilldeleted = true;
                    } else {
                        QString nextLine = skillFactoryStream.readLine();
                        textToCheckStd = "\"" + skillName +
                                         "\",                 []";
                        textToCheck = QString::fromStdString(textToCheckStd);
                        std::string textToCheckStdB = "\"" + skillName + "\", []";
                        QString textToCheckB = QString::fromStdString(textToCheckStdB);
                        if (nextLine.contains(textToCheck) || nextLine.contains(textToCheckB)) {
                            line = skillFactoryStream.readLine();
                            line = skillFactoryStream.readLine();
                            line = skillFactoryStream.readLine();
                            line = skillFactoryStream.readLine();
                            if(nextLine.contains(textToCheck)) {
                                line = skillFactoryStream.readLine();
                            }
                            skilldeleted = true;
                        } else {
                            newSkillFactoryStream << line + "\n";
                            line = nextLine;
                        }
                    }
                }
            }
            newSkillFactoryStream << line + "\n";
        }

        skillFactoryFile.close();
        newSkillFactoryFile.close();
        skillFactoryFile.remove();
        newSkillFactoryFile.rename(QString::fromStdString(skillFactoryFilePath));
    }

    std::vector<KUKADU_SHARED_PTR<Hardware> > filterComponentsByConfig(int configId, std::vector<KUKADU_SHARED_PTR<Hardware> > hardwareComponents) {

        std::vector<KUKADU_SHARED_PTR<Hardware> > retConfig;

        stringstream s;
        s << "select hwi.instance_name as hwname FROM robot_config as rc " <<
             " inner join hardware_instances as hwi on rc.hardware_instance_id = hwi.instance_id " <<
             " WHERE rc.robot_config_id=" << configId << " ORDER BY rc.order_id ASC";

        auto res = StorageSingleton::get().executeQuery(s.str());
        while(res->next()) {
            string hardwareString = res->getString("hwname");

            KUKADU_SHARED_PTR<Hardware> nextHw;
            // not very efficient
            for(auto& hw : hardwareComponents) {
                if(hw->getHardwareInstanceName() == hardwareString) {
                    nextHw = hw;
                    break;
                }
            }
            if(nextHw)
                retConfig.push_back(nextHw);
            else
                throw KukaduException("(SkillFactory) hardware was not found");

        }

        return retConfig;

    }

    KUKADU_SHARED_PTR<Controller> SkillFactory::loadSkill(std::string skillName,
                                                          std::vector<KUKADU_SHARED_PTR<Hardware> > hardwareComponents) {

        if (!hardwareComponents.size())
            hardwareComponents.push_back(HardwareFactory::get().loadHardware("no_hardware_instance"));

        stringstream s;
        s
                << "SELECT DISTINCT skills_robot.robot_config_id as 'roboConfigId' FROM skills_robot INNER JOIN skills ON skills.skill_id=skills_robot.skill_id WHERE skills.label='"
                << skillName << "'";
        auto roboConfigResults = storage.executeQuery(s.str());
        int roboConfigIdInOrder = -1;
        int roboConfigIdSet = -1;
        int roboConfigIdSubset = -1;        //find ids of configurations which match given hardwarecomponents. first match is taken and not overwritten, finishes when InOrderId was found

        while (roboConfigResults->next() && roboConfigIdInOrder == -1) {
            int roboConfigId = roboConfigResults->getInt("roboConfigId");
            RobotConfiguration roboConfig(storage, roboConfigId);
            if (roboConfig.containsHardwareInOrder(hardwareComponents)) { roboConfigIdInOrder = roboConfigId; }
            else if (roboConfigIdSet == -1 &&
                     roboConfig.containsHardwareAsSet(hardwareComponents)) { roboConfigIdSet = roboConfigId; }
            else if (roboConfigIdSubset == -1 &&
                     roboConfig.containsHardware(hardwareComponents)) { roboConfigIdSubset = roboConfigId; }
        }
        int configId = roboConfigIdInOrder != -1 ? roboConfigIdInOrder : roboConfigIdSet != -1 ? roboConfigIdSet
                                                                                               : roboConfigIdSubset;

        if(configId != -1) {

            hardwareComponents = filterComponentsByConfig(configId, hardwareComponents);

            s.str(std::string());
            s
                    << "SELECT skills.skill_id as 'skillId', skills.controller_type as 'controllerId', skills_robot.robot_config_id as 'roboConfigId'"
                    << "FROM skills_robot INNER JOIN skills ON skills.skill_id=skills_robot.skill_id "
                    << "WHERE skills.label='" << skillName << "' AND skills_robot.robot_config_id=" << configId;
            auto skillResult = storage.executeQuery(s.str());
            if (skillResult->next()) {
                long long int skillId = skillResult->getInt64("skillId");
                int controllerType = skillResult->getInt("controllerId");
                auto controllerClassLabel = storage.getCachedLabel("controller_types", "controller_id",
                                                                   "controller_implementation_class", controllerType);
                if (skillFactories.find(controllerClassLabel) != skillFactories.end())
                    return skillFactories[controllerClassLabel](storage, skillId, controllerType, hardwareComponents);
                else {
                    cerr << "(SkillFactory) automatic loading of the skill " << skillName
                         << " is not supported for the required controller " <<
                         controllerClassLabel << endl;
                    throw KukaduException("(SkillFactory) automatic loading is not supported for the required controller");
                }
            }
        } else {
            stringstream s;
            s << "(SkillFactory) requested skill \"" << skillName
              << "\" does not exist in the data base or is not available for your robot";
            cerr << s.str() << endl;
            throw KukaduException(s.str().c_str());
        }
    }

    std::vector<std::string> SkillFactory::listAvailableSkills() {
        vector<string> skills;
        auto skillsRes = storage.executeQuery("select skill_id, label from skills");
        while (skillsRes->next()) skills.push_back(skillsRes->getString("label"));
        return skills;
    }

    bool SkillFactory::skillExists(std::string skillName) {
        auto skillsList = listAvailableSkills();
        if (std::find(skillsList.begin(), skillsList.end(), skillName) != skillsList.end()) return true;
        return false;
    }

    std::string SkillFactory::getSkillController(std::string skillName) {
        auto controllerRes = storage.executeQuery(
                "select controller_implementation_class from skills inner join controller_types on skills.controller_type = controller_types.controller_id where skills.label = '" +
                skillName + "'");
        if (controllerRes->next())
            return controllerRes->getString("controller_implementation_class").asStdString();
        else throw KukaduException("(SkillFactory) no such skill in database or controller is not known");
    }

    std::vector<std::string> SkillFactory::getSupportedRobots(std::string skillName) {
        stringstream s;
        s << "select skr.instance_name as rname from skills as ski"
          << " inner join skills_robot skr on skr.skill_id = ski.skill_id"
          << " inner join hardware_instances as hwi on hwi.instance_id = skr.hardware_instance_id "             " where label = '"
          << skillName << "'";
        vector<string> supportedRobots;
        auto robotRes = storage.executeQuery(s.str());
        bool nothingFound = true;
        while (robotRes->next()) {
            if (nothingFound) nothingFound = false;
            supportedRobots.push_back(robotRes->getString("rname"));
        }
        if (nothingFound)
            throw KukaduException("(SkillFactory) no such skill in database or robot is not known anymore");
        return supportedRobots;
    }

    KUKADU_SHARED_PTR<kukadu_mersenne_twister> SkillFactory::getGenerator() {
        if (generator)
            return generator;
        return generator = make_shared<kukadu_mersenne_twister>(time(NULL));
    }

    std::vector<string> SkillFactory::loadPlayableSkills() {

        vector<string> retVec;

        auto res = StorageSingleton::get().executeQuery(
                "select label from skills inner join controller_types on skills.controller_type = controller_types.controller_id where is_playable = 1");
        while (res->next()) {
            retVec.push_back(res->getString("label"));
        }

        return retVec;

    }

}
