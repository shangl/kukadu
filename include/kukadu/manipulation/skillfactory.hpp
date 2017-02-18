#ifndef KUKADU_SKILLFACTORY_H
#define KUKADU_SKILLFACTORY_H

#include <map>
#include <string>
#include <functional>
#include <kukadu/robot/queue.hpp>
#include <kukadu/types/kukadutypes.hpp>
#include <kukadu/control/controller.hpp>
#include <kukadu/storage/storagesingleton.hpp>

namespace kukadu {

    class SkillFactory {

    private:

        StorageSingleton& storage;

        static std::map<std::string, std::function<KUKADU_SHARED_PTR<Controller>(StorageSingleton&, int, int, KUKADU_SHARED_PTR<Hardware>)> > skillFactories;

        SkillFactory();

    public:

        static SkillFactory& get();

        KUKADU_SHARED_PTR<Controller> loadSkill(std::string skillName, KUKADU_SHARED_PTR<Hardware> queue);

        std::vector<std::string> listAvailableSkills();
        std::string getSkillController(std::string skillName);
        std::vector<std::string> getSupportedRobots(std::string skillName);

        bool skillExists(std::string skillName);

    };

}

#endif
