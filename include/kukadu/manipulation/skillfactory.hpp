#ifndef KUKADU_SKILLFACTORY_H
#define KUKADU_SKILLFACTORY_H

#include <map>
#include <string>
#include <functional>
#include <kukadu/robot/queue.hpp>
#include <kukadu/vision/kinect.hpp>
#include <kukadu/types/kukadutypes.hpp>
#include <kukadu/control/controller.hpp>
#include <kukadu/storage/storagesingleton.hpp>

namespace kukadu {

    class SkillFactory {

    private:

        KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator;

        StorageSingleton& storage;

        static std::map<std::string, std::function<KUKADU_SHARED_PTR<Controller>(StorageSingleton&, int, int, std::vector<KUKADU_SHARED_PTR<Hardware> >)> > skillFactories;

        SkillFactory();

    public:

        static SkillFactory& get();
        static void addSkill(std::string skillName);
        static void removeSkill(std::string skillName);

        KUKADU_SHARED_PTR<Controller> loadSkill(std::string skillName, std::vector<KUKADU_SHARED_PTR<Hardware> > hardwareComponents);

        std::vector<std::string> listAvailableSkills();
        std::string getSkillController(std::string skillName);
        std::vector<std::string> getSupportedRobots(std::string skillName);

        bool skillExists(std::string skillName);

        KUKADU_SHARED_PTR<kukadu_mersenne_twister> getGenerator();

        std::vector<std::string> loadPlayableSkills();


    };

}

#endif
