#include <kukadu/manipulation/skillexporter.hpp>
#include <kukadu/manipulation/autonomoustester.hpp>

namespace kukadu {

    AutonomousTester::AutonomousTester(KUKADU_SHARED_PTR<kukadu::Controller> skill, std::pair<std::vector<int>, std::vector<arma::mat> >& skillData) {

        storedSkills[skill->getCaption()] = skill;
        skillsData[skill->getCaption()] = skillData;

    }

    void AutonomousTester::testSkill(std::string id) {

        if(storedSkills.find(id) != storedSkills.end()) {



        } else
            throw KukaduException("(AutonomousTester) skill data not available");

    }

}
