#ifndef KUKADU_AUTONOMOUS_TEST_H
#define KUKADU_AUTONOMOUS_TEST_H

#include <map>
#include <vector>
#include <utility>
#include <armadillo>
#include <kukadu/types/kukadutypes.hpp>
#include <kukadu/control/controller.hpp>

namespace kukadu {

    class AutonomousTester {

        std::map<std::string, KUKADU_SHARED_PTR<kukadu::Controller> > storedSkills;
        std::map<std::string, std::pair<std::vector<int>, std::vector<arma::mat> > > skillsData;

    public:

        AutonomousTester(KUKADU_SHARED_PTR<kukadu::Controller> skill, std::pair<std::vector<int>, std::vector<arma::mat> >& skillData);

        void testSkill(std::string id);

    };

}

#endif
