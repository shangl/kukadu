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

        double computeBestDistance(std::string skill, arma::mat& execution);
        double computeBestDistance(std::string skill, int prevEndTimeStep, int desiredEndTimeStep, arma::vec& prevDistances, arma::mat& execution);

    public:

        AutonomousTester(std::string skill, std::vector<KUKADU_SHARED_PTR<kukadu::Hardware> > availableHardware, std::pair<std::vector<int>, std::vector<arma::mat> >& skillData);

        void testSkill(std::string id);

        virtual double computeFailureProb(std::string skill, arma::mat execution);

    };

}

#endif
