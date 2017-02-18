#include <limits>
#include <armadillo>
#include <kukadu/manipulation/skillfactory.hpp>
#include <kukadu/manipulation/skillexporter.hpp>
#include <kukadu/manipulation/autonomoustester.hpp>

using namespace std;
using namespace arma;

namespace kukadu {

    AutonomousTester::AutonomousTester(std::string skill, std::vector<KUKADU_SHARED_PTR<kukadu::Hardware> > availableHardware, std::pair<std::vector<int>, std::vector<arma::mat> >& skillData) {

        auto& skillFactory = SkillFactory::get();

        try {
            if(availableHardware.size())
                storedSkills[skill] = skillFactory.loadSkill(skill, availableHardware.front());
            else
                cerr << "(AutonomousTester) no hardware passed" << endl;
        } catch(KukaduException& ex) {
            cerr << "(AutonomousTester) skill controller not registered" << endl;
        }

        skillsData[skill] = skillData;

    }

    void AutonomousTester::testSkill(std::string id) {

        if(storedSkills.find(id) != storedSkills.end()) {



        } else
            throw KukaduException("(AutonomousTester) skill data not available");

    }

    double AutonomousTester::computeBestDistance(std::string skill, int prevEndTimeStep, int desiredEndTimeStep, arma::vec& prevDistances, arma::mat& execution) {

        auto& skillData = skillsData[skill];

        double minDist = std::numeric_limits<double>::max();
        for(int i = prevEndTimeStep; i < execution.n_cols && i < desiredEndTimeStep; ++i) {
            vec currCol = execution.col(i);
            // compute all distances until the current time step
            for(int j = 0; j < skillData.first.size(); ++j) {
                // if that skill was successful
                if(!skillData.first.at(j)) {
                    auto diffVec = currCol - skillData.second.at(j).col(i);
                    vec distance = diffVec.t() * diffVec;
                    prevDistances(j) += distance(0);

                    if(i == (execution.n_cols - 1) || i == (desiredEndTimeStep - 1))
                        if(prevDistances(j) > 0.0 && prevDistances(j) < minDist)
                            minDist = prevDistances(j);

                }
            }
        }

        return minDist;

    }

    double AutonomousTester::computeBestDistance(std::string skill, arma::mat& execution) {

        auto& skillData = skillsData[skill];
        vec skillDistances(skillData.first.size());
        skillDistances.fill(0.0);

        double minDist = 0.0;
        for(int i = 0; i < execution.n_cols; ++i)
            minDist = computeBestDistance(skill, i, i + 1, skillDistances, execution);

        return minDist;

    }

    double AutonomousTester::computeFailureProb(std::string skill, arma::mat execution) {

        auto& skillData = skillsData[skill];
        vec skillDistances(skillData.first.size());
        skillDistances.fill(0.0);

        vector<double> distDevel;
        for(int i = 0; i < execution.n_cols; ++i) {
            cout << i << endl;
            distDevel.push_back(computeBestDistance(skill, i, i + 1, skillDistances, execution));
        }

        ofstream o;
        o.open("/tmp/blub.txt");

        for(int i = 0; i < distDevel.size(); ++i)
            o << distDevel.at(i) << endl;

        o.close();

    }

}
