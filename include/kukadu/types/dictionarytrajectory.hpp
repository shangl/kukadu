#ifndef KUKADU_DICTIONARYTRAJECTORY_H
#define KUKADU_DICTIONARYTRAJECTORY_H

#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <armadillo>
#include <kukadu/utils/utils.hpp>
#include <kukadu/utils/types.hpp>
#include <kukadu/control/dmp.hpp>
#include <kukadu/types/querypoint.hpp>
#include <kukadu/types/trajectory.hpp>
#include <kukadu/utils/conversion_utils.hpp>

namespace kukadu {

    class DictionaryTrajectory : public Trajectory {

    private:

        int degOfFreedom;
        std::string baseFolder;

        std::vector<std::string> files;
        std::vector<std::string> queryFiles;
        std::vector<std::string> dmpFiles;
        std::vector<std::string> trajFiles;
        std::vector<QueryPoint> queryPoints;
        std::vector<arma::vec> coefficients;

        arma::vec startingPos;

        std::vector<QueryPoint> mapFiles(std::vector<std::string> queryFiles, std::vector<std::string> trajFiles, std::string prefix1, std::string prefix2);
        std::vector<QueryPoint> mapFiles(std::vector<std::string> queryFiles, std::vector<std::string> trajFiles, std::vector<std::string> dmpFiles, std::string prefix1, std::string prefix2, std::string prefix3);

    public:

        DictionaryTrajectory();
        DictionaryTrajectory(const DictionaryTrajectory& copy);
        DictionaryTrajectory(std::string baseFolder, double az, double bz);

        void setTmax(double tmax);
        void setCoefficients(std::vector<arma::vec> coeffs);

        int getDegreesOfFreedom() const;
        int operator==(DictionaryTrajectory const& comp) const;

        double getTmax();

        arma::vec getStartingPos();

        std::vector<arma::vec> getCoefficients();
        std::vector<QueryPoint> getQueryPoints();

        KUKADU_SHARED_PTR<Trajectory> copy();

    };

}

#endif
