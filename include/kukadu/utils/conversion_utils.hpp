#ifndef CONVERSIONUTILS
#define CONVERSIONUTILS

#include <vector>
#include <fstream>
#include <iostream>
#include <armadillo>

#include "../utils/kukadutokenizer.hpp"

namespace kukadu {

    int compareArmadilloVec(arma::vec vec1, arma::vec vec2);
    int compareVectorOfDoubles(std::vector<double> vec1, std::vector<double> vec2);
    int compareVectorOfArmadillos(std::vector<arma::vec> vec1, std::vector<arma::vec> vec2);

    double string_to_double(const std::string& s);

    std::string double_to_string(const double d);

    arma::mat readMat(std::ifstream& inStream);

}

#endif
