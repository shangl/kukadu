#ifndef KUKADU_STATISTICS_H
#define KUKADU_STATISTICS_H

#include <vector>
#include <armadillo>

namespace kukadu {

    double computeEntropy(arma::vec& probabilities);
    double computeUnalikeability(std::vector<int>& dist);

}

#endif
