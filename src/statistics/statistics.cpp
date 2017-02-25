#include <cmath>
#include <kukadu/statistics/statistics.hpp>
#include <kukadu/storage/moduleusagesingleton.hpp>

using namespace std;

namespace kukadu {

    double computeUnalikeability(std::vector<int>& dist) {

        KUKADU_MODULE_START_USAGE();

        int cSum = 0;
        for(int i = 0; i < dist.size(); ++i) {
            for(int j = 0; j < dist.size(); ++j) {
                if(i == j)
                    continue;

                if(dist.at(i) != dist.at(j))
                    ++cSum;

            }
        }

        auto retVal = (double) cSum / (std::pow(dist.size(), 2.0) - dist.size());

        KUKADU_MODULE_END_USAGE();

        return retVal;

    }

    double computeEntropy(arma::vec& probabilities) {

        double entropy = 0.0;
        for(int i = 0; i < probabilities.n_elem; ++i)
            entropy -= probabilities(i) * log2(probabilities(i));

        return entropy;

    }

}
