#include <kukadu/statistics/statistics.hpp>

double computeUnalikeability(std::vector<int>& dist) {

    int cSum = 0;
    for(int i = 0; i < dist.size(); ++i) {
        for(int j = 0; j < dist.size(); ++j) {
            if(i == j)
                continue;

            if(dist.at(i) != dist.at(j))
                ++cSum;

        }
    }

    return (double) cSum / (std::pow(dist.size(), 2.0) - dist.size());

}
