#include <kukadu/statistics/statistics.hpp>
#include <kukadu/storage/moduleusagesingleton.hpp>

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

}
