#include <kukadu/utils/customset.hpp>

using namespace std;
using namespace arma;

namespace kukadu {

    CustomSet::CustomSet() {
    }

    std::pair<std::vector<arma::vec>::iterator, int> CustomSet::insert(arma::vec vec1) {

        std::pair<std::vector<arma::vec>::iterator, int> retPair;

        if(!(find(vec1).second)) {
            this->push_back(vec1);
            retPair.first = this->end();
            retPair.second = this->size();
            return retPair;
        }

        retPair.first = this->end();
        retPair.second = 0;
        return retPair;

    }

    std::pair<std::vector<arma::vec>::iterator, int> CustomSet::find(arma::vec vec1) {

        std::pair<std::vector<arma::vec>::iterator, int> retPair;

        int i = 0;
        for(std::vector<arma::vec>::iterator it = this->begin(); it != this->end(); ++it, ++i) {
            if(compareArmadilloVec(*it, vec1)) {
                retPair.first = it;
                retPair.second = i + 1;
                return retPair;
            }
        }

        retPair.first = this->end();
        retPair.second = 0;

        return retPair;

    }

    int CustomSet::compareArmadilloVec(arma::vec vec1, arma::vec vec2) {

        int retVal = 1;

        if(vec1.n_elem != vec2.n_elem)
            retVal = 0;

        for(int i = 0; i < vec1.n_elem; ++i) {
            if(vec1(i) != vec2(i))
                retVal = 0;
        }

        return retVal;

    }

}
