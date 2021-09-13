#ifndef KUKADU_CUSTOMSET
#define KUKADU_CUSTOMSET

#include <utility>
#include <math.h>
#include <vector>
#include <armadillo>

namespace kukadu {

    // created my own set implementation, because there is either a bug in the library or i am to stupid to define a total order on vectors
    class CustomSet : public std::vector<arma::vec> {

    private:

        int compareArmadilloVec(arma::vec vec1, arma::vec vec2);

    public:

        CustomSet();

        std::pair<std::vector<arma::vec>::iterator, int> insert(arma::vec vec1);
        std::pair<std::vector<arma::vec>::iterator, int> find(arma::vec vec1);

    };

}

#endif
