#ifndef DMPTYPES
#define DMPTYPES

#include <vector>
#include <armadillo>
#include <gsl/gsl_matrix.h>

struct mes_result {
    long long int time;
    arma::vec joints;
};

struct trajectory_learner_internal {
	arma::mat desMat;
	arma::vec coeff;
	arma::vec fity;
};

#endif
