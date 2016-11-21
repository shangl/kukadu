#ifndef KUKADU_POLYTRAJECTORYGENERATOR
#define KUKADU_POLYTRAJECTORYGENERATOR

#include <armadillo>
#include <kukadu/control/trajectorygenerator.hpp>

namespace kukadu {

    /** \brief Implements the TrajectoryGenerator interface for polynomials
     *
     * This class provides simple polynomials as basis functions. f(x) = sum c_i x^i
     * \ingroup ControlPolicyFramework
     */
    class PolyTrajectoryGenerator : public TrajectoryGenerator {

    private:

        int basisFunctionCount;

    public:

        /**
         * \brief constructor. the polynomials are defined by giving the degree of the polynomial
         * \param basisFunctionCount polynomial degree
         */
        PolyTrajectoryGenerator(int basisFunctionCount);

        double evaluateBasisFunction(double x, int fun);
        double evaluateByCoefficientsSingle(double x, arma::vec coeff);
        arma::vec evaluateByCoefficientsMultiple(arma::vec x, int sampleCount, arma::vec coeff);

        int getBasisFunctionCount();

        std::string getTrajectoryType();

    };

}

#endif
