#ifndef KUKADU_TRAJECTORYGENERATOR
#define KUKADU_TRAJECTORYGENERATOR

#include <string>
#include <armadillo>

namespace kukadu {

    /** \brief The TrajectoryGenerator defines an interface to define basis functions for linear regression (see GeneralFitter)
     *
     * An implementation of this class has to define an internal index on the basis functions where each basis function value
     * can be computed by setting the basis function index
     * \ingroup ControlPolicyFramework
     */
    class TrajectoryGenerator {

    private:

    public:

        /**
         * \brief constructor
         */
        TrajectoryGenerator();

        /**
         * \brief evaluates the value of a single basis function
         * \param x value, where the basis function should be evaluated
         * \param fun basis function index that specifies the basis function
         */
        virtual double evaluateBasisFunction(double x, int fun) = 0;

        /**
         * \brief evaluates the linear combination of basis functions by defining the coefficients for a single value x
         * \param x value, where the basis functions should be evaluated
         * \param coeff coefficients that have to be used for computing the linear combination
         */
        virtual double evaluateByCoefficientsSingle(double x, arma::vec coeff) = 0;

        /**
         * \brief performs the same as evaluateByCoefficientsSingle, but for multiple values of x
         * \param x vector of evaluation points
         * \param sampleCount size of vector x
         * \param coeff coefficients that have to be used for computing the linear combination
         */
        virtual arma::vec evaluateByCoefficientsMultiple(arma::vec x, int sampleCount, arma::vec coeff) = 0;

        /**
         * \brief returns the number of basis functions
         */
        virtual int getBasisFunctionCount() = 0;

        /**
         * \brief returns the name of the basis function system as a string
         */
        virtual std::string getTrajectoryType() = 0;

    };

}

#endif
