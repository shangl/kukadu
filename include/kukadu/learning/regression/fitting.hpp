#ifndef KUKADU_GENERALFITTER_H
#define KUKADU_GENERALFITTER_H

#include <armadillo>
#include <kukadu/control/trajectory.hpp>

namespace kukadu {

    /**
     * \defgroup LearningFramework
     * This framework provides elementary learning techniques such as linear regression and kernel methods
     */

    /** \brief Provides the generic part of the linear regression methods
     *
     * This class provides implements the generic part of the linear regression method. It is independent of the specific choice of basis functions.
     * These basis functions are defined via the TrajectoryGenerator class.
     * \ingroup LearningFramework
     */
    class GeneralFitter {

    private:

        arma::vec samplesX;
        arma::vec samplesY;
        int sampleCount;
        int regressionMode;

        TrajectoryGenerator* trajGen;

    public:

        /**
         * \brief constructor
         * \param samplesX vector of samples, where samplesX specifies the x-axis
         * \param samplesY vector of samples, where samplesY specifies the y-axis
         * \param sampleCount number of sample points
         * \param trajGen TrajectoryGenerator instance defining the basis functions
         */
        GeneralFitter(arma::vec samplesX, arma::vec samplesY, int sampleCount, TrajectoryGenerator* trajGen);

        /**
         * \brief returns the number of basis functions
         */
        int getBasisFunctionCount();

        /**
         * \brief returns the number of samples
         */
        int getSampleCount();

        /**
         * \brief performs linear regression and returns vector of coefficients corresponding to the basis functions
         */
        arma::vec computeLinFitCoefficients();

        /**
         * \brief performs linear regression and returns vector of coefficients corresponding to the basis functions by explicitly taking the design matrix as parameter
         * \param desMat design matrix needed for linear regression
         */
        arma::vec computeLinFitCoefficients(arma::mat desMat);

        /**
         * \brief computes design matrix
         */
        arma::mat computeDesignMatrix();

    };

}

#endif
