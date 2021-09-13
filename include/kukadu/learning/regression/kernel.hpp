#ifndef KUKADU_KERNEL_H
#define KUKADU_KERNEL_H

#include <armadillo>

namespace kukadu {

    /** \brief Provides an interface for generic kernel functions.
     *
     * The GenericKernel class is used by the KernelRegressor. Implementations of GenericKernel have to provide a certain kernel function, according to the kernel criteria.
     * \ingroup LearningFramework
     */
    class GenericKernel {

    private:

    public:

        /**
         * \brief constructor
         */
        GenericKernel();

        /**
         * \brief computes kernel values with given vectors q1 and q2 and passes a not further specified kernel parameter that can be used by the kernel implementation
         * \param q1 vector q1 with K = K(d(q1, q2))
         * \param q2 vector q2 with K = K(d(q1, q2))
         * \param kernelParam arbitrary kernel parameter
         */
        virtual double evaluateKernel(arma::vec q1, arma::vec q2, void* kernelParam) = 0;

    };

    /** \brief Implements a Gaussian kernel according to the GenericKernel specifiction.
     *
     * This class implements a Gaussian kernel given by the function K(u) = theta0 e^(- theta1 /2 u^2)
     * \ingroup LearningFramework
     */
    class GaussianKernel : public GenericKernel {

    private:

        double theta0;
        double theta1;

    public:

        /**
         * \brief constructor
         * \param theta0 paremeter theta0 of the kernel according to the given kernel function
         * \param theta1 paremeter theta1 of the kernel according to the given kernel function
         */
        GaussianKernel(double theta0, double theta1);

        double evaluateKernel(arma::vec q1, arma::vec q2, void* kernelParam);

    };

    /** \brief Implements a tricube kernel according to the GenericKernel specifiction.
     *
     * This class implements a tricube kernel given by the function K(u) = 70/81 (1 - |u|^3)^3
     * \ingroup LearningFramework
     */
    class TricubeKernel : public GenericKernel {

    private:

    public:

        /**
         * \brief constructor
         */
        TricubeKernel();

        double evaluateKernel(arma::vec q1, arma::vec q2, void* kernelParam);

    };

    class QuadraticKernel : public GenericKernel {

    private:

    public:

        /**
         * \brief constructor
         */
        QuadraticKernel();

        double evaluateKernel(arma::vec q1, arma::vec q2, void* kernelParam);

    };

    /** \brief Provides arbitrary interface for kernel methods
     *
     * This class provides the interface for kernel machine learning methods such as Gaussian process regression or locally weighted regression
     * \ingroup LearningFramework
     */
    class KernelRegressor {

    private:


    public:

        /**
         * \brief constructor
         */
        KernelRegressor();

        /**
         * \brief performs the kernel method for predicting the functin value at a given position
         * \param pos vector defining the required position
         */
        virtual arma::vec fitAtPosition(arma::vec pos) = 0;

    };

    /** \brief Implements the locally weighted regression method
     *
     * This class inherits from the KernelRegressor and implements locally weighted regression. It reuses the design matrices computed by the
     * GeneralFitter class.
     * \ingroup LearningFramework
     */
    class LWRRegressor : public KernelRegressor {

    private:

        std::vector<arma::vec> sampleXs;
        std::vector<arma::vec> sampleTs;

        std::vector<arma::mat> designMatrices;

        GenericKernel* kernel;

        double computeKernelNormValue();

    public:

        /**
         * \brief constructor, setting all the sample data, the selected kernel and a vector of design matrices (one design matrix for each sample)
         * \param sampleXs vector of samples (x-axis)
         * \param sampleTs vector of samples (y-axis)
         * \param kernel the selected kernel implementation
         * \param designMatrices vector of design matrices
         */
        LWRRegressor(std::vector<arma::vec> sampleXs, std::vector<arma::vec> sampleTs, GenericKernel* kernel, std::vector<arma::mat> designMatrices);
        arma::vec fitAtPosition(arma::vec pos);

    };

    /** \brief Implements the Gaussian process regression method
     *
     * This class inherits from the KernelRegressor and implements Gaussian process regression.
     * \ingroup LearningFramework
     */
    class GaussianProcessRegressor : public KernelRegressor {
    private:

        std::vector<arma::vec> sampleXs;
        arma::vec sampleTs;
        arma::mat coVarMatrix;

        GenericKernel* kernel;

        arma::mat computeCovarianceMatrix(GenericKernel* kernel, double beta);
        double computeKernelNormValue();

    public:

        /**
         * \brief constructor, setting all the sample data, the selected kernel and the variance beta
         * \param sampleXs vector of samples (x-axis)
         * \param sampleTs vector of samples (y-axis)
         * \param kernel the selected kernel implementation
         * \param beta the assumed sample variance
         */
        GaussianProcessRegressor(std::vector<arma::vec> sampleXs, arma::vec sampleTs, GenericKernel* kernel, double beta);

        arma::vec fitAtPosition(arma::vec pos);

    };

}

#endif
