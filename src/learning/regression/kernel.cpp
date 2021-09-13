#include <kukadu/learning/regression/kernel.hpp>
#include <kukadu/storage/moduleusagesingleton.hpp>

using namespace std;
using namespace arma;

namespace kukadu {

    GaussianKernel::GaussianKernel(double theta0, double theta1) {
        this->theta0 = theta0;
        this->theta1 = theta1;
    }

    double GaussianKernel::evaluateKernel(vec q1, vec q2, void* kernelParam) {

        KUKADU_MODULE_START_USAGE();

        double ret = 0.0;
        double dist = 0.0;

        vec sub = (q1 - q2);
        dist = norm(sub, 2);

        ret = theta0 * pow(M_E, - theta1 / 2.0 * pow(dist, 2));

        KUKADU_MODULE_END_USAGE();

        return ret;

    }

    TricubeKernel::TricubeKernel() { }

    double TricubeKernel::evaluateKernel(vec q1, vec q2, void* kernelParam) {

        KUKADU_MODULE_START_USAGE();

        double ret = 0.0;
        double dist = 0.0;
        double normal = *( (double*) kernelParam );

        vec sub = 1/normal * (q1 - q2);
        dist = norm(sub, 2);

        if(dist >= 1.0) dist = 1.0;
        ret = 70.0/81.0 * pow(1 - pow(dist, 3), 3);

        KUKADU_MODULE_END_USAGE();

        return ret;

    }

    QuadraticKernel::QuadraticKernel() { }

    // K(u) = 15/16 (1 - |u|^2)^2
    double QuadraticKernel::evaluateKernel(vec q1, vec q2, void* kernelParam) {

        KUKADU_MODULE_START_USAGE();

        double ret = 0.0;
        double dist = 0.0;
        double normal = *( (double*) kernelParam );

        vec sub = 1/normal * (q1 - q2);
        dist = norm(sub, 2);

        if(dist >= 1.0) dist = 1.0;
        ret = 15.0/16.0 * pow(1 - pow(dist, 2), 2);

        KUKADU_MODULE_END_USAGE();

        return ret;

    }

    KernelRegressor::KernelRegressor() { }

    GenericKernel::GenericKernel() { }

    GaussianProcessRegressor::GaussianProcessRegressor(vector<vec> sampleXs, vec sampleTs, GenericKernel* kernel, double beta) {
        this->sampleXs = sampleXs;
        this->sampleTs = sampleTs;
        this->kernel = kernel;
        coVarMatrix = computeCovarianceMatrix(kernel, beta);
    }

    vec GaussianProcessRegressor::fitAtPosition(vec pos) {

        KUKADU_MODULE_START_USAGE();

        vec retVector(1);
        double normVal = computeKernelNormValue();
        int sampleSize = sampleXs.size();

        vec k(sampleSize);
        vec t(sampleSize);
        for(int i = 0; i < sampleSize; ++i) {
            vec currentItem = sampleXs.at(i);
            k(i) = kernel->evaluateKernel(currentItem, pos, &normVal);
        }

        mat ret = k.t() * inv(coVarMatrix) * sampleTs;
        retVector(0) = ret(0,0);

        KUKADU_MODULE_END_USAGE();

        return retVector;

    }

    mat GaussianProcessRegressor::computeCovarianceMatrix(GenericKernel* kernel, double beta) {

        int retDim = sampleXs.size();
        double normVal = computeKernelNormValue();
        mat ret(retDim, retDim);
        for(int i = 0; i < retDim; ++i) {
            for(int j = 0; j < retDim; ++j) {
                vec iItem = sampleXs.at(i);
                vec jItem = sampleXs.at(j);
                double k = kernel->evaluateKernel(iItem, jItem, &normVal);
                if(i != j) ret(i, j) = k;
                else ret(i, j) = k + 1/beta;
            }
        }

        return ret;

    }

    double GaussianProcessRegressor::computeKernelNormValue() {

        double ret = 0.0;
        for(int i = 0; i < sampleXs.size(); ++i) {
            vec currentItem = sampleXs.at(i);
            ret += norm(currentItem, 2);
        }

        return ret / sampleXs.size();

    }

}

