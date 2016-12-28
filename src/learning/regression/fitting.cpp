#include <kukadu/types/kukadutypes.hpp>
#include <kukadu/learning/regression/fitting.hpp>
#include <kukadu/storage/moduleusagesingleton.hpp>

using namespace std;
using namespace arma;

namespace kukadu {

    GeneralFitter::GeneralFitter(vec samplesX, vec samplesY, int sampleCount, TrajectoryGenerator* trajGen) {
        this->samplesX = samplesX;
        this->samplesY = samplesY;
        this->sampleCount = sampleCount;
        this->trajGen = trajGen;
    }

    int GeneralFitter::getSampleCount() {
        return sampleCount;
    }

    vec GeneralFitter::computeLinFitCoefficients(mat desMat) {

        KUKADU_MODULE_START_USAGE();

        vec t(sampleCount);

        for(int i = 0; i < sampleCount; ++i)
            t.at(i) = samplesY(i);

        vec res = solve(desMat, t);
        for(int i = 0; i < res.n_elem; ++i)
            // if any element is NaN
            if(res(i) != res(i))
                throw KukaduException("(GeneralFitter) Learned coefficients contain NaN");

        KUKADU_MODULE_END_USAGE();

        return res;

    }

    vec GeneralFitter::computeLinFitCoefficients() {

        KUKADU_MODULE_START_USAGE();

        mat desMat = computeDesignMatrix();
        vec ret = computeLinFitCoefficients(desMat);

        KUKADU_MODULE_END_USAGE();

        return ret;

    }

    mat GeneralFitter::computeDesignMatrix() {

        int basisFunctionCount = trajGen->getBasisFunctionCount();
        mat X(sampleCount, basisFunctionCount);

        // computation of design matrix according to bishop equation 3.16 (page 142)
        for(int i = 0; i < sampleCount; ++i) {
            for(int j = 0; j < basisFunctionCount; j++) {
                double val = trajGen->evaluateBasisFunction(samplesX(i), j);
                X(i, j) = val;
            }
        }

        return X;

    }

}
