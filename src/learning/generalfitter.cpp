#include <kukadu/learning/regression/generalfitter.hpp>

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

        vec t(sampleCount);

        for(int i = 0; i < sampleCount; ++i)
            t.at(i) = samplesY(i);

        vec res = solve(desMat, t);

        cout << "(GeneralFitter) learned coefficients: " << res.t() << endl;

        // not needed
        // int basisFunctionCount = trajGen->getBasisFunctionCount();
        // vec resid = t - desMat * res;
        // double sig2 = as_scalar( trans(resid) * resid / (sampleCount - basisFunctionCount) );
        // vec stderrest = sqrt( sig2 * diagvec( inv(trans(desMat) * desMat)) );

        return res;

    }

    vec GeneralFitter::computeLinFitCoefficients() {

        mat desMat = computeDesignMatrix();
        vec ret = computeLinFitCoefficients(desMat);

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
