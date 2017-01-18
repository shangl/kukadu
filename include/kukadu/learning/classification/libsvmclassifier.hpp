#ifndef KUKADU_LIBSVMCLASSIFIER_H
#define KUKADU_LIBSVMCLASSIFIER_H

#include <kukadu/learning/classification/classifier.hpp>
#include <kukadu/learning/classification/eigenlibsvm/svm_utils.h>

namespace kukadu {

    class LibSvm : public Classifier {

    private:

        esvm::SVMClassifier internalClassifier;

        vector<double> minDim;
        vector<double> maxDim;

        std::vector<arma::mat> scaleDimensions(std::vector<arma::mat> samples, bool storeScalingInfo = false);

    public:

        LibSvm(std::string trainingFile);
        LibSvm(std::vector<int> classes, std::vector<arma::mat> samples);

        virtual bool train();
        virtual int classify(arma::vec sample);

    };

}

#endif
