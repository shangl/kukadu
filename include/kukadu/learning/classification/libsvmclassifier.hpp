#ifndef KUKADU_LIBSVMCLASSIFIER_H
#define KUKADU_LIBSVMCLASSIFIER_H

#include <kukadu/learning/classification/svmpp.hpp>
#include <kukadu/learning/classification/classifier.hpp>

namespace kukadu {

    std::pair<std::vector<int>, std::vector<arma::mat> > loadLibsvmFile(std::string trainingFile);

    class LibSvm : public Classifier {

    private:

        bool wasTrained;

        svmpp::TrainSet trainSet;
        svmpp::Svm::Params params;
        svmpp::Svm internalClassifier;

        std::vector<double> minDim;
        std::vector<double> maxDim;

        void setStdParams();
        void generateTrainSet();

    public:

        LibSvm(std::string trainingFile);
        LibSvm(std::vector<int> classes, std::vector<arma::mat> samples);

        virtual bool train();
        virtual int classify(arma::vec sample);

        virtual double crossValidate();

        std::pair<std::vector<arma::mat>, std::pair<std::vector<double>, std::vector<double> > > scaleDimensions(std::vector<arma::mat> samples, bool useStoredScalingInfo = false);

    };

}

#endif
