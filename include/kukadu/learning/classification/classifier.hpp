#ifndef KUKADU_CLASSIFIER_H
#define KUKADU_CLASSIFIER_H

#include <armadillo>

namespace kukadu {

    class Classifier {

    private:

        std::vector<int> classes;
        std::vector<arma::mat> samples;

    public:

        Classifier(std::vector<int> classes, std::vector<arma::mat> samples);

        std::vector<int> getClasses();
        std::vector<arma::mat> getSamples();

        virtual bool trainClassifier() = 0;
        virtual int classify(arma::vec sample) = 0;

    };

}

#endif
