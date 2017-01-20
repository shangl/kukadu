#include <string>
#include <fstream>
#include <utility>
#include <iostream>
#include <libsvm/svm.h>
#include <kukadu/utils/utils.hpp>
#include <kukadu/types/kukadutypes.hpp>
#include <kukadu/utils/kukadutokenizer.hpp>
#include <kukadu/learning/classification/classifier.hpp>
#include <kukadu/learning/classification/libsvmclassifier.hpp>

using namespace std;
using namespace arma;
using namespace svmpp;

namespace kukadu {

    Classifier::Classifier(std::vector<int> classes, std::vector<arma::mat> samples) {
        this->classes = classes;
        this->samples = samples;
    }

    std::vector<int> Classifier::getClasses() {
        return classes;
    }

    std::vector<arma::mat> Classifier::getSamples() {
        return samples;
    }

    int Classifier::getSampleDimensionality() {
        return samples.front().n_cols;
    }

    std::pair<std::vector<int>, std::vector<arma::mat> > loadLibsvmFile(std::string trainingFile) {

        ifstream s;
        s.open(trainingFile.c_str());

        vector<int> classIds;
        vector<mat> classSamples;

        string line;
        int sampleDim = 0;
        bool firstRun = true;
        while(getline(s, line)) {

            if(firstRun) {

                KukaduTokenizer tok(line);
                tok.next();
                string t;

                while((t = tok.next()) != "")
                    sampleDim++;

                firstRun = false;
            }

            KukaduTokenizer tok(line);
            int classId = atoi(tok.next().c_str());

            auto classIdIt = std::find(classIds.begin(), classIds.end(), classId);

            bool insertNew = false;
            if(classIdIt == classIds.end()) {
                insertNew = true;
                classIds.push_back(classId);
                mat newClassSampleMat;
                classSamples.push_back(newClassSampleMat);
                classIdIt = classIds.end() - 1;
            }

            int classIdIdx = (int) (classIdIt - classIds.begin());

            string token;
            int dimIdx = 0;
            mat newSampleMat(1, sampleDim);
            mat& currentClassSampleMat = classSamples.at(classIdIdx);

            while((token = tok.next()) != "") {

                KukaduTokenizer valueToken(token, ":");
                // ignore dimension label
                valueToken.next();
                double val = atof(valueToken.next().c_str());

                if(dimIdx < newSampleMat.n_cols)
                    newSampleMat(0, dimIdx) = val;

                dimIdx++;

            }

            currentClassSampleMat = join_cols(currentClassSampleMat, newSampleMat);

        }

        return {classIds, classSamples};

    }

    LibSvm::LibSvm(std::string trainingFile) : Classifier(loadLibsvmFile(trainingFile).first, scaleDimensions(loadLibsvmFile(trainingFile).second).first) {
        auto scaled = scaleDimensions(loadLibsvmFile(trainingFile).second, true, false);
        wasTrained = false;
        minDim = scaled.second.first;
        maxDim = scaled.second.second;
    }

    LibSvm::LibSvm(std::vector<int> classes, std::vector<arma::mat> samples) : Classifier(classes, scaleDimensions(samples).first) {

        auto scaled = scaleDimensions(samples, true, false);
        wasTrained = false;
        minDim = scaled.second.first;
        maxDim = scaled.second.second;

    }

    std::pair<std::vector<arma::mat>, std::pair<std::vector<double>, std::vector<double> > > LibSvm::scaleDimensions(std::vector<arma::mat> samples, bool storeScalingInfo, bool useStoredScalingInfo) {

        vector<double> minDimInt;
        vector<double> maxDimInt;

        for(mat& sampleMat : samples) {

            vec minCol(sampleMat.n_rows);
            vec maxCol(sampleMat.n_rows);
            vec lowerScale(sampleMat.n_rows);
            vec upperScale(sampleMat.n_rows);
            lowerScale.fill(-1.0);
            upperScale.fill(1.0);
            for(int i = 0; i < sampleMat.n_cols; ++i) {

                double currentMin;
                double currentMax;

                if(useStoredScalingInfo) {
                    currentMin = this->minDim.at(i);
                    currentMax = this->maxDim.at(i);
                } else {
                    currentMin = sampleMat.col(i).min();
                    currentMax = sampleMat.col(i).max();
                }

                if(storeScalingInfo) {
                    minDimInt.push_back(currentMin);
                    maxDimInt.push_back(currentMax);
                }

                minCol.fill(currentMin);
                maxCol.fill(currentMax);

                // no reason to scale columns where everything is 0
                if((currentMin != 0.0 || currentMax != 0.0) && (currentMax - currentMin) > 0.0) {
                    // scaling as it is done in the svmlib scaling tool
                    sampleMat.col(i) = lowerScale + (upperScale - lowerScale) % (sampleMat.col(i) - minCol) / (maxCol - minCol);
                    if(sampleMat.col(i).max() > 1.0 || sampleMat.col(i).min() < -1.0)
                        for(int j = 0; j < sampleMat.n_rows; ++j) {
                            sampleMat(j, i) = min(sampleMat(j, i), 1.0);
                            sampleMat(j, i) = max(sampleMat(j, i), -1.0);
                        }

                }

            }

        }

        return {samples, {minDimInt, maxDimInt}};

    }

    bool LibSvm::train() {

        generateTrainSet();
        internalClassifier = svmpp::Svm();

        setStdParams();
        internalClassifier.train(params, trainSet);

        wasTrained = true;

        return true;

    }

    void LibSvm::generateTrainSet() {

        auto classes = getClasses();
        auto samples = getSamples();
        if(classes.size() != samples.size())
            throw KukaduException("(LibSvm) classes size and samples size do not match");

        if(!classes.size())
            throw KukaduException("(LibSvm) no samples provided");

        int sampleDim = samples.front().n_cols;
        for(auto& sample : samples) {
            if(sampleDim != sample.n_cols)
                throw KukaduException("(LibSvm) sample dimensions do not match");
            if(sample.has_nan())
                throw KukaduException("(LibSvm) data contains NaN");
        }

        trainSet = svmpp::TrainSet();

        for(int i = 0; i < classes.size(); ++i) {
            auto& currentClass = classes.at(i);
            auto& currentClassSamples = samples.at(i);
            for(int j = 0; j < currentClassSamples.n_rows; ++j)
                trainSet.addEntry(armadilloToStdVec(currentClassSamples.row(j).t()), currentClass);
        }

    }

    void LibSvm::setStdParams() {

        // Setting parameters
        params.svm_type = C_SVC;
        params.kernel_type = RBF;
        params.degree = 3;
        params.gamma = 1.0 / getSampleDimensionality();
        params.coef0 = 0.0;
        params.cache_size = 100.0;
        params.eps = 0.001;
        params.C = 5.0;
        params.nr_weight = 0;
        params.weight_label = NULL;
        params.weight = NULL;
        params.nu = 0.5;
        params.p = 0.1;
        params.shrinking = 1;
        params.probability = 1;

    }

    int LibSvm::classify(arma::vec sample) {

        if(!wasTrained)
            throw KukaduException("(LibSvm) classifier was not trained yet");

        if(getSampleDimensionality() != sample.n_elem)
            throw KukaduException("(LibSvm) test sample dimensions does not fit the database dimension");

        mat x(1, sample.n_elem);
        for(int i = 0; i < sample.n_elem; ++i)
            x(0, i) = sample(i);

        vec scaledData = scaleDimensions({x}, false, true).first.front().row(0).t();

        Query query(armadilloToStdVec(scaledData));
        return internalClassifier.predict(query);

    }

    double LibSvm::crossValidate() {

        generateTrainSet();
        internalClassifier = svmpp::Svm();

        setStdParams();
        return internalClassifier.crossValidation(params, trainSet);

    }

}
