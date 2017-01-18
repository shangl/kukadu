#include <string>
#include <fstream>
#include <utility>
#include <iostream>
#include <kukadu/types/kukadutypes.hpp>
#include <kukadu/utils/kukadutokenizer.hpp>
#include <kukadu/learning/classification/classifier.hpp>
#include <kukadu/learning/classification/libsvmclassifier.hpp>

using namespace std;
using namespace arma;

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

    LibSvm::LibSvm(std::string trainingFile) : Classifier(loadLibsvmFile(trainingFile).first, loadLibsvmFile(trainingFile).second) {

    }

    LibSvm::LibSvm(std::vector<int> classes, std::vector<arma::mat> samples) : Classifier(classes, samples) {

    }

    bool LibSvm::trainClassifier() {

        auto classes = getClasses();
        auto samples = getSamples();
        if(classes.size() != samples.size())
            throw KukaduException("(LibSvm) classes size and samples size do not match");

        if(!classes.size())
            throw KukaduException("(LibSvm) no samples provided");

        int samplesCount = 0;
        int sampleDim = samples.front().n_cols;
        for(auto& sample : samples) {
            samplesCount += sample.n_rows;
            if(sampleDim != sample.n_cols)
                throw KukaduException("(libSvm) sample dimensions do not match");
        }

        vector<int> labelsVec;
        mat samplesMat(samplesCount, sampleDim);

        int runningIdx = 0;
        for(int i = 0; i < classes.size(); ++i) {
            auto& currentClass = classes.at(i);
            auto& currentClassSamples = samples.at(i);
            for(int j = 0; j < currentClassSamples.n_rows; ++j) {
                labelsVec.push_back(currentClass);
                for(int k = 0; k < currentClassSamples.n_cols; ++k)
                    samplesMat(runningIdx, k) = currentClassSamples(j, k);
                ++runningIdx;
            }
        }

        internalClassifier.train(samplesMat, labelsVec);

    }

    int LibSvm::classify(arma::vec sample) {

        vector<int> res;
        mat x(1, sample.n_elem);
        for(int i = 0; i < sample.n_elem; ++i)
            x(1, i) = sample(i);

        internalClassifier.test(x, res);

        return res.at(0);

    }

}
