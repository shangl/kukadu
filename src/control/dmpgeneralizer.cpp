#include <kukadu/control/dmpgeneralizer.hpp>

using namespace std;
using namespace arma;

namespace kukadu {

    DMPGeneralizer::DMPGeneralizer(string baseFolder, int degOfFreedom, vector<double> tmpmys, vector<double> tmpsigmas, double az, double bz) {

        this->degOfFreedom = degOfFreedom;
        this->baseFolder = baseFolder;

        this->ax = ax;
        this->tau = tau;

        dictTraj = new DictionaryTrajectory(baseFolder, az, bz);

    }

    double DMPGeneralizer::getDegOfFreedom() {
        return degOfFreedom;
    }

    QueryPoint DMPGeneralizer::getQueryPointByIndex(int index) {
        return dictTraj->getQueryPoints().at(index);
    }

    int DMPGeneralizer::getQueryPointCount() {
        return dictTraj->getQueryPoints().size();
    }

    // TODO: adapt DMPGeneralizer to new architecture
    KUKADU_SHARED_PTR<JointDmp> DMPGeneralizer::generalizeDmp(GenericKernel* trajectoryKernel, GenericKernel* parameterKernel, vec query, double beta) {

        string errStr = "(DMPGeneralizer) not adapted to new implementation yet";
        cerr << errStr << endl;
        throw KukaduException(errStr.c_str());

        vector<vec> dmpCoeffs;

        int queryCount = dictTraj->getQueryPoints().size();
        int degreesOfFreedom = dictTraj->getQueryPoints().at(0).getDmp()->getDegreesOfFreedom();
        // int samplePointCount = dictTraj->getQueryPoints().at(0).getDmp()->getDesignMatrix(0).n_rows;

        vec g(degreesOfFreedom);

        vector<vec> sampleXs;
        for(int i = 0; i < queryCount; ++i) sampleXs.push_back(dictTraj->getQueryPoints().at(i).getQueryPoint());

        for(int j = 0; j < degreesOfFreedom; ++j) {

            vector<mat> designMatrices;
            vector<vec> sampleTs;

            for(int i = 0; i < queryCount; ++i) {
                sampleTs.push_back(dictTraj->getQueryPoints().at(i).getDmp()->getFitYs().at(j));
                designMatrices.push_back(dictTraj->getQueryPoints().at(i).getDmp()->getDesignMatrix(j));
            }

            LWRRegressor* reg = new LWRRegressor(sampleXs, sampleTs, trajectoryKernel, designMatrices);
            vec res = reg->fitAtPosition(query);
            dmpCoeffs.push_back(res);

        }
        cout << endl;

        for(int j = 0; j < degreesOfFreedom; ++j) {

            vec gs(queryCount);
            for(int i = 0; i < queryCount; ++i) gs(i) = dictTraj->getQueryPoints().at(i).getDmp()->getG()(j);
            GaussianProcessRegressor* reg = new GaussianProcessRegressor(sampleXs, gs, parameterKernel, beta);
            double goal = reg->fitAtPosition(query)(0);
            g(j) = goal;

        }

        return KUKADU_SHARED_PTR<JointDmp>(new JointDmp({}, vector<vec>(), vector<vec>(), dmpCoeffs, dictTraj->getQueryPoints().at(0).getDmp()->getDmpBase(), vector<mat>(), dictTraj->getQueryPoints().at(0).getDmp()->getTau(),
               dictTraj->getQueryPoints().at(0).getDmp()->getAz(), dictTraj->getQueryPoints().at(0).getDmp()->getBz(), dictTraj->getQueryPoints().at(0).getDmp()->getAx()));

    }

    vector<QueryPoint> DMPGeneralizer::mapFiles(vector<string> queryFiles, vector<string> trajFiles, string prefix1, string prefix2) {

        vector<QueryPoint> ret;

        int prefix1Size = prefix1.size();
        int prefix2Size = prefix2.size();
        int querySize = queryFiles.size();
        int trajSize = trajFiles.size();

        for(int i = 0; i < querySize; ++i) {
            string currentQueryFile = string(queryFiles.at(i));
            string queryAppendix = currentQueryFile.substr(prefix1Size, currentQueryFile.size() - 1);
            for(int j = 0; j < trajSize; ++j) {
                string currentTrajFile = string(trajFiles.at(j));
                string trajAppendix = currentTrajFile.substr(prefix2Size, currentTrajFile.size() - 1);
                if(!queryAppendix.compare(trajAppendix)) {
                    // QueryPoint(std::string fileQueryPath, std::string fileDataPath, Dmp dmp, arma::vec queryPoint);
                    QueryPoint toAdd(queryFiles.at(i), trajFiles.at(j), string("dmp_") + trajAppendix + string(".txt"), KUKADU_SHARED_PTR<JointDmp>(new JointDmp()), vec());
                    ret.push_back(toAdd);
                }
            }
        }

        return ret;

    }

}
