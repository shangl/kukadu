#include <kukadu/utils/utils.hpp>
#include <kukadu/control/generalizer.hpp>
#include <kukadu/storage/moduleusagesingleton.hpp>

using namespace std;
using namespace arma;

namespace kukadu {

    /****************** public functions *******************************/

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

        KUKADU_MODULE_START_USAGE();

        throw KukaduException("(DMPGeneralizer) not adapted to new implementation yet");

        vector<vec> dmpCoeffs;

        int queryCount = dictTraj->getQueryPoints().size();
        int degreesOfFreedom = dictTraj->getQueryPoints().at(0).getDmp()->getDegreesOfFreedom();

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

        auto retVal = KUKADU_SHARED_PTR<JointDmp>(new JointDmp({}, vector<vec>(), vector<vec>(), dmpCoeffs, dictTraj->getQueryPoints().at(0).getDmp()->getDmpBase(), vector<mat>(), dictTraj->getQueryPoints().at(0).getDmp()->getTau(),
               dictTraj->getQueryPoints().at(0).getDmp()->getAz(), dictTraj->getQueryPoints().at(0).getDmp()->getBz(), dictTraj->getQueryPoints().at(0).getDmp()->getAx()));

        KUKADU_MODULE_END_USAGE();

        return retVal;

    }

    DictionaryTrajectory::DictionaryTrajectory(std::string baseFolder, double az, double bz) : Trajectory() {

        this->baseFolder = baseFolder;
        vector<string> files = getFilesInDirectory(baseFolder);

        queryFiles = sortPrefix(files, "query");
        trajFiles = sortPrefix(files, "traj");
        dmpFiles = sortPrefix(files, "dmp");

        if(dmpFiles.size() == 0) {

            // learn dmps
            queryPoints = mapFiles(queryFiles, trajFiles, "query", "traj");
            KUKADU_SHARED_PTR<JointDMPLearner> dmpLearner;

            vector<mat> jointsVec;
            vector<vec> timesVec;
            double tMax = 0.0;
            for(int i = 0; i < queryPoints.size(); ++i) {

                auto dmpData = readDmpData((string(baseFolder) + string(queryPoints.at(i).getFileDataPath())).c_str());
                vector<long long int> times = dmpData.first;
                mat joints = dmpData.second;
                degOfFreedom = joints.n_cols;

                queryPoints.at(i).setQueryPoint(readQuery(string(baseFolder) + string(queryPoints.at(i).getFileQueryPath())));
                jointsVec.push_back(joints);
                timesVec.push_back(convertAndRemoveOffset(times));
                double currentTMax = times.back();

                if(tMax < currentTMax)
                        tMax = currentTMax;

            }

            for(int i = 0; i < jointsVec.size(); ++i) {

                QueryPoint currentQueryPoint = queryPoints.at(i);
                mat joints = jointsVec.at(i);
                auto filledData = fillTrajectoryMatrix(timesVec.at(i), joints, tMax);
                auto filledTimesInSec = filledData.first;
                joints = filledData.second;
                dmpLearner = KUKADU_SHARED_PTR<JointDMPLearner>(new JointDMPLearner(az, bz, filledTimesInSec, joints));

                KUKADU_SHARED_PTR<Dmp> learnedDmps = dmpLearner->fitTrajectories();
                learnedDmps->serialize(baseFolder + currentQueryPoint.getFileDmpPath());
                queryPoints.at(i).setDmp(learnedDmps);
                startingPos = queryPoints.at(i).getDmp()->getY0();

                cout << "(DMPGeneralizer) goals for query point [" << currentQueryPoint.getQueryPoint().t() << "]" << endl << "\t [";
                cout << currentQueryPoint.getDmp()->getG().t() << "]" << endl;

                //delete dmpLearner;
                dmpLearner = KUKADU_SHARED_PTR<JointDMPLearner>();

            }

        } else {

            queryPoints = mapFiles(queryFiles, trajFiles, dmpFiles, "query", "traj", "dmp");

        }

        degOfFreedom = queryPoints.at(0).getDmp()->getDegreesOfFreedom();

    }

    std::vector<arma::vec> DictionaryTrajectory::getCoefficients() {

        return coefficients;

    }

    void DictionaryTrajectory::setCoefficients(std::vector<arma::vec> coeffs) {

        coefficients = coeffs;

    }

    DictionaryTrajectory::DictionaryTrajectory(const DictionaryTrajectory& copy) : Trajectory(copy) {

        this->degOfFreedom = copy.degOfFreedom;
        this->baseFolder = copy.baseFolder;

        this->files = copy.files;
        this->queryFiles = copy.queryFiles;
        this->trajFiles = copy.trajFiles;
        this->queryPoints = copy.queryPoints;
        this->startingPos = copy.startingPos;

    }

    DictionaryTrajectory::DictionaryTrajectory() {
    }

    int DictionaryTrajectory::getDegreesOfFreedom() const {

        return degOfFreedom;

    }

    int DictionaryTrajectory::operator==(DictionaryTrajectory const& comp) const {

        KUKADU_MODULE_START_USAGE();
        throw KukaduException("(DictionaryTrajectory) == operator not implemted yet");
        KUKADU_MODULE_END_USAGE();

    }

    void DictionaryTrajectory::setTmax(double tmax) {

        for(int i = 0; i < queryPoints.size(); ++i)
            queryPoints.at(i).getDmp()->setTmax(tmax);

    }

    double DictionaryTrajectory::getTmax() {

        return queryPoints.at(0).getDmp()->getTmax();

    }

    std::vector<QueryPoint> DictionaryTrajectory::getQueryPoints() {

        return queryPoints;

    }

    arma::vec DictionaryTrajectory::getStartingPos() {

        KUKADU_MODULE_START_USAGE();

        arma::vec retVal;

        if(startingPos.n_elem > 0)
            retVal = startingPos;
        else
            throw KukaduException("(DictionaryTrajectory) no starting position set yet");

        KUKADU_MODULE_END_USAGE();

        return retVal;

    }

    KUKADU_SHARED_PTR<Trajectory> DictionaryTrajectory::copy() {

        return KUKADU_SHARED_PTR<Trajectory>(new DictionaryTrajectory(*this));

    }

    QueryPoint::QueryPoint(std::string fileQueryPath, std::string fileDataPath, std::string fileDmpPath, KUKADU_SHARED_PTR<Dmp> dmp, arma::vec queryPoint) : internalDmp(dmp) {

        this->fileQueryPath = fileQueryPath;
        this->fileDataPath = fileDataPath;
        this->queryPoint = queryPoint;
        this->fileDmpPath = fileDmpPath;

    }

    QueryPoint::QueryPoint(const QueryPoint& qp) {
        this->fileQueryPath = qp.fileQueryPath;
        this->fileDataPath = qp.fileDataPath;
        this->queryPoint = qp.queryPoint;
        this->internalDmp = qp.internalDmp;
        this->fileDmpPath = qp.fileDmpPath;
    }

    std::string QueryPoint::getFileQueryPath() {
        return fileQueryPath;
    }

    std::string QueryPoint::getFileDataPath() {
        return fileDataPath;
    }

    std::string QueryPoint::getFileDmpPath() {
        return fileDmpPath;
    }

    arma::vec QueryPoint::getQueryPoint() {
        return queryPoint;
    }

    KUKADU_SHARED_PTR<Dmp> QueryPoint::getDmp() {
        return internalDmp;
    }

    void QueryPoint::setQueryPoint(arma::vec queryPoint) {
        this->queryPoint = queryPoint;
    }

    void QueryPoint::setDmp(KUKADU_SHARED_PTR<Dmp> dmp) {
        this->internalDmp = dmp;
    }

    /****************** private functions ******************************/

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

    vector<QueryPoint> DictionaryTrajectory::mapFiles(vector<string> queryFiles, vector<string> trajFiles, string prefix1, string prefix2) {

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
                    QueryPoint toAdd(queryFiles.at(i), trajFiles.at(j), string("dmp") + trajAppendix, make_shared<JointDmp>(), vec());
                    ret.push_back(toAdd);
                    if(i == 0)
                        startingPos = toAdd.getDmp()->getY0();
                }
            }
        }

        return ret;

    }

    vector<QueryPoint> DictionaryTrajectory::mapFiles(vector<string> queryFiles, vector<string> trajFiles, vector<string> dmpFiles, string prefix1, string prefix2, string prefix3) {

        vector<QueryPoint> ret;

        int prefix1Size = prefix1.size();
        int prefix2Size = prefix2.size();
        int prefix3Size = prefix3.size();
        int querySize = queryFiles.size();
        int trajSize = trajFiles.size();
        int dmpSize = dmpFiles.size();

        for(int i = 0; i < querySize; ++i) {
            string currentQueryFile = string(queryFiles.at(i));
            string queryAppendix = currentQueryFile.substr(prefix1Size, currentQueryFile.size() - 1);
            for(int j = 0; j < trajSize; ++j) {
                string currentTrajFile = string(trajFiles.at(j));
                string trajAppendix = currentTrajFile.substr(prefix2Size, currentTrajFile.size() - 1);
                if(!queryAppendix.compare(trajAppendix)) {
                    for(int k = 0; k < dmpSize; ++k) {
                        string currentDmpFile = string(dmpFiles.at(k));
                        string dmpAppendix = currentDmpFile.substr(prefix3Size, currentDmpFile.size() - 1);
                        if(!dmpAppendix.compare(queryAppendix)) {
                            // load dmp from file
                            QueryPoint toAdd(queryFiles.at(i), trajFiles.at(j), prefix3 + trajAppendix, KUKADU_SHARED_PTR<Dmp>(new JointDmp(baseFolder + prefix3 + trajAppendix)), vec());
                            toAdd.setQueryPoint(readQuery(string(baseFolder) + string(toAdd.getFileQueryPath())));
                            ret.push_back(toAdd);
                            if(i == 0)
                                startingPos = toAdd.getDmp()->getY0();
                        }
                    }
                }
            }
        }

        return ret;

    }

    /****************** end ********************************************/

}
