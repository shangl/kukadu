#include <kukadu/types/dictionarytrajectory.hpp>

using namespace std;
using namespace arma;

namespace kukadu {

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

    // TODO: implement == operator
    int DictionaryTrajectory::operator==(DictionaryTrajectory const& comp) const {

        string errStr = "(DictionaryTrajectory) == operator not implemted yet";
        cerr << errStr << endl;
        throw KukaduException(errStr.c_str());

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

        string errStr = "(DictionaryTrajectory) no starting position set yet";

        if(startingPos.n_elem > 0)
            return startingPos;

        cerr << errStr << endl;
        throw errStr;

    }

    KUKADU_SHARED_PTR<Trajectory> DictionaryTrajectory::copy() {

        return KUKADU_SHARED_PTR<Trajectory>(new DictionaryTrajectory(*this));

    }

}
