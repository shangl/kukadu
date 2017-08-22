#include <sstream>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_matrix.h>
#include <kukadu/utils/utils.hpp>
#include <kukadu/control/dmp.hpp>
#include <tf/transform_datatypes.h>
#include <kukadu/storage/sensorstorage.hpp>
#include <kukadu/utils/conversion_utils.hpp>
#include <kukadu/storage/moduleusagesingleton.hpp>

using namespace std;
using namespace arma;

namespace kukadu {

    std::vector<double> constructDmpMys(vec& timesInSeconds, mat& joints) {
        vector<double> ret;
        double tmax = timesInSeconds(timesInSeconds.n_rows - 1);
        for(double i = 0; i < (tmax + 1); i += 1.0)
            ret.push_back(i);

        return ret;
    }

    vector<double> computeDMPMys(vector<double> mys, double ax, double tau) {
        int mysSize = mys.size();
        vector<double> dmpMys;
        for(int i = 0; i < mysSize; ++i) {
            double val = std::exp(-ax / tau * mys.at(i));
            dmpMys.push_back(val);
        }
        return dmpMys;
    }

    std::vector<DMPBase> buildDMPBase(vector<double> tmpmys, vector<double> tmpsigmas, double ax, double tau) {

        std::vector<DMPBase> baseDef;
        vector<DMPBase>::iterator it = baseDef.begin();

        vector<double> mys = computeDMPMys(tmpmys, ax, tau);

        for(int i = 0; i < mys.size(); ++i) {

            double realMy = tmpmys.at(i);
            double my = mys.at(i);

            vector<double> sigmas;

            for(int j = 0; j < tmpsigmas.size(); ++j) {
                double realSigma = tmpsigmas.at(j);
                double sigma = my - std::exp(-ax / tau * (realMy + realSigma));
                sigmas.push_back(sigma);
            }

            DMPBase base(my, sigmas);

            // with this implementation, currently all sigmas have to be of the same size (see DMPTrajectoryGenerator::evaluateBasisFunction)
            it = baseDef.insert(it, base);

        }

        return baseDef;

    }

    /****************** public functions *******************************/

    CartesianDMP::CartesianDMP(std::vector<long long int> supervisedTs, std::vector<arma::vec> sampleYs, std::vector<arma::vec> fitYs, std::vector<arma::vec> dmpCoeffs, std::vector<DMPBase> dmpBase, std::vector<arma::mat> designMatrices,
                               double tau, double az, double bz, double ax, double ac, double dmpStepSize, double tolAbsErr, double tolRelErr) : Dmp(supervisedTs, sampleYs, fitYs, dmpCoeffs, dmpBase, designMatrices,
                                                                                                                                                     tau, az, bz, ax, ac, dmpStepSize, tolAbsErr, tolRelErr) {
    }

    CartesianDMP::CartesianDMP(std::vector<long long int> supervisedTs, std::vector<arma::vec> sampleYs, std::vector<arma::vec> fitYs, std::vector<arma::vec> dmpCoeffs, std::vector<DMPBase> dmpBase, std::vector<arma::mat> designMatrices,
                               double tau, double az, double bz, double ax) : Dmp(supervisedTs, sampleYs, fitYs, dmpCoeffs, dmpBase, designMatrices, tau, az, bz, ax) {

    }

    CartesianDMP::CartesianDMP() {
    }

    bool CartesianDMP::isCartesian() {
        return true;
    }

    KUKADU_SHARED_PTR<Trajectory> CartesianDMP::copy() {

        return KUKADU_SHARED_PTR<Trajectory>(new CartesianDMP(*this));

    }

    tf::Quaternion CartesianDMP::getQ0() {
        return getQByIdx(0);
    }

    tf::Quaternion CartesianDMP::getQByIdx(int idx) {
        return tf::Quaternion(sampleYs.at(3)(idx), sampleYs.at(4)(idx), sampleYs.at(5)(idx), sampleYs.at(6)(idx));
    }

    arma::vec CartesianDMP::getEta0() {

        return getEtaByIdx(0);

    }

    arma::vec CartesianDMP::getEtaByIdx(int idx) {

        arma::vec omega(3);
        omega.fill(0.0);

        if(idx < getSampleCount()) {
            tf::Quaternion q0 = getQByIdx(idx);
            tf::Quaternion q1 = getQByIdx(idx + 1);

            omega = 2 * kukadu::log(q1 * q0.inverse());

        }

        return getTau() * omega / this->dmpStepSize;
    }

    tf::Quaternion CartesianDMP::getQg() {
        return getQByIdx(getSampleCount() - 1);
    }

    DMPBase::DMPBase() {
        my = 0.0;
    }

    DMPBase::DMPBase(float my, std::vector<double> sigmas) {
        this->my = my;
        this->sigmas = sigmas;
    }

    float DMPBase::getMy() {
        return my;
    }

    std::vector<double> DMPBase::getSigmas() {
        return sigmas;
    }

    int DMPBase::operator==(DMPBase const& comp) const {
        return ( my == comp.my && compareVectorOfDoubles(sigmas, comp.sigmas) );
    }

    Dmp::Dmp(std::vector<long long int> supervisedTs, std::vector<arma::vec> sampleYs, std::vector<arma::vec> fitYs, std::vector<arma::vec> dmpCoeffs, std::vector<DMPBase> dmpBase, std::vector<arma::mat> designMatrices,
            double tau, double az, double bz, double ax, double ac, double dmpStepSize, double tolAbsErr, double tolRelErr)
                    : SingleSampleTrajectory(convertAndRemoveOffset(supervisedTs), sampleYs) {

        construct(fitYs, dmpCoeffs, dmpBase, designMatrices, tau, az, bz, ax, ac, dmpStepSize, tolAbsErr, tolRelErr);

    }

    Dmp::Dmp(std::vector<long long int> supervisedTs, std::vector<arma::vec> sampleYs, std::vector<arma::vec> fitYs, std::vector<arma::vec> dmpCoeffs, std::vector<DMPBase> dmpBase, std::vector<arma::mat> designMatrices,
            double tau, double az, double bz, double ax) : SingleSampleTrajectory(convertAndRemoveOffset(supervisedTs), sampleYs) {

        construct(fitYs, dmpCoeffs, dmpBase, designMatrices, tau, az, bz, ax, 10.0, 1.8 * 1e4 * 1e-6, 0.1, 0.1);

    }

    Dmp::Dmp() : SingleSampleTrajectory(vec(), vector<vec>()) {

    }

    Dmp::Dmp(std::string dmpFile) {

        std::ifstream dmpFileStream(dmpFile.c_str(), std::ifstream::in);
        mat m = readMat(dmpFileStream);
        int degreesOfFreedom = m(0,0);

        m = readMat(dmpFileStream);
        int baseTermCount = m(0,0);

        m = readMat(dmpFileStream);
        vec fileSuperVisedTs = m.col(0);

        vector<vec> fileSampleYs;
        for(int i = 0; i < degreesOfFreedom; ++i) {
            vec fileCurrentSampleY = readMat(dmpFileStream).col(0);
            fileSampleYs.push_back(fileCurrentSampleY);
        }

        vector<vec> fileFitYs;
        for(int i = 0; i < degreesOfFreedom; ++i) {
            vec fileCurrentFitY = readMat(dmpFileStream).col(0);
            fileFitYs.push_back(fileCurrentFitY);
        }

        vector<vec> fileDmpCoeffs;
        for(int i = 0; i < degreesOfFreedom; ++i) {
            vec fileCurrentCoeffs = readMat(dmpFileStream).col(0);
            fileDmpCoeffs.push_back(fileCurrentCoeffs);
        }

        vector<DMPBase> fileDmpBases;
        for(int i = 0; i < baseTermCount; ++i) {

            mat fileCurrentMyMat = readMat(dmpFileStream);
            double fileCurrentMy = fileCurrentMyMat(0,0);
            vec fileCurrentSigmasVec = readMat(dmpFileStream).col(0);
            vector<double> fileCurrentSigmas;
            for(int j = 0; j < fileCurrentSigmasVec.n_elem; ++j)
                fileCurrentSigmas.push_back(fileCurrentSigmasVec(j));
            DMPBase currentBase(fileCurrentMy, fileCurrentSigmas);
            fileDmpBases.push_back(currentBase);

        }

        vector<mat> fileDesignMatrices;
        for(int i = 0; i < degreesOfFreedom; ++i) {
            mat fileCurrentDesignMatrix = readMat(dmpFileStream);
            fileDesignMatrices.push_back(fileCurrentDesignMatrix);
        }

        vec fileDmpConsts = readMat(dmpFileStream).row(0).t();
        this->setSupervisedTs(fileSuperVisedTs);
        this->setSampleYs(fileSampleYs);

        construct(fileFitYs, fileDmpCoeffs, fileDmpBases, fileDesignMatrices,
                  fileDmpConsts(0), fileDmpConsts(1), fileDmpConsts(2), fileDmpConsts(3), fileDmpConsts(4),
                  fileDmpConsts(5), fileDmpConsts(6), fileDmpConsts(7));

        cout << "(Dmp) Dmp loaded from file" << endl;

    }

    void Dmp::storeDatabase(std::string skillLabel, bool overwriteIfExists) {

    }

    void Dmp::serialize(string dmpFile) {

        string baseString = "";

        int i = 0;
        for(i = 0; i < getDmpBase().size(); ++i) {

            // serialize base
            DMPBase currentBase = getDmpBase().at(i);
            double currentMy = currentBase.getMy();

            vector<double> currentSigmas = currentBase.getSigmas();
            baseString += double_to_string(currentMy) + "\n\n" + "=" + "\n";
            for(int j = 0; j < currentSigmas.size(); ++j)
                baseString += double_to_string(currentSigmas.at(j)) + "\n\n";
            baseString = baseString + "=" + "\n";

        }

        ofstream dmpFileStream;
        dmpFileStream.open(dmpFile.c_str());

        dmpFileStream << getDegreesOfFreedom() << endl;
        dmpFileStream << "=" << endl;
        dmpFileStream << i << endl << getDmpBase().at(0).getSigmas().size() << endl;
        dmpFileStream << "=" << endl;

        dmpFileStream << getSupervisedTs() << endl;
        dmpFileStream << "=" << endl;

        for(int i = 0; i < getSampleYs().size(); ++i) {
            dmpFileStream << getSampleYByIndex(i) << endl;
            dmpFileStream << "=" << endl;
        }

        for(int i = 0; i < getFitYs().size(); ++i) {
            dmpFileStream << getFitYs().at(i) << endl;
            dmpFileStream << "=" << endl;
        }

        for(int i = 0; i < getDmpCoeffs().size(); ++i) {
            dmpFileStream << getDmpCoeffs().at(i) << endl;
            dmpFileStream << "=" << endl;
        }

       dmpFileStream << baseString;

        for(int i = 0; i < getDesignMatrixCount(); ++i) {
            dmpFileStream << getDesignMatrix(i) << endl;
            dmpFileStream << "=" << endl;
        }

        dmpFileStream << tau << " " << az << " " << bz << " " << ax << " " << ac << " " << dmpStepSize << " " << tolAbsErr << " " << tolRelErr << endl;

    }

    int Dmp::getSampleCount() {
        return sampleYs.at(0).size();
    }

    double Dmp::getDeltaTByIdx(int idx) {
        return (supervisedTs(idx + 1) - supervisedTs(idx));
    }

    void Dmp::construct(std::vector<arma::vec> &fitYs, std::vector<arma::vec> &dmpCoeffs, std::vector<DMPBase> &dmpBase, std::vector<arma::mat> &designMatrices,
            double tau, double az, double bz, double ax, double ac, double dmpStepSize, double tolAbsErr, double tolRelErr) {

        this->dmpCoeffs = dmpCoeffs;
        this->dmpBase = dmpBase;
        this->designMatrices = designMatrices;
        this->tau = tau;
        this->az = az;
        this->bz = bz;
        this->ax = ax;
        this->fitYs = fitYs;

        this->ac = ac;
        this->dmpStepSize = dmpStepSize;
        this->tolAbsErr = tolAbsErr;
        this->tolRelErr = tolRelErr;

        initializeY0();
        initializeDy0();
        initializeDdy0();
        initializeG();

        int s = getDataPointsNum();
        if(s > 0)
            tmax = getT(s - 1);
        else
            tmax = 0.0;

    }

    void Dmp::setDmpCoeffs(std::vector<arma::vec> coeffs) {
        dmpCoeffs = coeffs;
    }

    Dmp::Dmp(const Dmp& copy) : SingleSampleTrajectory(copy) {

        this->dmpCoeffs = copy.dmpCoeffs;
        this->dmpBase = copy.dmpBase;
        this->designMatrices = copy.designMatrices;
        this->tau = copy.tau;
        this->az = copy.az;
        this->bz = copy.bz;
        this->ax = copy.ax;
        this->dmpStepSize = copy.dmpStepSize;
        this->ac = copy.ac;
        this->tolAbsErr = copy.tolAbsErr;
        this->tolRelErr = copy.tolRelErr;

        initializeY0();
        initializeDy0();
        initializeDdy0();
        initializeG();

        this->tmax = copy.tmax;

    }

    Dmp::Dmp(std::vector<arma::vec> dmpCoeffs, std::vector<DMPBase> dmpBase, arma::vec y0, arma::vec dy0, arma::vec g, double tMax, double tau, double az, double bz, double ax, double ac, double dmpStepSize, double tolAbsErr, double tolRelErr) {
        this->dmpCoeffs = dmpCoeffs;
        this->dmpBase = dmpBase;
        this->tau = tau;
        this->az = az;
        this->bz = bz;
        this->ax = ax;
        this->ac = ac;
        this->dmpStepSize = dmpStepSize;
        this->tolAbsErr = tolAbsErr;
        this->tolRelErr = tolRelErr;
        this->tmax = tMax;
        this->y0 = y0;
        this->g = g;
        this->dy0 = dy0;
    }

    std::vector<arma::vec> Dmp::getFitYs() {
        return fitYs;
    }

    arma::vec Dmp::getY0() {
        return y0;
    }

    arma::vec Dmp::getDy0() {
        return dy0;
    }

    arma::vec Dmp::getDdy0() {
        return ddy0;
    }

    arma::vec Dmp::getG() {
        return g;
    }

    int Dmp::getDesignMatrixCount() {
        return designMatrices.size();
    }

    arma::mat Dmp::getDesignMatrix(int freedomIdx) {
        return designMatrices.at(freedomIdx);
    }

    std::vector<arma::vec> Dmp::getDmpCoeffs() {
        return dmpCoeffs;
    }

    std::vector<arma::vec> Dmp::getCoefficients() {
        return getDmpCoeffs();
    }

    double Dmp::getY0(int freedomIdx) {
        return y0(freedomIdx);
    }

    double Dmp::getDy0(int freedomIdx) {
        return dy0(freedomIdx);
    }

    double Dmp::getDdy0(int freedomIdx) {
        return ddy0(freedomIdx);
    }

    double Dmp::getG(int freedomIdx) {
        return g(freedomIdx);
    }

    arma::vec Dmp::getDmpCoeffs(int freedomIdx) {
        return dmpCoeffs.at(freedomIdx);
    }

    void Dmp::setCoefficients(std::vector<arma::vec> coeffs) {
        setDmpCoeffs(coeffs);
    }

    std::vector<DMPBase> Dmp::getDmpBase() {
        return dmpBase;
    }

    double Dmp::getTau() {
        return tau;
    }

    double Dmp::getAz() {
        return az;
    }

    double Dmp::getBz() {
        return bz;
    }

    double Dmp::getAx() {
        return ax;
    }

    double Dmp::getStepSize() {
        return dmpStepSize;
    }

    double Dmp::getTolAbsErr() {
        return tolAbsErr;
    }

    double Dmp::getTolRelErr() {
        return tolRelErr;
    }

    double Dmp::getTmax() {

        return tmax;

    }

    void Dmp::setTmax(double tmax) {
        this->tmax = tmax;
    }

    void Dmp::initializeY0() {

        int degOfFreemdom = getDegreesOfFreedom();

        y0 = vec(degOfFreemdom);
        for(int i = 0; i < degOfFreemdom; ++i)
            y0(i) = getDataPoint(i, 0);

    }

    void Dmp::initializeDy0() {

        int degOfFreemdom = getDegreesOfFreedom();
        dy0 = vec(degOfFreemdom);
        for(int i = 0; i < degOfFreemdom; ++i)
            dy0(i) = ( getDataPoint(i, 1) - getDataPoint(i, 0) ) / ( getT(1) - getT(0) );

    }

    void Dmp::initializeDdy0() {

        int degOfFreemdom = getDegreesOfFreedom();
        ddy0 = vec(degOfFreemdom);
        for(int i = 0; i < degOfFreemdom; ++i) {
            double dy = ( getDataPoint(i, 2) - getDataPoint(i, 1) ) / ( getT(2) - getT(1) );
            ddy0(i) = ( dy - dy0(i) ) / ( getT(2) - getT(0) );
        }

    }

    void Dmp::initializeG() {

        int s = getDataPointsNum();
        int degOfFreemdom = getDegreesOfFreedom();
        g = vec(degOfFreemdom);
        for(int i = 0; i < degOfFreemdom; ++i)
            g(i) = getDataPoint(i, s - 1);

    }

    int Dmp::operator==(KUKADU_SHARED_PTR<Dmp> const& comp) const {

        // design matrices are ignored here
        return (compareVectorOfArmadillos(dmpCoeffs, comp->dmpCoeffs) && compareArmadilloVec(y0, comp->y0) && compareArmadilloVec(dy0, comp->dy0) &&
                    compareArmadilloVec(ddy0, comp->ddy0) && compareArmadilloVec(g, comp->g) && dmpBase == comp->dmpBase &&
                    tau == comp->tau && az == comp->az && bz == comp->bz && ax == comp->ax );

    }

    JointDmp::JointDmp(std::vector<long long int> supervisedTs, std::vector<arma::vec> sampleYs, std::vector<arma::vec> fitYs, std::vector<arma::vec> dmpCoeffs, std::vector<DMPBase> dmpBase, std::vector<arma::mat> designMatrices,
                            double tau, double az, double bz, double ax, double ac, double dmpStepSize, double tolAbsErr, double tolRelErr) : Dmp(supervisedTs, sampleYs, fitYs, dmpCoeffs, dmpBase, designMatrices,
                                                                                                                                                  tau, az, bz, ax, ac, dmpStepSize, tolAbsErr, tolRelErr) {
    }

    JointDmp::JointDmp(std::vector<long long int> supervisedTs, std::vector<arma::vec> sampleYs, std::vector<arma::vec> fitYs, std::vector<arma::vec> dmpCoeffs, std::vector<DMPBase> dmpBase, std::vector<arma::mat> designMatrices,
                       double tau, double az, double bz, double ax) : Dmp(supervisedTs, sampleYs, fitYs, dmpCoeffs, dmpBase, designMatrices, tau, az, bz, ax) {

    }

    JointDmp::JointDmp(std::string dmpFile) : Dmp(dmpFile) {

    }

    JointDmp::JointDmp() {

    }

    JointDmp::JointDmp(std::vector<arma::vec> dmpCoeffs, std::vector<DMPBase> dmpBase, arma::vec y0, arma::vec dy0, arma::vec g, double tMax, double tau, double az, double bz, double ax, double ac, double dmpStepSize, double tolAbsErr, double tolRelErr)
        : Dmp(dmpCoeffs, dmpBase, y0, dy0, g, tMax, tau, az, bz, ax, ac, dmpStepSize, tolAbsErr, tolRelErr) {

    }

    bool JointDmp::isCartesian() {
        return false;
    }

    KUKADU_SHARED_PTR<Trajectory> JointDmp::copy() {
        return make_shared<JointDmp>(*this);
    }

    CartesianDMPLearner::CartesianDMPLearner(std::vector<DMPBase> dmpBase, double tau, double az, double bz, double ax, arma::vec timesInSeconds, arma::mat joints) : GeneralDmpLearner(dmpBase, tau, az, bz, ax, timesInSeconds, joints) {

    }

    CartesianDMPLearner::CartesianDMPLearner(std::vector<double> mysDef, std::vector<double> sigmasDef, double az, double bz, std::string file) : GeneralDmpLearner(mysDef, sigmasDef, az, bz, file) {

    }

    CartesianDMPLearner::CartesianDMPLearner(double az, double bz, std::string file) : GeneralDmpLearner(az, bz, file) {

    }

    CartesianDMPLearner::CartesianDMPLearner(double az, double bz, arma::vec timesInSeconds, arma::mat joints) : GeneralDmpLearner(az, bz, timesInSeconds, joints) {

    }

    KUKADU_SHARED_PTR<Dmp> CartesianDMPLearner::createDmpInstance(arma::vec& supervisedTsInSeconds, std::vector<arma::vec> sampleYs, std::vector<arma::vec> fitYs, std::vector<arma::vec> dmpCoeffs, std::vector<DMPBase> dmpBase, std::vector<arma::mat> designMatrices,
                                                                double tau, double az, double bz, double ax) {
        return make_shared<CartesianDMP>(convertTimesInSecondsToTimeInMilliseconds(supervisedTsInSeconds), sampleYs, fitYs, dmpCoeffs, dmpBase, designMatrices, tau, az, bz, ax);
    }

    DMPExecutor::DMPExecutor(StorageSingleton& dbStorage, KUKADU_SHARED_PTR<Dmp> traj, KUKADU_SHARED_PTR<ControlQueue> execQueue, int suppressMessages) :
        TrajectoryExecutor(dbStorage, execQueue, traj) {
        construct(execQueue, suppressMessages);
    }

    DMPExecutor::DMPExecutor(StorageSingleton& dbStorage, int skillId, KUKADU_SHARED_PTR<ControlQueue> execQueue, int suppressMessages) :
        TrajectoryExecutor(dbStorage, execQueue, loadDmpFromDb(dbStorage, skillId, execQueue)) {
        construct(execQueue, suppressMessages);
    }

    void DMPExecutor::setExecutionDuration(double tmax) {
        executionDuration = tmax;
    }

    KUKADU_SHARED_PTR<Dmp> DMPExecutor::getDmp() {
        return dmp;
    }

    bool compareDouble(double& d1, double& d2, int precision) {
        double mult = pow(10, precision);
        return ((int) (d1 * mult) == (int) (d2 * mult));
    }

    std::pair<vector<DMPBase>, vector<vec> > loadBaseAndCoefficients(StorageSingleton& dbStorage, int skillId, KUKADU_SHARED_PTR<ControlQueue> execQueue, int robotId) {

        stringstream s;
        // load dmp base from database
        vector<DMPBase> bases;
        cout << "(dmp) warning: dmp is assumed to be unique for different configurations" << endl;
        s << "select co.joint_id as ji, my, sigma, coeff from skill_dmp_coeff as co inner join skills as ski on co.skill_id = ski.skill_id " <<
             " inner join skills_robot skr on skr.skill_id = ski.skill_id inner join robot_config on skr.robot_config_id=robot_config.robot_config_id" <<
             " where ski.skill_id = " << skillId
                << " and robot_config.hardware_instance_id = " << robotId << " order by co.joint_id asc, co.my asc, co.sigma asc";

        auto baseRes = dbStorage.executeQuery(s.str());

        /************ loading base and coefficients ************/

        bool firstRunAfterFirstJoint = true;
        int firstJointId = -1;
        int prevJointId = -1;
        double firstMy = -1;
        double prevMy = -1;
        vector<double> currentSigmas;
        vector<double> currentCoefficientsPerJoint;
        vector<vec> allCoeffs;

        while(baseRes->next()) {

            auto currentJointId = baseRes->getInt("ji");
            double currentMy = baseRes->getDouble("my");
            double currentSigma = baseRes->getDouble("sigma");
            double currentCoeff = baseRes->getDouble("coeff");

            /** start generating the base **/

            // remember first joint id
            if(firstJointId == -1) {
                firstJointId = prevJointId = currentJointId;
                firstMy = prevMy = currentMy;
            }

            // if first joint --> construct the base
            if(firstJointId == currentJointId) {

                if(compareDouble(currentMy, prevMy, 6))
                    currentSigmas.push_back(currentSigma);
                else {
                    DMPBase b(prevMy, currentSigmas);
                    bases.push_back(b);
                    currentSigmas.clear();
                    currentSigmas.push_back(currentSigma);
                    prevMy = currentMy;

                }

            }
            // if it is the first run after the first joint --> we still have to insert the previous my
            else if(firstRunAfterFirstJoint) {
                DMPBase b(prevMy, currentSigmas);
                bases.push_back(b);
                firstRunAfterFirstJoint = false;
            }

            /** end generating the base **/

            // this part loads the coefficients

            // if we are still in the same joint --> insert the coefficient to that joint
            if(prevJointId == currentJointId) {
                // do nothing
            // else --> one joint is complete --> add it and clear the vector
            } else {
                allCoeffs.push_back(stdToArmadilloVec(currentCoefficientsPerJoint));
                currentCoefficientsPerJoint.clear();
                prevJointId = currentJointId;
            }

            currentCoefficientsPerJoint.push_back(currentCoeff);

        }

        // add last coefficients as well
        allCoeffs.push_back(stdToArmadilloVec(currentCoefficientsPerJoint));

        if(allCoeffs.size() != execQueue->getDegreesOfFreedom() || allCoeffs.front().n_elem != (bases.front().getSigmas().size() * bases.size()))
            throw KukaduException("(DMPExecutor.loadBaseAndCoefficients) malformed skill information in database");

        return {bases, allCoeffs};

    }

    arma::vec loadJointsFromTable(StorageSingleton& dbStorage, std::string table, int skillId, KUKADU_SHARED_PTR<ControlQueue> execQueue) {

        stringstream s;
        s << "select joint_id, position from " << table << " where skill_id = " << skillId << " order by joint_id asc";
        auto jointsRes = dbStorage.executeQuery(s.str());
        vector<double> joints;

        int jointCount = 0;
        while(jointsRes->next()) {
            joints.push_back(jointsRes->getDouble("position"));
            ++jointCount;
        }

        if(jointCount != execQueue->getDegreesOfFreedom())
            throw KukaduException("(DMPExecutor.loadJointsFromTable) malformed skill information in database");

        return stdToArmadilloVec(joints);

    }

    std::vector<double> loadDmpParameters(StorageSingleton& dbStorage, int skillId) {

        stringstream s;
        s << "select tau, az, bz, ax, step_size, tol_abs_err, tol_rel_err, tmax from skill_dmp where skill_id = " << skillId;
        auto parameterRes = dbStorage.executeQuery(s.str());
        if(parameterRes->next())
            return {(double) parameterRes->getDouble("tau"), (double) parameterRes->getDouble("az"), (double) parameterRes->getDouble("bz"), (double) parameterRes->getDouble("ax"),
            (double) parameterRes->getDouble("step_size"), (double) parameterRes->getDouble("tol_abs_err"), (double) parameterRes->getDouble("tol_rel_err"),
            (double) parameterRes->getDouble("tmax")};
        else
            throw KukaduException("(DMPExecutor.loadDmpParameters) malformed skill information in database");
        return {};

    }

    KUKADU_SHARED_PTR<Dmp> DMPExecutor::loadDmpFromDb(StorageSingleton& dbStorage, int skillId, KUKADU_SHARED_PTR<ControlQueue> execQueue) {

        auto robotId = execQueue->getRobotId();

        auto baseAndCoeff = loadBaseAndCoefficients(dbStorage, skillId, execQueue, robotId);
        auto& base = baseAndCoeff.first;
        auto& coeff = baseAndCoeff.second;

        auto y0 = loadJointsFromTable(dbStorage, "skill_dmp_y0", skillId, execQueue);
        auto dy0 = loadJointsFromTable(dbStorage, "skill_dmp_dy0", skillId, execQueue);
        auto g = loadJointsFromTable(dbStorage, "skill_dmp_g", skillId, execQueue);

        auto dmpParams = loadDmpParameters(dbStorage, skillId);

        return make_shared<JointDmp>(coeff, base, y0, dy0, g, dmpParams.at(7), dmpParams.at(0), dmpParams.at(1), dmpParams.at(2), dmpParams.at(3),
                                     0.0, dmpParams.at(4), dmpParams.at(5), dmpParams.at(6));

    }

    void DMPExecutor::createSkillFromThisInternal(std::string skillName) {

        // if it is a joint dmp --> skill creation is supported, otherwise not yet
        if(!isCartesian) {

            KUKADU_SHARED_PTR<Dmp> jointDmp = KUKADU_DYNAMIC_POINTER_CAST<Dmp>(getTrajectory());

            // dmp specific information
            auto y0 = jointDmp->getY0();
            auto dy0 = jointDmp->getDy0();
            auto g = jointDmp->getG();
            auto coeff = jointDmp->getCoefficients();
            auto tau = jointDmp->getTau();
            auto az = jointDmp->getAz();
            auto bz = jointDmp->getBz();
            auto ax = jointDmp->getAx();
            auto dmpStepSize = jointDmp->getStepSize();
            auto tolAbsErr= jointDmp->getTolAbsErr();
            auto tolRelErr = jointDmp->getTolRelErr();
            auto tMax = jointDmp->getTmax();
            auto dmpBase = jointDmp->getDmpBase();

            // robot information
            auto jointIds = controlQueue->getJointIds();

            auto skillId = getStorage().getCachedLabelId("skills", "skill_id", "label", skillName);

            stringstream s;
            s.str("");
            s << "insert into skill_dmp(skill_id, tau, az, bz, ax, step_size, tol_abs_err, tol_rel_err, tmax) values(" << skillId << ", " << tau << ", " <<
                 az << ", " << bz << ", " << ax << ", " << dmpStepSize << ", " << tolAbsErr << ", " << tolRelErr << ", " << tMax << ")";
            getStorage().executeStatementPriority(s.str());

            stringstream goalStream;
            stringstream dy0Stream;
            for(int i = 0; i < jointIds.size(); ++i) {
                s.str("");
                goalStream.str("");
                dy0Stream.str("");
                auto& currentJointId = jointIds.at(i);
                s << "insert into skill_dmp_y0(skill_id, joint_id, position) values(" << skillId << ", " << currentJointId << ", " << y0(i) << ")";
                goalStream << "insert into skill_dmp_g(skill_id, joint_id, position) values(" << skillId << ", " << currentJointId << ", " << g(i) << ")";
                dy0Stream << "insert into skill_dmp_dy0(skill_id, joint_id, position) values(" << skillId << ", " << currentJointId << ", " << dy0(i) << ")";
                getStorage().executeStatementPriority(s.str());
                getStorage().executeStatementPriority(goalStream.str());
                getStorage().executeStatementPriority(dy0Stream.str());
            }

            // the dmp basis functions are the same for all joints --> but this is not fixed forever,
            // so the database model is such that this can be changed straight forward in the future (for now redundant information is stored)
            for(int i = 0; i < jointIds.size(); ++i) {

                auto& currentCoeffs = coeff.at(i);
                for(int j = 0, m = 0; j < dmpBase.size(); ++j) {

                    s.str("");
                    auto& currentBase = dmpBase.at(j);
                    vector<double> currentSigmas = currentBase.getSigmas();

                    s << "insert into skill_dmp_coeff(skill_id, joint_id, my, sigma, coeff) values(" << skillId << ", " << jointIds.at(i) << ", " << currentBase.getMy();
                    auto firstPart = s.str();
                    for(int k = 0; k < currentSigmas.size(); ++k) {
                        double& sigma = currentSigmas.at(k);
                        s.str("");
                        s << firstPart << ", " << sigma << ", " << currentCoeffs(m) << ")";
                        getStorage().executeStatementPriority(s.str());
                        ++m;
                    }

                }

            }

        } else
            throw KukaduException("(DMPExecutor) skill creation only for joint dmps supported");

    }

    bool DMPExecutor::requiresGraspInternal() {
        return false;
    }

    bool DMPExecutor::producesGraspInternal() {
        return true;
    }

    std::pair<bool, std::string> DMPExecutor::getAugmentedInfoTableName() {
        return {true, "skill_dmp"};
    }

    void  DMPExecutor::setRollbackTime(double rollbackTime) {
        KUKADU_MODULE_START_USAGE();
        this->rollbackTime = rollbackTime;
        KUKADU_MODULE_END_USAGE();
    }

    void DMPExecutor::setTrajectory(KUKADU_SHARED_PTR<Trajectory> traj) {

        KUKADU_SHARED_PTR<Dmp> dmp = KUKADU_DYNAMIC_POINTER_CAST<Dmp>(traj);
        construct(controlQueue, suppressMessages);

        vec_t.clear();
        vec_y.clear();

    }

    void DMPExecutor::useExternalError(int external) {
        KUKADU_MODULE_START_USAGE();
        externalErrorUsing = external;
        KUKADU_MODULE_END_USAGE();
    }

    void DMPExecutor::setExternalError(double error) {
        externalError = error;
    }

    int DMPExecutor::usesExternalError() {
        return externalErrorUsing;
    }

    double DMPExecutor::getExternalError() {
        return externalError;
    }

    KUKADU_SHARED_PTR<ControllerResult> DMPExecutor::simulateTrajectory(double tStart, double tEnd, double tolAbsErr, double tolRelErr) {

        KUKADU_MODULE_START_USAGE();

        setExecutionMode(SIMULATE_ROBOT);
        KUKADU_SHARED_PTR<ControllerResult> ret = this->executeDMP(tStart, tEnd, tolAbsErr, tolRelErr);

        KUKADU_MODULE_END_USAGE();
        return ret;

    }

    void DMPExecutor::setExecutionMode(enum execution_modes mode) {
        TrajectoryExecutor::setExecutionMode(mode);
    }

    void DMPExecutor::setDoRollBackOnMaxForceEvent(bool doRollBackOnMaxForceEvent) {
        this->doRollBackOnMaxForceEvent(doRollBackOnMaxForceEvent);
    }

    void DMPExecutor::doRollBackOnMaxForceEvent(bool doRollback) {
        KUKADU_MODULE_START_USAGE();
        this->doRollback = doRollback;
        cout << "(DMPExecutor) doRollback was set to " << this->doRollback << endl;
        KUKADU_MODULE_END_USAGE();
    }

    void DMPExecutor::setMaxAbsForce(double maxAbsForce) {
        if(maxAbsForce == IGNORE_FORCE)
            this->maxAllowedForce = DBL_MAX;
        else
            this->maxAllowedForce = maxAbsForce;
    }

    void DMPExecutor::setMaxXForce(double maxXForce) {
        if(maxXForce == IGNORE_FORCE)
            this->maxXForce = DBL_MAX;
        else
            this->maxXForce = maxXForce;
    }

    void DMPExecutor::setMaxYForce(double maxYForce) {
        if(maxYForce == IGNORE_FORCE)
            this->maxYForce = DBL_MAX;
        else
            this->maxYForce = maxYForce;
    }

    void DMPExecutor::setMaxZForce(double maxZForce) {
        if(maxZForce == IGNORE_FORCE)
            this->maxZForce = DBL_MAX;
        else
            this->maxZForce = maxZForce;
    }


    void DMPExecutor::enableMaxForceMode(double maxAbsForce, double maxXForce, double maxYForce, double maxZForce) {
        KUKADU_MODULE_START_USAGE();

        if(maxAbsForce == IGNORE_FORCE)
            maxAllowedForce = DBL_MAX;
        else
            maxAllowedForce = maxAbsForce;

        if(maxXForce == IGNORE_FORCE)
            this->maxXForce = DBL_MAX;
        else
            this->maxXForce = maxXForce;

        if(maxYForce == IGNORE_FORCE)
            this->maxYForce = DBL_MAX;
        else
            this->maxYForce = maxYForce;

        if(maxZForce == IGNORE_FORCE)
            this->maxZForce = DBL_MAX;
        else
            this->maxZForce = maxZForce;

        KUKADU_MODULE_END_USAGE();
    }



    KUKADU_SHARED_PTR<ControllerResult> DMPExecutor::simulateTrajectory() {

        double tmax = dmp->getTmax();
        if(executionDuration > 0)
            tmax = executionDuration;

        setExecutionMode(SIMULATE_ROBOT);

        return this->executeDMP(0, tmax, dmp->getTolAbsErr(), dmp->getTolRelErr());

    }

    KUKADU_SHARED_PTR<ControllerResult> DMPExecutor::executeTrajectory() {

        double tmax = dmp->getTmax();
        if(executionDuration > 0)
            tmax = executionDuration;

        setExecutionMode(EXECUTE_ROBOT);

        return this->executeDMP(0, tmax, dmp->getTolAbsErr(), dmp->getTolRelErr());

    }

    void DMPExecutor::setCycleTime(double cycleTime) {
        customCycleTime = cycleTime;
    }

    KUKADU_SHARED_PTR<ControllerResult> DMPExecutor::executeDMP(double tStart, double tEnd, double tolAbsErr, double tolRelErr) {

        KUKADU_MODULE_START_USAGE();

        if(customCycleTime < 0.0)
            throw KukaduException("(DMPExecutor) neither a control queue nor a cycle time were provided");

        // two variables are really required here, because executionRunning is changed by other functions that really need to know
        // whether the exeuction was stopped (this is checked by executionStoppingDone)
        executionRunning = true;
        executionStoppingDone = false;
        if(controlQueue)
            if(!isCartesian) {
                maxFrcThread = KUKADU_SHARED_PTR<kukadu_thread>(new kukadu_thread(&DMPExecutor::runCheckMaxForces, this));
                controlQueue->startRollBackMode(3.0);
            }

        double currentTime = 0.0;

        vector<vec> retY;

        vector<double> retT;
        geometry_msgs::Pose start;

        if(controlQueue) {
            if(!isCartesian) {
                vec currentState = controlQueue->getCurrentJoints().joints;
                for(int i = 0; i < y0s.n_elem; ++i)
                    if(std::abs(currentState(i) - y0s(i)) > 0.01) {
                        controlQueue->jointPtp(y0s);
                        ros::Rate r(2); r.sleep();
                        break;
                    }
            } else {
                start = controlQueue->getCurrentCartesianPose();
                controlQueue->cartesianPtp(vectorarma2pose(&y0s));
            }
        }

        initializeIntegration(0, tolAbsErr, tolRelErr);

        vec nextJoints(degofFreedom);
        nextJoints.fill(0.0);

        // execute dmps and compute linear combination
        for(int j = 0; currentTime < tEnd && executionRunning; ++j, currentTime += customCycleTime) {

            try {

                nextJoints = doIntegrationStep(ac);

            } catch(const char* s) {
                cerr << string(s) << ": stopped execution at time " << currentTime << endl;
                break;
            }

            retY.push_back(nextJoints);

            if(controlQueue && getExecutionMode() == EXECUTE_ROBOT) {

                controlQueue->synchronizeToQueue(1);
                if(!isCartesian)
                    controlQueue->move(nextJoints);
                else {

                    geometry_msgs::Pose newP = vectorarma2pose(&nextJoints);
                    controlQueue->move(newP);

                }

            }

            retT.push_back(currentTime);

        }

        executionRunning = false;
        executionStoppingDone = true;

        if(maxFrcThread && maxFrcThread->joinable())
            maxFrcThread->join();

        auto retVal = KUKADU_SHARED_PTR<ControllerResult>(new ControllerResult(stdToArmadilloVec(retT), retY, true));

        KUKADU_MODULE_END_USAGE();

        return retVal;

    }

    DMPTrajectoryGenerator::DMPTrajectoryGenerator(std::vector<DMPBase> baseDef, double ax, double tau) {

        this->baseFunctionCount = -1;
        this->baseDef = baseDef;

        this->ax = ax;
        this->tau = tau;

        this->previousX = -1;
        this->prevBasFun = vec(getBasisFunctionCount());

        myssize = baseDef.size();
        sigmassize = baseDef.at(0).getSigmas().size();

    }

    double DMPTrajectoryGenerator::evaluateByCoefficientsSingle(double x, vec coeff) {

        int coeffDegree = this->getBasisFunctionCount();
        double val = 0.0;

        if(previousX != x) {
            for(int i = 0; i < coeffDegree; ++i) {
                prevBasFun(i) = evaluateBasisFunctionNonExponential(x, i);
                previousX = x;
            }
        }

        for(int i = 0; i < coeffDegree; ++i)
            val += coeff(i) * prevBasFun(i);

        return val;

    }

    double DMPTrajectoryGenerator::evaluateByCoefficientsSingleNonExponential(double x, vec coeff) {

        int coeffDegree = this->getBasisFunctionCount();
        double val = 0.0;

        if(previousX != x) {

            for(int i = 0; i < coeffDegree; ++i) {
                prevBasFun(i) = evaluateBasisFunctionNonExponential(x, i);
                previousX = x;
            }

        }

        for(int i = 0; i < coeffDegree; ++i)
            val += coeff(i) * prevBasFun(i);

        return val;

    }


    vec DMPTrajectoryGenerator::evaluateByCoefficientsMultiple(vec x, int sampleCount, vec coeff) {
        int coeffDegree = this->getBasisFunctionCount();
        vec values(sampleCount);
        for(int i = 0; i < sampleCount; ++i) {
            values(i) = evaluateByCoefficientsSingle(x(i), coeff);
        }
        return values;
    }

    vec DMPTrajectoryGenerator::evaluateByCoefficientsMultipleNonExponential(vec x, int sampleCount, vec coeff) {
        int coeffDegree = this->getBasisFunctionCount();
        vec values(sampleCount);
        for(int i = 0; i < sampleCount; ++i) {
            values(i) = evaluateByCoefficientsSingleNonExponential(x(i), coeff);
        }
        return values;
    }

    // with this implementation, currently all sigmas have to be of the same size (see DMPTrajectoryGenerator::evaluateBasisFunction and DMPTrajectoryGenerator::getBasisFunctionCount)
    double DMPTrajectoryGenerator::evaluateBasisFunction(double x, int fun) {

        int mypos = fun / sigmassize;
        int sigmapos = fun % sigmassize;

        double my = baseDef.at(mypos).getMy();
        double sigma = baseDef.at(mypos).getSigmas().at(sigmapos);

        double expVal = exp( -ax / tau * x );

        double base = exp(  - pow( expVal  - my, 2) / (2 * pow(sigma, 2))   ) * expVal;
        double normVal = computeNormalization(exp( -ax / tau * x ));

        return base / normVal;

    }

    double DMPTrajectoryGenerator::evaluateBasisFunctionNonExponential(double x, int fun) {

        int mypos = fun / sigmassize;
        int sigmapos = fun % sigmassize;

        double my = baseDef.at(mypos).getMy();
        double sigma = baseDef.at(mypos).getSigmas().at(sigmapos);
        double base = exp( -pow( x - my , 2 ) / (2 * pow(sigma, 2)) ) * x;
        double normVal = computeNormalization(x);

        return base / normVal;

    }

    // with this implementation, currently all sigmas have to be of the same size (see DMPTrajectoryGenerator::evaluateBasisFunction and DMPTrajectoryGenerator::getBasisFunctionCount)
    int DMPTrajectoryGenerator::getBasisFunctionCount() {

        if(baseFunctionCount == -1) {
            int myssize = baseDef.size();
            int sigmassize = baseDef.at(0).getSigmas().size();
            baseFunctionCount = myssize * sigmassize;
        }

        return baseFunctionCount;

    }

    string DMPTrajectoryGenerator::getTrajectoryType() {
        return "dmp";
    }

    GeneralDmpLearner::GeneralDmpLearner(std::vector<DMPBase> dmpBase, double tau, double az, double bz, double ax, arma::vec timesInSeconds, mat joints) {
        this->construct(dmpBase, tau, az, bz, ax, timesInSeconds, joints, joints.n_cols);
    }

    GeneralDmpLearner::GeneralDmpLearner(vector<double> mysDef, vector<double> sigmasDef, double az, double bz, string file) {

        // reading in file
        auto dmpData = readDmpData(file);
        vector<long long int> timesInMilliSeconds = dmpData.first;
        mat joints = dmpData.second;

        double tau = timesInMilliSeconds.back();
        double ax = -std::log(0.1) / tau / tau;

        int degFreedom = joints.n_cols;

        vector<DMPBase> baseDef = buildDMPBase(mysDef, sigmasDef, ax, tau);

        this->construct(baseDef, tau, az, bz, ax, convertAndRemoveOffset(timesInMilliSeconds), joints, degFreedom);

    }

    GeneralDmpLearner::GeneralDmpLearner(double az, double bz, std::string file) {

        vector<double> tmpmys;

        vector<double> tmpsigmas;
        tmpsigmas.push_back(0.2); tmpsigmas.push_back(0.8);

        auto dmpData = readDmpData(file);
        vector<long long int> timesInMilliSeconds = dmpData.first;
        vec timesInSeconds = convertAndRemoveOffset(timesInMilliSeconds);
        mat joints = dmpData.second;

        int degFreedom = joints.n_cols;

        tmpmys = constructDmpMys(timesInSeconds, joints);

        double tau = timesInMilliSeconds.back();
        double ax = -std::log(0.1) / tau / tau;

        vector<DMPBase> baseDef = buildDMPBase(tmpmys, tmpsigmas, ax, tau);
        this->construct(baseDef, tau, az, bz, ax, timesInSeconds, joints, degFreedom);

    }

    GeneralDmpLearner::GeneralDmpLearner(double az, double bz, arma::vec timesInSeconds, arma::mat joints) {

        vector<double> tmpmys;
        vector<double> tmpsigmas;
        tmpsigmas.push_back(0.2); tmpsigmas.push_back(0.8);
        int degFreedom = joints.n_cols;

        tmpmys = constructDmpMys(timesInSeconds, joints);

        double tau = timesInSeconds(timesInSeconds.n_rows - 1, 0);
        double ax = -std::log(0.1) / tau / tau;

        vector<DMPBase> baseDef = buildDMPBase(tmpmys, tmpsigmas, ax, tau);
        this->construct(baseDef, tau, az, bz, ax, timesInSeconds, joints, degFreedom);

    }

    KUKADU_SHARED_PTR<Dmp> GeneralDmpLearner::fitTrajectories() {

        KUKADU_MODULE_START_USAGE();

        int dataPointsNum = joints.n_rows;

        vec g(degFreedom);
        vec y0(degFreedom);
        vec dy0(degFreedom);
        vec ddy0(degFreedom);

        vector<vec> dmpCoeffs;
        vector<vec> sampleYs;
        vector<vec> fitYs;

        vector<mat> designMatrices;
        vec timeVec = timesInSeconds;

        mat all_y;
        mat all_dy;
        mat all_ddy;

        // retrieve all columns for different degrees of freedom
        vector<vec> trajectories;
        for(int i = 0; i < degFreedom; ++i) {

            vec trajectory = joints.col(i);
            trajectories.push_back(trajectory);

            vec vec_dy = computeDiscreteDerivatives(timeVec, trajectory);
            vec vec_ddy = computeDiscreteDerivatives(timeVec, vec_dy);

            all_y = join_rows(all_y, trajectory);
            all_dy = join_rows(all_dy, vec_dy);
            all_ddy = join_rows(all_ddy, vec_ddy);

        }

        vector<trajectory_learner_internal> dmpResAll = fitTrajectory(timeVec, all_y, all_dy, all_ddy);

        for(int i = 0; i < dmpResAll.size(); ++i) {

            trajectory_learner_internal dmpRes = dmpResAll.at(i);
            vec dmpCoeff = dmpRes.coeff;
            vec fity = dmpRes.fity;

            g(i) = (all_y.col(i))(dataPointsNum - 1);
            y0(i) = all_y.col(i)(0);
            dy0(i) = all_dy.col(i)(0);
            ddy0(i) = all_ddy.col(i)(0);
            dmpCoeffs.push_back(dmpCoeff);

            fitYs.push_back(fity);
            designMatrices.push_back(dmpRes.desMat);

        }

        for (int i = 0; i < all_y.n_cols; ++i) sampleYs.push_back(all_y.col(i));

        auto retVal = createDmpInstance(timeVec, sampleYs, fitYs, dmpCoeffs, dmpBase, designMatrices, tau, az, bz, ax);

        KUKADU_MODULE_END_USAGE();

        return retVal;

    }

    JointDMPLearner::JointDMPLearner(std::vector<DMPBase> dmpBase, double tau, double az, double bz, double ax, arma::vec timesInSeconds, arma::mat joints) : GeneralDmpLearner(dmpBase, tau, az, bz, ax, timesInSeconds, joints) {

    }

    JointDMPLearner::JointDMPLearner(std::vector<double> mysDef, std::vector<double> sigmasDef, double az, double bz, std::string file) : GeneralDmpLearner(mysDef, sigmasDef, az, bz, file) {

    }

    JointDMPLearner::JointDMPLearner(double az, double bz, std::string file) : GeneralDmpLearner(az, bz, file) {

    }

    JointDMPLearner::JointDMPLearner(StorageSingleton& storage, KUKADU_SHARED_PTR<ControlQueue> queue, double az, double bz, long long int startTime, long long int endTime) :
        GeneralDmpLearner(az, bz,
                          SensorStorage::loadSampleTimesInRangeFromDb(storage, queue, startTime, endTime),
                          SensorStorage::loadJointsFromDb(storage, queue, startTime, endTime)) {

    }

    JointDMPLearner::JointDMPLearner(double az, double bz, arma::vec& timesInSeconds, arma::mat joints) : GeneralDmpLearner(az, bz, timesInSeconds, joints) {

    }

    KUKADU_SHARED_PTR<Dmp> JointDMPLearner::createDmpInstance(arma::vec& supervisedTsInSeconds, std::vector<arma::vec> sampleYs, std::vector<arma::vec> fitYs, std::vector<arma::vec> dmpCoeffs, std::vector<DMPBase> dmpBase, std::vector<arma::mat> designMatrices,
                                           double tau, double az, double bz, double ax) {

        return make_shared<JointDmp>(convertTimesInSecondsToTimeInMilliseconds(supervisedTsInSeconds), sampleYs, fitYs, dmpCoeffs, dmpBase, designMatrices, tau, az, bz, ax);

    }

    /****************** private functions ******************************/

    void DMPExecutor::construct(KUKADU_SHARED_PTR<ControlQueue> execQueue, int suppressMessages) {

        if(execQueue)
            customCycleTime = execQueue->getCycleTime();

        executionDuration = -1;

        dmp = KUKADU_DYNAMIC_POINTER_CAST<Dmp>(getTrajectory());

        // max force safety is switched of
        doRollback = true;
        maxAllowedForce = DBL_MAX;
        maxXForce = DBL_MAX;
        maxYForce = DBL_MAX;
        maxZForce = DBL_MAX;
        executionRunning = false;

        rollbackTime = 1.0;

        this->isCartesian = dmp->isCartesian();
        this->controlQueue = execQueue;

        this->ac = 0.0;
        this->vecYs = arma::vec(1);

        this->dmpCoeffs = dmp->getDmpCoeffs();
        this->baseDef = dmp->getDmpBase();

        this->tau = dmp->getTau(); this->az = dmp->getAz(); this->bz = dmp->getBz(); this->ax = dmp->getAx(); this->gs = dmp->getG();
        this->y0s = this->currentJoints = dmp->getY0(); this->dy0s = dmp->getDy0(); this->ddy0s = dmp->getDdy0();
        this->trajGen = new DMPTrajectoryGenerator(this->baseDef, ax, tau);

        this->axDivTau = ax / tau;
        this->oneDivTau = 1 / tau;

        this->degofFreedom = y0s.n_elem;
        if(isCartesian)
            this->odeSystemSize = 3 * 3 + 1;
        else
            this->odeSystemSize = 2 * this->degofFreedom + 1;
        this->suppressMessages = suppressMessages;

        previousDesiredJoints = y0s;

        externalErrorUsing = 0;
        externalError = 0.0;
        t = 0.0;

        this->odeSystemSizeMinOne = odeSystemSize - 1;

        setExecutionMode(kukadu::TrajectoryExecutor::EXECUTE_ROBOT);

    }

    arma::mat CartesianDMPLearner::computeFitY(arma::vec& time, arma::mat &y, arma::mat &dy, arma::mat &ddy, arma::vec& vec_g) {

        KUKADU_MODULE_START_USAGE();

        // position
        mat retMat(y.n_cols - 1, y.n_rows);

        for(int i = 0; i < y.n_rows; ++i) {

            for(int j = 0; j < 3; ++j) {

                double yVal = y(i, j);
                double dyVal = dy(i, j);
                double ddyVal = ddy(i, j);
                retMat(j, i) = /*1 / (vec_g(j) - y(0, j)) */ tau * tau * ddyVal - az * (bz * (vec_g(j) - yVal) - tau * dyVal);

            }

        }

        // orientation
        arma::mat omega(y.n_rows, 3);
        arma::mat domega;
        arma::mat eta;
        arma::mat deta;

        for (int j = 0; j < y.n_rows - 1; ++j) {

            vec logL= log(tf::Quaternion(y(j + 1, 3), y(j + 1,  4), y(j + 1, 5), y(j + 1, 6)) * tf::Quaternion(y(j, 3), y(j, 4), y(j, 5), y(j, 6)).inverse());

            for (int i = 0; i < 3; i++)
                omega(j, i) = 2 * logL(i) / (time(1)-time(0));

            if (j == y.n_rows - 2)
                for (int i = 0; i < 3; i++)
                    omega(y.n_rows - 1, i) = 2 * logL(i) / (time(1)-time(0));

        }

        for(int i = 0; i < 3 ; ++i) {

            vec trajectory = omega.col(i);
            vec domegaV = computeDiscreteDerivatives(time, trajectory);
            domega = join_rows(domega, domegaV);

        }

        eta = tau * omega;
        deta = tau * domega;

        for (int i = 0; i < y.n_rows; ++i) {

            vec logL = log(tf::Quaternion(vec_g(3), vec_g(4), vec_g(5), vec_g(6)) * tf::Quaternion(y(i, 3), y(i, 4), y(i, 5), y(i, 6)).inverse());
            for (int j = 3; j < retMat.n_rows; ++j)
                retMat(j, i) = tau * deta (i, j - 3) - az * (bz * 2 * logL(j - 3) - eta(i, j - 3));

        }

        KUKADU_MODULE_END_USAGE();

        return retMat;

    }

    void DMPExecutor::runCheckMaxForces() {

        bool rollBack = false;
        ros::Rate pollingRate(50);

        while(executionRunning) {

            double currentForce = controlQueue->getAbsoluteCartForce();
            mes_result currentFrcTrq = controlQueue->getCurrentCartesianFrcTrq();

            if((currentForce > maxAllowedForce && maxAllowedForce != IGNORE_FORCE)
                    || (std::abs(currentFrcTrq.joints(0)) > maxXForce && maxXForce != IGNORE_FORCE)
                    || (std::abs(currentFrcTrq.joints(1)) > maxYForce && maxYForce != IGNORE_FORCE)
                    || (std::abs(currentFrcTrq.joints(2)) > maxZForce && maxZForce != IGNORE_FORCE)) {
                executionRunning = false;
                rollBack = true;
            }

            pollingRate.sleep();

        }

        while(!executionStoppingDone) {
            pollingRate.sleep();
        }

        // if maxforce event detected --> kill execution and roll back
        if(rollBack && doRollback) {

            cout << "(DMPExecutor) max force threshold exceeded - rolling back a bit and stopping execution" << endl;
            controlQueue->rollBack(rollbackTime);
            cout << "(DMPExecutor) rollback done" << endl;

        }

    }

    double DMPExecutor::addTerm(double t, const double* currentDesiredYs, int jointNumber, KUKADU_SHARED_PTR<ControlQueue> queue) {
        return 0.0;
    }

    int DMPExecutor::func(double t, const double* y, double* f, void* params) {

        if(!isCartesian) {

            // y' = z / tau
            // z' = 1 / tau * ( az * (bz * (g - y) - z) + f);
            // x' = -ax / tau * x

            for(int i = 0; i < odeSystemSizeMinOne; i = i + 2) {

                double yPlusOne = y[i + 1];
                int currentSystem = (int) (i / 2);
                f[i] = yPlusOne * oneDivTau;
                double g = gs(currentSystem);
                arma::vec currentCoeffs = dmpCoeffs.at(currentSystem);

                if(t <= (dmp->getTmax() - 1)) {

                    double addTerm = trajGen->evaluateByCoefficientsSingleNonExponential(y[odeSystemSizeMinOne], currentCoeffs);

                    double x = this->addTerm(t, y, currentSystem, controlQueue);

                    if(!std::isnan(x))
                        f[i + 1] = oneDivTau * (az * (bz * (g - y[i]) - yPlusOne) + addTerm) + x; // + this->addTerm(t, y, currentSystem, controlQueue);
                    else
                        f[i + 1] = oneDivTau * (az * (bz * (g - y[i]) - yPlusOne) + addTerm);

                } else {
                    f[i + 1] = oneDivTau * (az * (bz * (g - y[i]) - yPlusOne));
                }

            }

        } else {

            vec vecF0(3);
            vec vecExtAdd(3);

            for(int i = 0, dim = 0; i < odeSystemSizeMinOne; i = i + 3, ++dim) {

                arma::vec currentCoeffs = dmpCoeffs.at(dim + 3);
                vecF0(dim) = trajGen->evaluateByCoefficientsSingleNonExponential(y[odeSystemSizeMinOne], currentCoeffs);
                vecExtAdd(dim) =  this->addTerm(t, y, dim + 3, controlQueue);

            }

            if(t <= (dmp->getTmax() - 1))
                nextDEta = oneDivTau * (az * (2.0 * bz * log(qG * currentQ.inverse()) - currentEta) + vecF0);
            else
                nextDEta = oneDivTau * az * (2.0 * bz * log(qG * currentQ.inverse()) - currentEta);

            // cartesian position and orientation
            for(int i = 0; i < odeSystemSizeMinOne; i = i + 3) {

                double yPlusOne = y[i + 1];
                int currentSystem = (int) (i / 3);

                f[i] = yPlusOne * oneDivTau;
                double g = gs(currentSystem);
                arma::vec currentCoeffs = dmpCoeffs.at(currentSystem);

                if(t <= (dmp->getTmax() - 1)) {

                    double addTerm = trajGen->evaluateByCoefficientsSingleNonExponential(y[odeSystemSizeMinOne], currentCoeffs);
                    double x = this->addTerm(t, y, currentSystem, controlQueue);
                    if(!std::isnan(x))
                        f[i + 1] = oneDivTau * (az * (bz * (g - y[i]) - yPlusOne) + addTerm) + x; // + this->addTerm(t, y, currentSystem, controlQueue);
                    else
                        f[i + 1] = oneDivTau * (az * (bz * (g - y[i]) - yPlusOne) + addTerm);
                } else {
                    f[i + 1] = oneDivTau * (az * (bz * (g - y[i]) - yPlusOne));
                }

                f[i + 2] = nextDEta(currentSystem);

            }

        }

        if(getExecutionMode() == EXECUTE_ROBOT) {

            if(!isCartesian)
                currentJoints = controlQueue->getCurrentJoints().joints;
            else {
                geometry_msgs::Pose currentPose = controlQueue->getCurrentCartesianPose();
                currentJoints(0) = currentPose.position.x;
                currentJoints(1) = currentPose.position.y;
                currentJoints(2) = currentPose.position.x;
                arma::vec currentOrient = log(tf::Quaternion(currentPose.orientation.x, currentPose.orientation.y, currentPose.orientation.z, currentPose.orientation.w));
                for (int i = 0; i < 3; i++)
                    currentJoints(i + 3) = currentOrient(i);

            }

            double corrector = 0.0;
            if(!usesExternalError()) {

                // include original  phase stopping
                for(int i = 0; i < degofFreedom; ++i) previousDesiredJoints[i] =  y[2 * i];
                double dist = computeDistance(previousDesiredJoints, currentJoints);

                corrector = 1.0 + ac * dist;

            } else
                corrector = 1.0 + ac * getExternalError();

            f[odeSystemSizeMinOne] = -axDivTau * y[odeSystemSizeMinOne] / corrector;

        } else if(getExecutionMode() == SIMULATE_ROBOT) {

            // progress as usual
            f[odeSystemSizeMinOne] = - axDivTau * y[odeSystemSizeMinOne];

        }

        return GSL_SUCCESS;

    }

    double DMPExecutor::computeDistance(const arma::vec yDes, arma::vec yCurr) {

        if(!isCartesian) {
            // distance for quaternion has to be introduced if this is enabled
            arma::vec tmp = (yDes - yCurr).t() * (yDes - yCurr);
            return tmp(0);
        }
        else
            return 0;

    }

    void DMPExecutor::setAc(double ac) {
        this->ac = ac;
    }

    int DMPExecutor::jac(double t, const double* y, double *dfdy, double* dfdt, void* params) {

        // not implemented (not required for most of the ode solvers)
        return GSL_SUCCESS;

    }

    struct gsl_delete_expression {
        void operator()(gsl_odeiv2_driver* p) const {
            gsl_odeiv2_driver_free(p);
        }
    };

    void DMPExecutor::initializeIntegration(double tStart, double tolAbsErr, double tolRelErr) {

        t = tStart;

        initializeIntegrationQuat();

        vec_y.clear();
        double ys[odeSystemSize];
        vecYs = vec(odeSystemSize);

        if(!isCartesian) {
            for(int i = 0; i < (odeSystemSize - 1); i = i + 2) {
                int iHalf = (int) i / 2;
                ys[i + 0] = y0s((int) iHalf);
                ys[i + 1] = tau * dy0s((int) iHalf);
            }
        } else {
            for(int i = 0, dim = 0; i < (odeSystemSize - 3); i = i + 3, ++dim) {
                ys[i + 0] = y0s(dim);
                ys[i + 1] = tau * dy0s(dim);
                ys[i + 2] = Eta0(dim);
            }
        }

        ys[odeSystemSize - 1] = 1;

        gsl_odeiv2_system tmp_sys = {static_func, NULL, odeSystemSize, this};
        sys = tmp_sys;
        d = KUKADU_SHARED_PTR<gsl_odeiv2_driver>(gsl_odeiv2_driver_alloc_y_new(&sys, gsl_odeiv2_step_rkf45, customCycleTime, tolAbsErr, tolRelErr), gsl_delete_expression());

        for(int i = 0; i < odeSystemSize; ++i)
            vecYs(i) = ys[i];

    }

    void DMPExecutor::initializeIntegrationQuat() {

        if(t > 0.0) {
            throw KukaduException("(DMPExecutor) t > 0 is not considered yet with cartesian dmp");
        }

        if(isCartesian) {

            KUKADU_SHARED_PTR<CartesianDMP> cartDmp = KUKADU_DYNAMIC_POINTER_CAST<CartesianDMP>(dmp);
            double firstDt = cartDmp->getDeltaTByIdx(0);
            vec eta0 = cartDmp->getEta0();
            vec eta1 = cartDmp->getEtaByIdx(1);

            tf::Quaternion q0 = cartDmp->getQ0();
            currentQ = q0;
            currentEta = eta0;
            nextEta = eta0;

            //dQ0 = oneDivTau * 0.5 * eta0Quat * q0;

            dEta0 = 1.0 / firstDt * (eta1 - eta0);
            Eta0 = eta0;
            nextDEta = dEta0;

            qG = cartDmp->getQg();

        }

    }

    arma::vec DMPExecutor::doIntegrationStep(double ac) {

        double ys[odeSystemSize];

        for(int i = 0; i < odeSystemSize; ++i)
            ys[i] = vecYs(i);

        arma::vec retJoints(degofFreedom);
        retJoints.fill(0.0);

        if(t < dmp->getTmax()) {

            int s = gsl_odeiv2_driver_apply_fixed_step(d.get(), &t, customCycleTime, 1, ys);

            if (s != GSL_SUCCESS) {
                cout << "(DMPExecutor) error: driver returned " << s << endl;
                throw KukaduException(string("(DMPExecutor) error: driver returned " + s).c_str());
            }

        }

        if(!isCartesian) {

            for(int i = 0; i < degofFreedom; ++i)
                retJoints(i) = ys[2 * i];

        } else {


            for(int i = 0; i < 3; ++i) {
                retJoints(i) = ys[3 * i];
                nextEta(i) = ys[3 * i + 2];
            }


            vec alteredCurrentEta(3);
            alteredCurrentEta = customCycleTime / 2.0 * oneDivTau * nextEta;

            currentQ = vecExp(alteredCurrentEta) * currentQ;

            retJoints(3) = currentQ.getX(); retJoints(4) = currentQ.getY(); retJoints(5) = currentQ.getZ(); retJoints(6) = currentQ.getW();
            currentEta = nextEta;

        }

        for(int i = 0; i < odeSystemSize; ++i)
            vecYs(i) = ys[i];

        return retJoints;

    }

    void DMPExecutor::destroyIntegration() {

        d = KUKADU_SHARED_PTR<gsl_odeiv2_driver>();

    }

    int DMPExecutor::static_func(double t, const double y[], double f[], void *params) {
        return ((DMPExecutor*)params)->func(t, y, f, NULL);
    }

    int DMPExecutor::static_jac (double t, const double y[], double *dfdy, double dfdt[], void *params) {
        return ((DMPExecutor*)params)->jac(t, y, dfdy, dfdt, NULL);
    }

    double DMPTrajectoryGenerator::computeNormalization(double x) {

        int myssize = baseDef.size();

        if(previousX != x) {

            double normVal = 0.0;

            for(int i = 0; i < myssize; ++i) {
                double my = baseDef.at(i).getMy();
                vector<double> currentSigmas = baseDef.at(i).getSigmas();
                for(int j = 0; j < currentSigmas.size(); ++j) {
                    double sigma = currentSigmas.at(j);
                    normVal += exp(  -pow(x - my, 2) / (2 * pow(sigma, 2))   );
                }
            }
            previousNorm = normVal;
        }

        return previousNorm;

    }

    void GeneralDmpLearner::construct(std::vector<DMPBase> dmpBase, double tau, double az, double bz, double ax, arma::vec timesInSeconds, arma::mat joints, int degFreedom) {

        double minTimelyDist = 0.2;
        double prevStoredTime = timesInSeconds(0);
        vector<int> filteredIdx{0};
        for(int i = 1; i < timesInSeconds.n_elem; ++i)
            if((timesInSeconds(i) - prevStoredTime) >= minTimelyDist) {
                filteredIdx.push_back(i);
                prevStoredTime = timesInSeconds(i);
            }

        vec filteredTimesInSeconds(filteredIdx.size());
        mat filteredJoints(filteredIdx.size(), joints.n_cols);
        for(int i = 0; i < filteredTimesInSeconds.size(); ++i) {
            filteredTimesInSeconds(i) = timesInSeconds(filteredIdx.at(i));
            for(int j = 0; j < joints.n_cols; ++j)
                    filteredJoints(i, j) = joints(filteredIdx.at(i), j);
        }

        this->dmpBase = dmpBase;
        this->tau = tau; this->az = az; this->bz = bz; this->ax = ax;

        this->joints = filteredJoints;
        this->timesInSeconds = filteredTimesInSeconds;

        this->degFreedom = degFreedom;

    }

    std::vector<trajectory_learner_internal> GeneralDmpLearner::fitTrajectory(vec time, mat y, mat dy, mat ddy) {

        int dataPointsNum = time.n_elem;
        vec vec_g = y.row(dataPointsNum - 1).t();

        mat fity = computeFitY(time, y, dy, ddy, vec_g);

        vector<trajectory_learner_internal> retVec;

        for(int i = 0; i < fity.n_rows; ++i) {

            trajectory_learner_internal ret;
            DMPTrajectoryGenerator dmpTrajGen(dmpBase, ax, tau);
            GeneralFitter dmpGenFit(time, fity.row(i).t(), dataPointsNum, &dmpTrajGen);
            mat designMatrix = dmpGenFit.computeDesignMatrix();
            vec dmpCoeff = dmpGenFit.computeLinFitCoefficients(designMatrix);

            ret.fity = fity.row(i).t();
            ret.coeff = dmpCoeff;
            ret.desMat = designMatrix;

            retVec.push_back(ret);

        }

        return retVec;

    }

    arma::mat JointDMPLearner::computeFitY(arma::vec& timeInSec, arma::mat &y, arma::mat &dy, arma::mat &ddy, arma::vec& vec_g) {

        KUKADU_MODULE_START_USAGE();

        mat retMat(y.n_cols, y.n_rows);
        for(int i = 0; i < y.n_rows; ++i) {

            for(int j = 0; j < y.n_cols; ++j) {

                double yVal = y(i, j);
                double dyVal = dy(i, j);
                double ddyVal = ddy(i, j);
                retMat(j, i) = tau * tau * ddyVal - az * (bz * (vec_g(j) - yVal) - tau * dyVal);

            }

        }

        KUKADU_MODULE_END_USAGE();

        return retMat;

    }

    /****************** end ********************************************/

}
