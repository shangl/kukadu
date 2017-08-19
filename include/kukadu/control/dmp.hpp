#ifndef KUKADU_DMP_CONTROLLER_H
#define KUKADU_DMP_CONTROLLER_H

#include <vector>
#include <armadillo>
#include <kukadu/utils/types.hpp>
#include <kukadu/robot/queue.hpp>
#include <kukadu/control/trajectory.hpp>
#include <kukadu/learning/regression/fitting.hpp>

#include <tf/tf.h>
#include <gsl/gsl_odeiv2.h>

namespace kukadu {

    class DMPBase {

    private:

        float my;
        std::vector<double> sigmas;

    public:

        DMPBase();

        DMPBase(float my, std::vector<double> sigmas);

        float getMy();

        std::vector<double> getSigmas();

        int operator==(DMPBase const &comp) const;

    };

    class Dmp : public SingleSampleTrajectory {

    private:

        std::vector<arma::vec> dmpCoeffs;
        arma::vec y0;
        arma::vec dy0;
        arma::vec ddy0;
        arma::vec g;

        std::vector<DMPBase> dmpBase;
        std::vector<arma::mat> designMatrices;

        double az;
        double bz;
        double ax;
        double ac;
        double tau;
        double tmax;
        double tolAbsErr;
        double tolRelErr;

        void initializeG();

        void initializeY0();

        void initializeDy0();

        void initializeDdy0();

        void construct(std::vector<arma::vec> &fitYs, std::vector<arma::vec> &dmpCoeffs, std::vector<DMPBase> &dmpBase,
                       std::vector<arma::mat> &designMatrices,
                       double tau, double az, double bz, double ax, double ac, double dmpStepSize, double tolAbsErr,
                       double tolRelErr);

    protected:

        double dmpStepSize;
        std::vector<arma::vec> fitYs;

        Dmp();

    public:

        Dmp(const Dmp &copy);

        Dmp(std::string dmpFile);

        Dmp(std::vector<arma::vec> dmpCoeffs, std::vector<DMPBase> dmpBase, arma::vec y0, arma::vec dy0, arma::vec g,
            double tMax, double tau, double az, double bz, double ax, double ac, double dmpStepSize, double tolAbsErr,
            double tolRelErr);

        Dmp(std::vector<long long int> supervisedTs, std::vector<arma::vec> sampleYs, std::vector<arma::vec> fitYs,
            std::vector<arma::vec> dmpCoeffs, std::vector<DMPBase> dmpBase, std::vector<arma::mat> designMatrices,
            double tau, double az, double bz, double ax, double ac, double dmpStepSize, double tolAbsErr,
            double tolRelErr);

        Dmp(std::vector<long long int> supervisedTs, std::vector<arma::vec> sampleYs, std::vector<arma::vec> fitYs,
            std::vector<arma::vec> dmpCoeffs, std::vector<DMPBase> dmpBase, std::vector<arma::mat> designMatrices,
            double tau, double az, double bz, double ax);

        void storeDatabase(std::string skillLabel, bool overwriteIfExists = false);

        void serialize(std::string dmpFile);

        int getSampleCount();

        double getY0(int freedomIdx);

        double getDy0(int freedomIdx);

        double getDdy0(int freedomIdx);

        double getG(int freedomIdx);

        arma::vec getY0();

        arma::vec getDy0();

        arma::vec getDdy0();

        arma::vec getG();

        std::vector<arma::vec> getCoefficients();

        void setCoefficients(std::vector<arma::vec> coeffs);

        std::vector<arma::vec> getDmpCoeffs();

        std::vector<arma::vec> getFitYs();

        void setDmpCoeffs(std::vector<arma::vec> coeffs);

        arma::vec getDmpCoeffs(int freedomIdx);

        arma::mat getDesignMatrix(int freedomIdx);

        int getDesignMatrixCount();

        std::vector<DMPBase> getDmpBase();

        double getDeltaTByIdx(int idx);

        double getTau();

        double getAz();

        double getBz();

        double getAx();

        double getStepSize();

        double getTolAbsErr();

        double getTolRelErr();

        double getTmax();

        void setTmax(double tmax);

        virtual bool isCartesian() = 0;

        int operator==(KUKADU_SHARED_PTR<Dmp> const &comp) const;

    };

    class JointDmp : public Dmp {

    public:

        JointDmp(std::vector<long long int> supervisedTsInMilliseconds, std::vector<arma::vec> sampleYs,
                 std::vector<arma::vec> fitYs, std::vector<arma::vec> dmpCoeffs, std::vector<DMPBase> dmpBase,
                 std::vector<arma::mat> designMatrices,
                 double tau, double az, double bz, double ax, double ac, double dmpStepSize, double tolAbsErr,
                 double tolRelErr);

        JointDmp(std::vector<long long int> supervisedTsInMilliseconds, std::vector<arma::vec> sampleYs,
                 std::vector<arma::vec> fitYs, std::vector<arma::vec> dmpCoeffs, std::vector<DMPBase> dmpBase,
                 std::vector<arma::mat> designMatrices,
                 double tau, double az, double bz, double ax);

        JointDmp(std::vector<arma::vec> dmpCoeffs, std::vector<DMPBase> dmpBase, arma::vec y0, arma::vec dy0,
                 arma::vec g, double tMax, double tau, double az, double bz, double ax, double ac, double dmpStepSize,
                 double tolAbsErr, double tolRelErr);

        JointDmp(std::string dmpFile);

        JointDmp();

        bool isCartesian();

        virtual KUKADU_SHARED_PTR<Trajectory> copy();

    };

    class CartesianDMP : public Dmp {

    public:

        CartesianDMP(std::vector<long long int> supervisedTs, std::vector<arma::vec> sampleYs,
                     std::vector<arma::vec> fitYs, std::vector<arma::vec> dmpCoeffs, std::vector<DMPBase> dmpBase,
                     std::vector<arma::mat> designMatrices,
                     double tau, double az, double bz, double ax, double ac, double dmpStepSize, double tolAbsErr,
                     double tolRelErr);

        CartesianDMP(std::vector<long long int> supervisedTs, std::vector<arma::vec> sampleYs,
                     std::vector<arma::vec> fitYs, std::vector<arma::vec> dmpCoeffs, std::vector<DMPBase> dmpBase,
                     std::vector<arma::mat> designMatrices,
                     double tau, double az, double bz, double ax);

        CartesianDMP(std::string dmpFile);

        CartesianDMP();

        bool isCartesian();

        tf::Quaternion getQ0();

        tf::Quaternion getQg();

        tf::Quaternion getQByIdx(int idx);

        arma::vec getEta0();

        arma::vec getEtaByIdx(int idx);

        KUKADU_SHARED_PTR<Trajectory> copy();

    };

    /** \brief Implements the TrajectoryGenerator interface for dynamic movement primitives
     *
     * This class provides the basis functions for learning dynamic movement primitives with linear regression (see papers on dmps)
     * \ingroup ControlPolicyFramework
     */
    class DMPTrajectoryGenerator : public TrajectoryGenerator {

    private:


        std::vector<DMPBase> baseDef;

        double ax, tau;
        // for optimization
        double previousX;
        arma::vec prevBasFun;
        double previousNorm;

        int baseFunctionCount;
        int myssize;
        int sigmassize;

        double computeNormalization(double x);

    public:

        /**
         * \brief constructor
         * \param baseDef defines the basis functions by setting the means and variances of the Gaussian basis functions
         * \param ax dmp constant
         * \param tau dmp time constant
         */
        DMPTrajectoryGenerator(std::vector<DMPBase> baseDef, double ax, double tau);

        double evaluateBasisFunction(double x, int fun);

        /**
         * \brief computes the basis function value by setting x = e^(-ax / tau * x)
         * \param x position to evaluate
         * \param fun basis function index
         */
        double evaluateBasisFunctionNonExponential(double x, int fun);

        double evaluateByCoefficientsSingle(double x, arma::vec coeff);

        /**
         * \brief computes the linear combination of basis functions values by setting x = e^(-ax / tau * x)
         * \param x position to evaluate
         * \param coeff vector of basis function coefficients
         */
        double evaluateByCoefficientsSingleNonExponential(double x, arma::vec coeff);

        arma::vec evaluateByCoefficientsMultiple(arma::vec x, int sampleCount, arma::vec coeff);

        /**
         * \brief computes the linear combination of basis functions values for multiple positions by setting x = e^(-ax / tau * x)
         * \param x vector of positions to evaluate
         * \param sampleCount size of vector x
         * \param coeff vector of basis function coefficients
         */
        arma::vec evaluateByCoefficientsMultipleNonExponential(arma::vec x, int sampleCount, arma::vec coeff);

        int getBasisFunctionCount();

        std::string getTrajectoryType();

    };

    class GeneralDmpLearner {

    private:

        int degFreedom;

        arma::vec timesInSeconds;
        arma::mat joints;

        std::vector<DMPBase> dmpBase;

        void
        construct(std::vector<DMPBase> dmpBase, double tau, double az, double bz, double ax, arma::vec timesInSeconds,
                  arma::mat joints, int degFreedom);

        std::vector<trajectory_learner_internal>
        fitTrajectory(arma::vec time, arma::mat y, arma::mat dy, arma::mat ddy);

    protected:

        double az;
        double bz;
        double ax;
        double tau;

        virtual KUKADU_SHARED_PTR<Dmp>
        createDmpInstance(arma::vec &supervisedTsInSeconds, std::vector<arma::vec> sampleYs,
                          std::vector<arma::vec> fitYs, std::vector<arma::vec> dmpCoeffs, std::vector<DMPBase> dmpBase,
                          std::vector<arma::mat> designMatrices,
                          double tau, double az, double bz, double ax) = 0;

        virtual arma::mat
        computeFitY(arma::vec &time, arma::mat &y, arma::mat &dy, arma::mat &ddy, arma::vec &vec_g) = 0;

    public:

        /**
         * \brief constructor
         * \param dmpBase dmp basis function definition
         * \param tau dmp timing constant
         * \param az dmp az constant
         * \param bz dmp bz constant
         * \param ax dmp ax constant
         * \param joints measured joints
         * \param degFreedom robots degrees of freedom
         */
        GeneralDmpLearner(std::vector<DMPBase> dmpBase, double tau, double az, double bz, double ax,
                          arma::vec timesInSeconds, arma::mat joints);

        /**
         * \brief constructor
         * \param dmpBase dmp basis function definition
         * \param tau dmp timing constant
         * \param az dmp az constant
         * \param bz dmp bz constant
         * \param ax dmp ax constant
         * \param file file containing the measured joints
         * \param degFreedom robots degrees of freedom
         */
        GeneralDmpLearner(std::vector<double> mysDef, std::vector<double> sigmasDef, double az, double bz,
                          std::string file);

        GeneralDmpLearner(double az, double bz, std::string file);

        GeneralDmpLearner(double az, double bz, arma::vec timesInSeconds, arma::mat joints);

        /**
         * \brief fit the specified trajectories
         */
        KUKADU_SHARED_PTR<Dmp> fitTrajectories();

    };

    /** \brief The TrajectoryDMPLearner encapsulates the dmp learning process
     *
     * Dynamic movement primitives can be easily learned by using this class and providing the joint data or a file containing this data. Basically this is a helper
     * that enables the programmer to reduce code complexity.
     * \ingroup ControlPolicyFramework
     */
    class JointDMPLearner : public GeneralDmpLearner {

    protected:

        KUKADU_SHARED_PTR<Dmp> createDmpInstance(arma::vec &supervisedTsInSeconds, std::vector<arma::vec> sampleYs,
                                                 std::vector<arma::vec> fitYs, std::vector<arma::vec> dmpCoeffs,
                                                 std::vector<DMPBase> dmpBase, std::vector<arma::mat> designMatrices,
                                                 double tau, double az, double bz, double ax);

        arma::mat computeFitY(arma::vec &timeInSec, arma::mat &y, arma::mat &dy, arma::mat &ddy, arma::vec &vec_g);

    public:

        /**
         * \brief constructor
         * \param dmpBase dmp basis function definition
         * \param tau dmp timing constant
         * \param az dmp az constant
         * \param bz dmp bz constant
         * \param ax dmp ax constant
         * \param joints measured joints
         * \param degFreedom robots degrees of freedom
         */
        JointDMPLearner(std::vector<DMPBase> dmpBase, double tau, double az, double bz, double ax,
                        arma::vec timesInSeconds, arma::mat joints);

        /**
         * \brief constructor
         * \param dmpBase dmp basis function definition
         * \param tau dmp timing constant
         * \param az dmp az constant
         * \param bz dmp bz constant
         * \param ax dmp ax constant
         * \param file file containing the measured joints
         * \param degFreedom robots degrees of freedom
         */
        JointDMPLearner(std::vector<double> mysDef, std::vector<double> sigmasDef, double az, double bz,
                        std::string file);

        JointDMPLearner(double az, double bz, std::string file);

        JointDMPLearner(StorageSingleton &storage, KUKADU_SHARED_PTR<ControlQueue> queue, double az, double bz,
                        long long int startTime, long long int endTime);

        JointDMPLearner(double az, double bz, arma::vec &timesInSeconds, arma::mat joints);

    };

    /** \brief The TrajectoryDMPLearner encapsulates the dmp learning process
     *
     * Dynamic movement primitives can be easily learned by using this class and providing the cartesian data or a file containing this data. Basically this is a helper
     * that enables the programmer to reduce code complexity.
     * \ingroup ControlPolicyFramework
     */
    class CartesianDMPLearner : public GeneralDmpLearner {

    protected:

        KUKADU_SHARED_PTR<Dmp> createDmpInstance(arma::vec &supervisedTsInSeconds, std::vector<arma::vec> sampleYs,
                                                 std::vector<arma::vec> fitYs, std::vector<arma::vec> dmpCoeffs,
                                                 std::vector<DMPBase> dmpBase, std::vector<arma::mat> designMatrices,
                                                 double tau, double az, double bz, double ax);

        arma::mat computeFitY(arma::vec &time, arma::mat &y, arma::mat &dy, arma::mat &ddy, arma::vec &vec_g);

    public:

        /**
         * \brief constructor
         * \param dmpBase dmp basis function definition
         * \param tau dmp timing constant
         * \param az dmp az constant
         * \param bz dmp bz constant
         * \param ax dmp ax constant
         * \param joints measured joints
         * \param degFreedom robots degrees of freedom
         */
        CartesianDMPLearner(std::vector<DMPBase> dmpBase, double tau, double az, double bz, double ax,
                            arma::vec timesInSeconds, arma::mat joints);

        /**
         * \brief constructor
         * \param dmpBase dmp basis function definition
         * \param tau dmp timing constant
         * \param az dmp az constant
         * \param bz dmp bz constant
         * \param ax dmp ax constant
         * \param file file containing the measured joints
         * \param degFreedom robots degrees of freedom
         */
        CartesianDMPLearner(std::vector<double> mysDef, std::vector<double> sigmasDef, double az, double bz,
                            std::string file);

        CartesianDMPLearner(double az, double bz, std::string file);

        CartesianDMPLearner(double az, double bz, arma::vec timesInSeconds, arma::mat joints);

    };

    /** \brief This class is responsible for dmp execution
     *
     * The DMPExecutor computes the evolution of the dynmic movement primitives by using a numerical differential equation solver. It provides execution and simulation mode.
     * If the execution mode is selected, a control queue has to be passed to constructor.
     * \ingroup ControlPolicyFramework
     */
    class DMPExecutor : public TrajectoryExecutor {

    protected:

        bool doRollback;
        bool isCartesian;
        bool executionRunning;
        bool executionStoppingDone;

        int degofFreedom;
        int suppressMessages;
        int externalErrorUsing;
        int odeSystemSizeMinOne;

        long unsigned int odeSystemSize;

        double t;
        double az;
        double bz;
        double ax;
        double ac;
        double tau;
        // for optimization
        double axDivTau;
        double oneDivTau;
        double externalError;
        double maxAllowedForce;

        double maxXForce;
        double maxYForce;
        double maxZForce;
        double rollbackTime;

        double customCycleTime;
        double executionDuration;

        arma::vec gs;
        arma::vec y0s;
        arma::vec dy0s;
        arma::vec Eta0;
        arma::vec ddy0s;
        arma::vec vecYs;
        arma::vec dEta0;
        arma::vec nextEta;
        arma::vec nextDEta;
        arma::vec currentEta;
        arma::vec currentJoints;
        arma::vec previousDesiredJoints;

        std::vector<DMPBase> baseDef;
        std::vector<arma::vec> dmpCoeffs;

        DMPTrajectoryGenerator *trajGen;

        KUKADU_SHARED_PTR<Dmp> dmp;
        KUKADU_SHARED_PTR<gsl_odeiv2_driver> d;
        KUKADU_SHARED_PTR<ControlQueue> controlQueue;
        KUKADU_SHARED_PTR<kukadu_thread> maxFrcThread;

        std::vector<double> vec_t;
        std::vector<double> vec_y;
        std::vector<double> internalClock;

        gsl_odeiv2_system sys;

        tf::Quaternion qG;
        tf::Quaternion dQ0;
        tf::Quaternion nextQ;
        tf::Quaternion currentQ;

        void runCheckMaxForces();

        // needed for workaround (see here http://stackoverflow.com/questions/10687397/static-virtual-workaround-in-gsl)
        static int static_func(double t, const double y[], double f[], void *params);

        static int static_jac(double t, const double y[], double *dfdy, double dfdt[], void *params);

        double computeDistance(const arma::vec yDes, arma::vec yCurr);

        KUKADU_SHARED_PTR<ControllerResult> executeDMP(double tStart, double tEnd, double tolAbsErr, double tolRelErr);

        void construct(KUKADU_SHARED_PTR<ControlQueue> execQueue, int suppressMessages);

        KUKADU_SHARED_PTR<Dmp>
        loadDmpFromDb(StorageSingleton &dbStorage, int skillId, KUKADU_SHARED_PTR<ControlQueue> execQueue);

    protected:

        virtual int func(double t, const double *y, double *f, void *params);

        virtual int jac(double t, const double *y, double *dfdy, double *dfdt, void *params);

        virtual double
        addTerm(double t, const double *currentDesiredYs, int jointNumber, KUKADU_SHARED_PTR<ControlQueue> queue);

        KUKADU_SHARED_PTR<ControllerResult> executeTrajectory();

        KUKADU_SHARED_PTR<ControllerResult> simulateTrajectory();

        KUKADU_SHARED_PTR<ControllerResult>
        simulateTrajectory(double tStart, double tEnd, double tolAbsErr, double tolRelErr);

        virtual bool requiresGraspInternal();

        virtual bool producesGraspInternal();

        virtual void createSkillFromThisInternal(std::string skillName);

    public:

        /**
         * \brief constructor
         * \param dmp the dmp that should be executed
         */
        DMPExecutor(StorageSingleton &dbStorage, KUKADU_SHARED_PTR<Dmp> dmp, KUKADU_SHARED_PTR<ControlQueue> execQueue,
                    int suppressMessages = 1);

        DMPExecutor(StorageSingleton &dbStorage, int skillId, KUKADU_SHARED_PTR<ControlQueue> execQueue,
                    int suppressMessages = 1);

        int usesExternalError();

        void destroyIntegration();

        void initializeIntegrationQuat();

        void useExternalError(int external);

        void setExternalError(double error);

        // for now, only works in joint mode
        void enableMaxForceMode(double maxAbsForce, double maxXForce, double maxYForce, double maxZForce);

        void setExecutionMode(enum execution_modes mode);

        void setDoRollBackOnMaxForceEvent(bool doRollBackOnMaxForceEvent);

        void doRollBackOnMaxForceEvent(bool doRollback);

        void setTrajectory(KUKADU_SHARED_PTR<Trajectory> traj);

        void initializeIntegration(double tStart, double tolAbsErr, double tolRelErr);

        void setRollbackTime(double rollbackTime);

        void setExecutionDuration(double tmax);

        KUKADU_SHARED_PTR<Dmp> getDmp();

        void setAc(double ac);

        double getExternalError();

        void setCycleTime(double cycleTime);

        arma::vec doIntegrationStep(double ac);

        virtual std::string getClassName() { return "DMPExecutor"; }

        virtual std::pair<bool, std::string> getAugmentedInfoTableName();

        void setMaxAbsForce(double maxAbsForce);
        void setMaxXForce(double maxXForce);
        void setMaxYForce(double maxYForce);
        void setMaxZForce(double maxZForce);

        static const int KUKADU_EXEC_JOINT = 1;
        static const int KUKADU_EXEC_CART = 2;

        static const int IGNORE_FORCE = -1;

    };

}

#endif
