#ifndef KUKADU_TRAJECTORY_H
#define KUKADU_TRAJECTORY_H

#include <armadillo>
#include <kukadu/robot/queue.hpp>
#include <kukadu/types/kukadutypes.hpp>
#include <kukadu/control/controller.hpp>

namespace kukadu {

    class Trajectory {

    private:

    public:

        Trajectory();
        Trajectory(const Trajectory& copy);

        virtual int getDegreesOfFreedom() const = 0;
        virtual arma::vec getStartingPos() = 0;

        virtual std::vector<arma::vec> getCoefficients() = 0;
        virtual void setCoefficients(std::vector<arma::vec> coeffs) = 0;

        int operator==(Trajectory const& comp) const;

        virtual KUKADU_SHARED_PTR<Trajectory> copy() = 0;

        virtual double getTmax() = 0;
        virtual void setTmax(double tmax) = 0;

    };

    class SingleSampleTrajectory : public Trajectory {

    private:

    protected:

        std::vector<arma::vec> sampleYs;
        arma::vec supervisedTs;

    public:

        SingleSampleTrajectory(arma::vec supervisedTs, std::vector<arma::vec> sampleYs);
        SingleSampleTrajectory(const SingleSampleTrajectory& copy);
        SingleSampleTrajectory();

        int getDegreesOfFreedom() const;
        int getDataPointsNum();

        double getT(int ptIdx);
        double getDataPoint(int freedomIdx, int ptIdx);

        void setSupervisedTs(arma::vec supervisedTs);
        void setSampleYs(std::vector<arma::vec> sampleYs);

        arma::vec getStartingPos();
        arma::vec getSupervisedTs();
        arma::vec getSampleYByIndex(int idx);
        std::vector<arma::vec> getSampleYs();

        virtual std::vector<arma::vec> getCoefficients() = 0;
        virtual void setCoefficients(std::vector<arma::vec> coeffs) = 0;

        int operator==(SingleSampleTrajectory const& comp) const;

    };

    /** \brief The TrajectoryGenerator defines an interface to define basis functions for linear regression (see GeneralFitter)
     *
     * An implementation of this class has to define an internal index on the basis functions where each basis function value
     * can be computed by setting the basis function index
     * \ingroup ControlPolicyFramework
     */
    class TrajectoryGenerator {

    private:

    public:

        /**
         * \brief constructor
         */
        TrajectoryGenerator();

        /**
         * \brief evaluates the value of a single basis function
         * \param x value, where the basis function should be evaluated
         * \param fun basis function index that specifies the basis function
         */
        virtual double evaluateBasisFunction(double x, int fun) = 0;

        /**
         * \brief evaluates the linear combination of basis functions by defining the coefficients for a single value x
         * \param x value, where the basis functions should be evaluated
         * \param coeff coefficients that have to be used for computing the linear combination
         */
        virtual double evaluateByCoefficientsSingle(double x, arma::vec coeff) = 0;

        /**
         * \brief performs the same as evaluateByCoefficientsSingle, but for multiple values of x
         * \param x vector of evaluation points
         * \param sampleCount size of vector x
         * \param coeff coefficients that have to be used for computing the linear combination
         */
        virtual arma::vec evaluateByCoefficientsMultiple(arma::vec x, int sampleCount, arma::vec coeff) = 0;

        /**
         * \brief returns the number of basis functions
         */
        virtual int getBasisFunctionCount() = 0;

        /**
         * \brief returns the name of the basis function system as a string
         */
        virtual std::string getTrajectoryType() = 0;

    };

    /** \brief Implements the TrajectoryGenerator interface for polynomials
     *
     * This class provides simple polynomials as basis functions. f(x) = sum c_i x^i
     * \ingroup ControlPolicyFramework
     */
    class PolyTrajectoryGenerator : public TrajectoryGenerator {

    private:

        int basisFunctionCount;

    public:

        /**
         * \brief constructor. the polynomials are defined by giving the degree of the polynomial
         * \param basisFunctionCount polynomial degree
         */
        PolyTrajectoryGenerator(int basisFunctionCount);

        double evaluateBasisFunction(double x, int fun);
        double evaluateByCoefficientsSingle(double x, arma::vec coeff);
        arma::vec evaluateByCoefficientsMultiple(arma::vec x, int sampleCount, arma::vec coeff);

        int getBasisFunctionCount();

        std::string getTrajectoryType();

    };

    class TrajectoryExecutor : public Controller {

    public:

        enum execution_modes{SIMULATE_ROBOT, EXECUTE_ROBOT};

    private:

        double tEnd;
        double tStart;
        enum execution_modes executionMode;

        KUKADU_SHARED_PTR<Trajectory> trajectory;

    protected:

        virtual KUKADU_SHARED_PTR<ControllerResult> executeInternal();
        virtual KUKADU_SHARED_PTR<ControllerResult> executeTrajectory() = 0;
        virtual KUKADU_SHARED_PTR<ControllerResult> simulateTrajectory() = 0;

    public:

        TrajectoryExecutor(StorageSingleton& dbStorage, KUKADU_SHARED_PTR<ControlQueue> usedQueue, KUKADU_SHARED_PTR<Trajectory> trajectory);

        void setTEnd(double tEndInSeconds);
        void setTStart(double tStartInSeconds);
        void setExecutionMode(enum execution_modes mode);

        double getTEnd();
        double getTStart();
        TrajectoryExecutor::execution_modes getExecutionMode();

        virtual KUKADU_SHARED_PTR<Trajectory> getTrajectory();

        virtual void setTrajectory(KUKADU_SHARED_PTR<Trajectory> traj) = 0;

        virtual std::string getClassName() { return "TrajectoryExecutor"; }

    };

    class TrajectoryComparator {

    public:

        virtual double computeDistance() = 0;

    };

}

#endif
