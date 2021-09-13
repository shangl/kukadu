#ifndef KUKADU_GENERALIZER_H
#define KUKADU_GENERALIZER_H

#include <armadillo>
#include <kukadu/utils/types.hpp>
#include <kukadu/control/dmp.hpp>
#include <kukadu/types/kukadutypes.hpp>
#include <kukadu/learning/regression/kernel.hpp>

namespace kukadu {

    class QueryPoint {

    private:

        std::string fileDmpPath;
        std::string fileDataPath;
        std::string fileQueryPath;

        arma::vec queryPoint;

        KUKADU_SHARED_PTR<Dmp> internalDmp;

    public:

        QueryPoint(const QueryPoint& qp);
        QueryPoint(std::string fileQueryPath, std::string fileDataPath, std::string fileDmpPath, KUKADU_SHARED_PTR<Dmp> dmp, arma::vec queryPoint);

        void setDmp(KUKADU_SHARED_PTR<Dmp> dmp);
        void setQueryPoint(arma::vec queryPoint);

        std::string getFileDmpPath();
        std::string getFileDataPath();
        std::string getFileQueryPath();

        arma::vec getQueryPoint();

        KUKADU_SHARED_PTR<Dmp> getDmp();


    };

    class DictionaryTrajectory : public Trajectory {

    private:

        int degOfFreedom;
        std::string baseFolder;

        std::vector<std::string> files;
        std::vector<std::string> queryFiles;
        std::vector<std::string> dmpFiles;
        std::vector<std::string> trajFiles;
        std::vector<QueryPoint> queryPoints;
        std::vector<arma::vec> coefficients;

        arma::vec startingPos;

        std::vector<QueryPoint> mapFiles(std::vector<std::string> queryFiles, std::vector<std::string> trajFiles, std::string prefix1, std::string prefix2);
        std::vector<QueryPoint> mapFiles(std::vector<std::string> queryFiles, std::vector<std::string> trajFiles, std::vector<std::string> dmpFiles, std::string prefix1, std::string prefix2, std::string prefix3);

    public:

        DictionaryTrajectory();
        DictionaryTrajectory(const DictionaryTrajectory& copy);
        DictionaryTrajectory(std::string baseFolder, double az, double bz);

        void setTmax(double tmax);
        void setCoefficients(std::vector<arma::vec> coeffs);

        int getDegreesOfFreedom() const;
        int operator==(DictionaryTrajectory const& comp) const;

        double getTmax();

        arma::vec getStartingPos();

        std::vector<arma::vec> getCoefficients();
        std::vector<QueryPoint> getQueryPoints();

        KUKADU_SHARED_PTR<Trajectory> copy();

    };

    /** \brief
     *
     *
     * \ingroup ControlPolicyFramework
     */
    class GenericGeneralizer {

    private:

    public:

        virtual KUKADU_SHARED_PTR<JointDmp> generalizeDmp(GenericKernel* trajectoryKernel, GenericKernel* parameterKernel, arma::vec query, double beta) = 0;

    };

    /**
     * \defgroup ControlPolicyFramework
     * This framework provides several methods for control policy learning, execution and reinforcement
     */

    /** \brief This class is able to generalize task specific trajectories from previous examples
     *
     * This method works well on tasks, where the sample trajectories are similar in shape. Therefore, several simple trajectories
     * have to be measured and stored in files.
     * \ingroup ControlPolicyFramework
     */
    class DMPGeneralizer : public GenericGeneralizer {

    private:

        double ax;
        double tau;
        double degOfFreedom;

        std::string baseFolder;

        DictionaryTrajectory* dictTraj;

        arma::mat computeCovarianceMatrix(int level, GenericKernel* kernel, double beta);
        std::vector<QueryPoint> mapFiles(std::vector<std::string> queryFiles, std::vector<std::string> trajFiles, std::string prefix1, std::string prefix2);

    public:

        /**
         * \brief constructor
         * \param baseFolder folder that contains the sample trajectory files
         * \param degOfFreedom number of degrees of freedom
         * \param tmpmys defines basis functions for dynamic movement primitives
         * \param tmpsigmas defines basis functions for dynamic movement primitives
         * \param az dmp az parameter
         * \param bz dmp bz parameter
         */
        DMPGeneralizer(std::string baseFolder, int degOfFreedom, std::vector<double> tmpmys, std::vector<double> tmpsigmas, double az, double bz);

        /**
         * \brief returns number of trajectory samples
         */
        int getQueryPointCount();

        double getDegOfFreedom();

        /**
         * \brief returns a sample point by index
         * \param index index of sample point
         */
        QueryPoint getQueryPointByIndex(int index);

        /**
         * \brief returns the generalized trajectory at a certain query point
         * \param trajectoryKernel kernel for trajectory shape generalization
         * \param parameterKernel kernel for trajectory goal generalization
         * \param query required query point
         * \param beta beta for Gaussian processes
         */
        KUKADU_SHARED_PTR<JointDmp> generalizeDmp(GenericKernel* trajectoryKernel, GenericKernel* parameterKernel, arma::vec query, double beta);

    };

}

#endif
