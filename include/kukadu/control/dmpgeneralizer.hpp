#ifndef KUKADU_DMPGENERALIZER_H
#define KUKADU_DMPGENERALIZER_H

#include <string>
#include <vector>
#include <armadillo>
#include <kukadu/utils/types.hpp>
#include <kukadu/utils/utils.hpp>
#include <kukadu/types/querypoint.hpp>
#include <kukadu/types/kukadutypes.hpp>
#include <kukadu/types/dictionarytrajectory.hpp>
#include <kukadu/control/genericgeneralizer.hpp>
#include <kukadu/learning/regression/lwrregressor.hpp>
#include <kukadu/learning/regression/generickernel.hpp>
#include <kukadu/learning/regression/gaussianprocessregressor.hpp>

namespace kukadu {

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
