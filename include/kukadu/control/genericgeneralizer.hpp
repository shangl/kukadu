#ifndef KUKADU_GENERICGENERALIZER_H
#define KUKADU_GENERICGENERALIZER_H

#include <vector>
#include <string>
#include <armadillo>

#include <kukadu/utils/types.hpp>
#include <kukadu/control/dmp.hpp>
#include <kukadu/types/kukadutypes.hpp>

namespace kukadu {

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

}

#endif
