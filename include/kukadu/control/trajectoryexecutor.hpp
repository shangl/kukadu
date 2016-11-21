#ifndef KUKADU_TRAJECTORYEXECUTOR_H
#define KUKADU_TRAJECTORYEXECUTOR_H

#include <kukadu/types/trajectory.hpp>
#include <kukadu/types/kukadutypes.hpp>
#include <kukadu/types/controllerresult.hpp>
#include <kukadu/manipulation/controller.hpp>

namespace kukadu {

    class TrajectoryExecutor : public Controller {

    private:


    public:

        TrajectoryExecutor();

        virtual KUKADU_SHARED_PTR<ControllerResult> executeTrajectory() = 0;
        virtual KUKADU_SHARED_PTR<ControllerResult> simulateTrajectory() = 0;

        virtual void setTrajectory(KUKADU_SHARED_PTR<Trajectory> traj) = 0;

        KUKADU_SHARED_PTR<ControllerResult> performAction();

    };

}

#endif
