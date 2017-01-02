#ifndef KUKADU_CONTROLLER_H
#define KUKADU_CONTROLLER_H

#include <string>
#include <kukadu/types/kukadutypes.hpp>
#include <kukadu/types/controllerresult.hpp>
#include <kukadu/storage/storagesingleton.hpp>

namespace kukadu {

    class Controller : private TimedObject {

    private:

#ifdef USEBOOST
        static const int CONTROLLER_ID_NOT_FOUND = -1;
#else
        static auto constexpr CONTROLLER_ID_NOT_FOUND = -1;
#endif

        bool simulation;
        bool isInstalled;

        int controllerId;

        double simulationFailingProbability;

        std::string caption;

    protected:

        bool isShutUp;

        StorageSingleton& storage;

        // is called by set simulation mode
        virtual void setSimulationModeInChain(bool simulationMode);
        virtual KUKADU_SHARED_PTR<ControllerResult> executeInternal() = 0;

        virtual bool requiresGraspInternal() = 0;
        virtual bool producesGraspInternal() = 0;

        // provides information whether or not a separated database table is required to store more information
        // pair.first = true if an additional table is required
        // pair.second = name of the required additional table (will be ignored if pair.first == false)
        virtual std::pair<bool, std::string> getAugmentedInfoTableName();

        virtual void installDb();
        bool isControllerInstalled();

    public:

        Controller(StorageSingleton& dbStorage, std::string caption, double simulationFailingProbability);

        void shutUp();
        void startTalking();
        virtual void setSimulationMode(bool simulationMode);

        virtual bool requiresGrasp();
        virtual bool producesGrasp();

        virtual void initialize();

        // returns true, if the controller should be simulated
        // returns false otherwise
        virtual bool getSimulationMode();

        double getSimFailingProb();

        std::string getCaption();

        virtual KUKADU_SHARED_PTR<ControllerResult> execute();

        virtual std::string getClassName() = 0;

    };

}

#endif
