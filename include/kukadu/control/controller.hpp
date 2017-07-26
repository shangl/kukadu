#ifndef KUKADU_CONTROLLER_H
#define KUKADU_CONTROLLER_H

#include <string>
#include <kukadu/robot/queue.hpp>
#include <kukadu/robot/hardware.hpp>
#include <kukadu/vision/localizer.hpp>
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

        bool isSkill;
        std::string skillName;

        int controllerId;

        double simulationFailingProbability;

        std::string caption;

        std::vector<KUKADU_SHARED_PTR<Hardware> > usedHardware;

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

        // stores the current controller instance as skill (which solves a specific problem) to the database
        virtual void createSkillFromThisInternal(std::string skillName) = 0;

    public:

        Controller(StorageSingleton& dbStorage, std::string caption, std::vector<KUKADU_SHARED_PTR<Hardware> > usedHardware,
                   double simulationFailingProbability, bool isSkill = false, std::string skillName = "");

        void shutUp();
        void startTalking();
        virtual void setSimulationMode(bool simulationMode);

        virtual std::vector<KUKADU_SHARED_PTR<Hardware> > getUsedHardware();

        virtual bool requiresGrasp();
        virtual bool producesGrasp();

        virtual void initialize();

        // returns true, if the controller should be simulated
        // returns false otherwise
        virtual bool getSimulationMode();

        virtual void prepare() { }

        double getSimFailingProb();

        std::string getCaption();

        int getControllerId();

        bool getIsSkill();
        virtual bool getLastSkillExecutionSuccessful();

        virtual KUKADU_SHARED_PTR<ControllerResult> execute();

        virtual std::string getClassName() = 0;

        // stores the current controller instance as skill (which solves a specific problem) to the database
        virtual void createSkillFromThis(std::string skillName);

    };

    class CartesianPtp : public Controller {

    private:

        KUKADU_SHARED_PTR<ControlQueue> leftQueue;

    protected:

        virtual void createSkillFromThisInternal(std::string skillName);

    public:

        CartesianPtp(StorageSingleton& storage, KUKADU_SHARED_PTR<ControlQueue> leftQueue);

        bool requiresGraspInternal();
        bool producesGraspInternal();

        std::shared_ptr<ControllerResult> executeInternal();

        std::string getClassName();

    };


    class JointPtp : public Controller {

    private:

        std::vector<double> joints;
        KUKADU_SHARED_PTR<JointHardware> hardware;

    protected:

        virtual void createSkillFromThisInternal(std::string skillName);

    public:

        JointPtp(StorageSingleton& storage, KUKADU_SHARED_PTR<JointHardware> hardware);

        bool requiresGraspInternal();
        bool producesGraspInternal();

        void setJoints(std::vector<double> joints);

        std::shared_ptr<ControllerResult> executeInternal();

        std::string getClassName();

    };

    class LocalizeObject : public Controller {

    private:

        KUKADU_SHARED_PTR<Localizer> loc;

    protected:

        virtual void createSkillFromThisInternal(std::string skillName);

    public:

        LocalizeObject(StorageSingleton& storage, KUKADU_SHARED_PTR<Kinect> hardware);

        virtual bool requiresGraspInternal();
        virtual bool producesGraspInternal();

        KUKADU_SHARED_PTR<ControllerResult> executeInternal();

        std::string getClassName();

    };

    std::vector<KUKADU_SHARED_PTR<Hardware> > mergeHardware(std::vector<KUKADU_SHARED_PTR<kukadu::Controller> > controllers);

}

#endif
