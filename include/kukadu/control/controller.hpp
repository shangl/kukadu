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

    class Controller : protected TimedObject, public StorageHolder {

    private:

#ifdef USEBOOST
        static const int CONTROLLER_ID_NOT_FOUND = -1;
#else
        static auto constexpr CONTROLLER_ID_NOT_FOUND = -1;
#endif

        bool simulation;
        bool isInstalled;
        bool storeMetaData;
        bool storeSensorData;

        bool isSkill;
        std::string skillName;

        int controllerId;

        double simulationFailingProbability;

        std::string caption;

        std::vector<KUKADU_SHARED_PTR<Hardware> > usedHardware;

    protected:

        bool isShutUp;

        // is called by set simulation mode
        virtual void setSimulationModeInChain(bool simulationMode);

        virtual KUKADU_SHARED_PTR<ControllerResult> executeInternal() = 0;

        virtual bool requiresGraspInternal() = 0;

        virtual bool producesGraspInternal() = 0;

        virtual bool isPlayable() { return false; }

        // provides information whether or not a separated database table is required to store more information
        // pair.first = true if an additional table is required
        // pair.second = name of the required additional table (will be ignored if pair.first == false)
        virtual std::pair<bool, std::string> getAugmentedInfoTableName();

        virtual void installDb();

        bool isControllerInstalled();

        // stores the current controller instance as skill (which solves a specific problem) to the database
        virtual void createSkillFromThisInternal(std::string skillName) = 0;

    public:

        Controller(StorageSingleton &dbStorage, std::string caption,
                   std::vector<KUKADU_SHARED_PTR<Hardware> > usedHardware,
                   double simulationFailingProbability,
                   const bool storeSensorData = true,
                   const bool storeMetaData = true,
                   bool isSkill = false, std::string skillName = "");

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

        virtual void prepare() {}

        double getSimFailingProb();

        std::string getCaption();

        int getControllerId();

        bool getIsSkill();

        virtual bool getLastSkillExecutionSuccessful();

        virtual KUKADU_SHARED_PTR<ControllerResult> execute();

        virtual std::string getClassName() = 0;

        // stores the current controller instance as skill (which solves a specific problem) to the database
        virtual void createSkillFromThis(std::string skillName);

        bool getStoreMetaData() const;
        bool getStoreSensorData() const;

    };

    class CartesianPtp : public Controller {

    private:

        KUKADU_SHARED_PTR<ControlQueue> leftQueue;

        geometry_msgs::Pose cartesians;

        double maxForce;
        bool maxForceSet;

    protected:

        virtual void createSkillFromThisInternal(std::string skillName);

    public:

        CartesianPtp(StorageSingleton &storage, KUKADU_SHARED_PTR<ControlQueue> leftQueue);

        bool requiresGraspInternal();

        bool producesGraspInternal();

        void setCartesians(geometry_msgs::Pose cartesians);

        std::shared_ptr<ControllerResult> executeInternal();

        void setMaxForce(double maxForce);

        std::string getClassName();

    };


    class JointPtp : public Controller {

    private:

        std::vector<double> joints;
        KUKADU_SHARED_PTR<JointHardware> hardware;
        double maxForce;

    protected:

        virtual void createSkillFromThisInternal(std::string skillName);

    public:

        JointPtp(StorageSingleton &storage, KUKADU_SHARED_PTR<JointHardware> hardware);

        bool requiresGraspInternal();

        bool producesGraspInternal();

        void setJoints(std::vector<double> joints);

        std::shared_ptr<ControllerResult> executeInternal();

        std::string getClassName();

        void setMaxForce(double maxForce);

    };

    class LocalizeObject : public Controller {

    private:

        KUKADU_SHARED_PTR<PCBlobDetector> loc;
        std::string objectName;

    protected:

        virtual void createSkillFromThisInternal(std::string skillName);

    public:

        LocalizeObject(StorageSingleton &storage, KUKADU_SHARED_PTR<Kinect> hardware);

        virtual bool requiresGraspInternal();

        virtual bool producesGraspInternal();

        void setObjectToLoad(std::string objectToLoad);

        KUKADU_SHARED_PTR<ControllerResult> executeInternal();

        void setCenterX(double x);

        void setCenterY(double y);

        void setCenterZ(double z);

        void setBoxDimX(double x);

        void setBoxDimY(double y);

        std::string getClassName();

    };

    std::vector<KUKADU_SHARED_PTR<Hardware> >
    mergeHardware(std::vector<KUKADU_SHARED_PTR<kukadu::Controller> > controllers);

    class Nothing : public kukadu::Controller {
    private:
        static int currentInstanceCount;
        static std::string nextInstanceLabel();
    protected:
        virtual KUKADU_SHARED_PTR<kukadu::ControllerResult> executeInternal();
        virtual bool requiresGraspInternal();
        virtual bool producesGraspInternal();
        virtual void createSkillFromThisInternal(std::string skillName);
    public:
        Nothing(kukadu::StorageSingleton& storage);
        virtual std::string getClassName();
    };

}

#endif
