#ifndef KUKADU_PLAYINGCONTROLLERS_H
#define KUKADU_PLAYINGCONTROLLERS_H

#include <limits>
#include <vector>
#include <string>
#include <memory>
#include <kukadu/robot/hand.hpp>
#include <kukadu/robot/queue.hpp>
#include <kukadu/types/kukadutypes.hpp>
#include <kukadu/control/controller.hpp>
#include <kukadu/types/controllerresult.hpp>
#include <kukadu/storage/storagesingleton.hpp>
#include <kukadu/learning/projective_simulation/core.hpp>
#include <kukadu/learning/classification/libsvmclassifier.hpp>

namespace kukadu {

    arma::vec fitToDim(arma::mat v, int dim);
    std::pair<std::vector<int>, std::vector<arma::mat> > loadClassificationData(std::vector<int> classIds, std::vector<std::string> fileNames);

    class HapticControllerResult : public ControllerResult {

    private:

        bool bored;

        std::vector<int> walkedPath;

        std::map<std::string, std::vector<double> > entropies;
        std::map<std::string, std::pair<double, double> > meanAndVar;

        KUKADU_SHARED_PTR<std::tuple<double, KUKADU_SHARED_PTR<kukadu::Clip>, std::vector<KUKADU_SHARED_PTR<kukadu::Clip> > > > environmentTransition;

    public:

        HapticControllerResult(arma::vec t, std::vector<arma::vec> ys, bool success, bool bored, std::vector<int> walkedPath, KUKADU_SHARED_PTR<std::tuple<double, KUKADU_SHARED_PTR<kukadu::Clip>, std::vector<KUKADU_SHARED_PTR<kukadu::Clip> > > > environmentTransition);

        bool wasBored();

        void setWasBored(bool wasBored);

        int getFinalStateClass();
        std::vector<int> getWalkedPath();

        std::map<std::string, std::vector<double> > getEntropies();
        std::map<std::string, std::pair<double, double> > getMeanAndVar();

        void setEntropyMeanAndVariance(std::map<std::string, std::pair<double, double> > meanAndVar);
        void setEntropies(std::map<std::string, std::vector<double> > entropies);

    };

    class EnvironmentReward : public Reward {

    private:

        double reward;

    protected:

        virtual double computeRewardInternal(KUKADU_SHARED_PTR<PerceptClip> providedPercept, KUKADU_SHARED_PTR<ActionClip> takenAction);

    public:

        EnvironmentReward(KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, double stdReward);

        virtual int getDimensionality();
        virtual KUKADU_SHARED_PTR<PerceptClip> generateNextPerceptClip(int immunity);
        virtual KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<ActionClip> > > generateActionClips();
        virtual KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<PerceptClip> > > generatePerceptClips();

    };

    class SensingController : public Controller, public std::enable_shared_from_this<SensingController> {

    private:

        bool databaseAlreadySet;
        bool requiresDatabaseFlag;

        int hapticMode;
        int stateCount;
        int currentIterationNum;
        int simulationGroundTruth;
        double simulatedClassificationPrecision;

        KUKADU_DISCRETE_DISTRIBUTION<int> classifierDist;
        KUKADU_SHARED_PTR<kukadu::Controller> parentController;

        std::string tmpPath;
        std::string databasePath;
        std::string classifierPath;
        std::string classifierFile;
        std::string classifierFunction;

        StorageSingleton& dbStorage;
        KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator;

        std::vector<KUKADU_SHARED_PTR<GenericHand> > hands;
        std::vector<KUKADU_SHARED_PTR<ControlQueue> > queues;

        void writeLabelFile(std::string baseFolderPath, std::vector<std::pair<int, std::string> > collectedSamples);

        void gatherData(std::string completePath);
        void gatherData(std::string dataBasePath, std::string dataName);

        int callClassifier(std::string passedFilePath);

    protected:

        KUKADU_SHARED_PTR<kukadu_mersenne_twister> getGenerator();
        std::vector<KUKADU_SHARED_PTR<ControlQueue> > getQueues();
        std::vector<KUKADU_SHARED_PTR<GenericHand> > getHands();
        std::string getTmpPath();
        int getHapticMode();
        double getSimClassificationPrecision();

        KUKADU_SHARED_PTR<Classifier> classifier;

        virtual KUKADU_SHARED_PTR<ControllerResult> executeInternal();

        virtual void createSkillFromThisInternal(std::string skillName);

    public:

        SensingController(StorageSingleton& storage, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, int hapticMode,
                          std::string caption,
                          std::vector<KUKADU_SHARED_PTR<ControlQueue> > cQueues, std::vector<KUKADU_SHARED_PTR<GenericHand> > hands,
                          std::string tmpPath,
                          double simClassificationPrecision,
                          std::vector<KUKADU_SHARED_PTR<Hardware> > allHardware = {},
                          const bool storeSensorData = true, const bool storeMetaData = true,
                          const bool requiresDatabaseFlag = true);

        void setSimulationGroundTruth(int idx);
        void setSimulationClassificationPrecision(int percent);

        virtual void prepare() = 0;
        virtual void cleanUp() = 0;
        virtual void performCore() = 0;

        virtual KUKADU_SHARED_PTR<kukadu::SensingController> clone() = 0;

        void setParentComplexController(KUKADU_SHARED_PTR<kukadu::Controller> parent);

        virtual int performClassification();
        int createRandomGroundTruthIdx();
        int getSimulationGroundTruthIdx();

        int getStateCount();
        void setStateCount(const int& stateCount);

        // creates a fresh database if perceptualStateId == -1 and extends the database in case the id is != -1
        double createDataBase(int perceptualStateId = -1);

        void setDatabasePath(std::string databasePath);

        std::string getDatabasePath();
        std::string getFirstRobotFileName();

        int callClassifier();

        // if a database is not required, but the classficiation is done by some non-data-driven procedure
        // overwrite requiresDatabase: return false;
        // overwrite hardcodedClassification to overwrite the classification procedure
        virtual bool requiresDatabase();
        virtual int hardcodedClassification();

        static const int HAPTIC_MODE_TERMINAL = 0;
        static const int HAPTIC_MODE_CLASSIFIER = 1;

    };

    class ConcatController : public Controller {

    private:

        std::vector<KUKADU_SHARED_PTR<kukadu::Controller> > controllers;

    protected:

        virtual KUKADU_SHARED_PTR<ControllerResult> executeInternal();

        virtual bool requiresGraspInternal();
        virtual bool producesGraspInternal();

        virtual void createSkillFromThisInternal(std::string skillName);

    public:

        ConcatController(StorageSingleton& dbStorage, std::vector<KUKADU_SHARED_PTR<kukadu::Controller> > controllers, const bool storeSensorData = true, const bool storeMetaData = true);

        virtual bool getSimulationMode();

        virtual std::string getClassName() { return "ConcatController"; }

        static std::string generateLabelFromControllers(std::vector<KUKADU_SHARED_PTR<kukadu::Controller> >& controllers);

    };

    class ControllerActionClip : public ActionClip, public Controller {

    private:

        std::string caption;

        KUKADU_SHARED_PTR<Controller> actionController;

    protected:

        virtual KUKADU_SHARED_PTR<ControllerResult> executeInternal();

        virtual bool requiresGraspInternal();
        virtual bool producesGraspInternal();

        virtual void createSkillFromThisInternal(std::string skillName);

    public:

        ControllerActionClip(StorageSingleton& dbStorage, int actionId, KUKADU_SHARED_PTR<Controller> actionController,
                             KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator,
                             const bool storeSensorData = true, const bool storeMetaData = true);

        virtual std::string toString() const;

        KUKADU_SHARED_PTR<Controller> getActionController();

        virtual std::string getClassName() { return "ControllerActionClip"; }

    };

    class IntermediateEventClip : public Clip {

    private:

        int hapticMode;

        std::string caption;

        KUKADU_SHARED_PTR<SensingController> sensingEvent;

    public:

        IntermediateEventClip(KUKADU_SHARED_PTR<SensingController> sensingEvent,
                              int level, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, std::string clipValues, int immunity);

        IntermediateEventClip(KUKADU_SHARED_PTR<SensingController> sensingEvent,
                              int level, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, KUKADU_SHARED_PTR<std::vector<int> > clipValues, int immunity);

        virtual std::string toString() const;
        std::pair<int, KUKADU_SHARED_PTR<Clip> > jumpNextRandom();

        KUKADU_SHARED_PTR<SensingController> getSensingController();

        virtual std::string getClassName() { return "IntermediateEventClip"; }

    };

    class ComplexController : public Controller, public Reward, public KUKADU_ENABLE_SHARED_FROM_THIS<ComplexController> {

    private:

        bool cleanup;
        bool storeReward;
        bool useCreativity;
        bool colPrevRewards;

        bool generateNewGroundTruth;
        bool lastSkillWasSuccessful;

        bool creativeControllerCreated;

        bool executeBasicBehaviourOnly;

        int stdPrepWeight;
        int maxEnvPathLength;
        int currentIterationNum;
        int consecutiveBoredomCount;

        double gamma;
        double boredom;
        double stdReward;
        double punishReward;
        double senseStretch;
        double pathLengthCost;

        double creativityAlpha1;
        double creativityAlpha2;
        double creativityBeta;
        double creativityCthresh;
        double creativityMultiplier;
        double nothingStateProbThresh;

        double creativityGamma;
        double creativityDelta;

        KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator;

        KUKADU_SHARED_PTR<Controller> nothingController;
        KUKADU_SHARED_PTR<EnvironmentReward> envReward;

        // outer map is used to create a nothing clip map for each sensing action
        // the inner one is used to search faster inside the nothing clips
        std::map<std::string, std::map<std::string, KUKADU_SHARED_PTR<Clip> > > nothingStateClips;

        std::string psPath;
        std::string storePath;
        std::string historyPath;
        std::string envModelPath;
        std::string rewardHistoryPath;

        std::vector<KUKADU_SHARED_PTR<Clip> > stateClips;
        std::vector<std::vector<KUKADU_SHARED_PTR<Clip> > > stateClipsBySensing;

        KUKADU_SHARED_PTR<PerceptClip> root;
        KUKADU_SHARED_PTR<kukadu_mersenne_twister> gen;
        KUKADU_SHARED_PTR<ProjectiveSimulator> projSim;
        KUKADU_SHARED_PTR<std::ofstream> rewardHistoryStream;
        KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<Clip> > > prepActions;
        KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<ActionClip> > > prepActionsCasted;

        KUKADU_DISCRETE_DISTRIBUTION<int> simSuccDist;

        std::vector<double> sensingWeights;
        std::vector<KUKADU_SHARED_PTR<Controller> > preparationControllers;
        std::vector<KUKADU_SHARED_PTR<SensingController> > sensingControllers;

        std::vector<KUKADU_SHARED_PTR<Clip> > sensingClips;

        std::map<std::string, KUKADU_SHARED_PTR<kukadu::SensingController> > availableSensingControllers;
        std::map<std::string, KUKADU_SHARED_PTR<kukadu::Controller> > availablePreparatoryControllers;

        // novelty in icdl paper
        std::map<std::string, KUKADU_SHARED_PTR<kukadu::ProjectiveSimulator> > environmentModels;
        std::map<std::string, std::vector<KUKADU_SHARED_PTR<kukadu::Clip> > > stateClipsPerSensingAction;

        double computeRewardInternal(KUKADU_SHARED_PTR<PerceptClip> providedPercept, KUKADU_SHARED_PTR<ActionClip> takenAction);

        KUKADU_SHARED_PTR<kukadu::ProjectiveSimulator> createEnvironmentModelForSensingAction(KUKADU_SHARED_PTR<kukadu::SensingController> sensingAction, KUKADU_SHARED_PTR<ProjectiveSimulator> projSim);
        std::vector<std::tuple<double, KUKADU_SHARED_PTR<Clip>, std::vector<KUKADU_SHARED_PTR<Clip> >, int> > computeEnvironmentPaths(KUKADU_SHARED_PTR<Clip> sensingClip, KUKADU_SHARED_PTR<Clip> stateClip, int maxPathLength, double confidenceCut);
        void computeTotalPathCost(KUKADU_SHARED_PTR<IntermediateEventClip> sensingClip, std::vector<std::tuple<double, KUKADU_SHARED_PTR<Clip>, std::vector<KUKADU_SHARED_PTR<Clip> >, int> >& paths);

        std::tuple<double, int, int> computeEnvironmentTransitionConfidence(KUKADU_SHARED_PTR<Clip> stateClip);

        void printPath(std::tuple<double, KUKADU_SHARED_PTR<Clip>, std::vector<KUKADU_SHARED_PTR<Clip> >, int>& path);
        void printPaths(std::vector<std::tuple<double, std::shared_ptr<Clip>, std::vector<std::shared_ptr<Clip> >, int> > &paths);

        std::tuple<KUKADU_SHARED_PTR<IntermediateEventClip>, KUKADU_SHARED_PTR<Clip>, KUKADU_SHARED_PTR<ControllerActionClip> > extractClipsFromPath(std::vector<int>& hops);

        std::vector<KUKADU_SHARED_PTR<Clip> > getAllStateClips();
        std::vector<KUKADU_SHARED_PTR<Clip> > getStateClipsForSensingId(KUKADU_SHARED_PTR<SensingController> sensingId);

        bool hasDuplicateStatesInPath(std::vector<KUKADU_SHARED_PTR<Clip> >& path);

        void loadTargetClips(KUKADU_SHARED_PTR<Clip> sensingClip, KUKADU_SHARED_PTR<Clip> sensedState);

        void storeComposition(std::string destination);

    protected:

        virtual void setSimulationModeInChain(bool simulationMode);
        virtual double getSimulatedReward(KUKADU_SHARED_PTR<kukadu::IntermediateEventClip> sensingClip,
                                          KUKADU_SHARED_PTR<kukadu::Clip> stateClip,
                                          KUKADU_SHARED_PTR<kukadu::ControllerActionClip> actionClip) = 0;

        virtual double getSimulatedRewardInternal(KUKADU_SHARED_PTR<kukadu::IntermediateEventClip> sensingClip,
                                          KUKADU_SHARED_PTR<kukadu::Clip> stateClip,
                                          KUKADU_SHARED_PTR<kukadu::ControllerActionClip> actionClip);

        virtual KUKADU_SHARED_PTR<ControllerResult> executeInternal();

        virtual void createSkillFromThisInternal(std::string skillName);

        virtual bool requiresGraspInternal() = 0;
        virtual bool producesGraspInternal() = 0;

        virtual bool isPlayable() { return true; }

    public:

        ComplexController(StorageSingleton& dbStorage, std::string caption, std::vector<KUKADU_SHARED_PTR<Hardware> > usedHardware, std::string storePath,
                          KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator,
                          KUKADU_SHARED_PTR<Controller> nothingController,
                          bool storeReward = false, double senseStretch = 15.0, double boredom = 1.0,
                          int stdReward = 100, double punishReward = -15.0, double gamma = 0.001,
                          int stdPrepWeight = 30, bool collectPrevRewards = false, int simulationFailingProbability = 0.0,
                          int maxEnvPathLength = 4, double pathLengthCost = 0.01, double stdEnvironmentReward = 10.0,
                          double creativityAlpha1 = 0.1, double creativityAlpha2 = 0.9, double creativityBeta = 0.3, double creativityCthresh = 0.8,
                          double nothingStateProbThresh = 0.8, double creativityMultiplier = 3.0,
                          const bool storeSensorData = true, const bool storeMetaData = true);
        ~ComplexController();

        virtual void cleanupAfterAction() = 0;
        virtual void executeComplexAction() = 0;
        virtual int getStateCount(const std::string& sensingName) = 0;
        virtual KUKADU_SHARED_PTR<Clip> computeGroundTruthTransition(KUKADU_SHARED_PTR<Clip> sensingClip, KUKADU_SHARED_PTR<Clip> stateClip, KUKADU_SHARED_PTR<Clip> actionClip) = 0;

        int getPrepActionCount();

        void store();
        void load(std::string path, std::map<std::string, KUKADU_SHARED_PTR<kukadu::SensingController> > availableSensingControllers, std::map<std::string, KUKADU_SHARED_PTR<kukadu::Controller> > availablePreparatoryControllers);

        virtual void initialize();
        void storeNextIteration();
        void createSensingDatabase();
        void setBoredom(double boredom);
        void store(std::string destination);
        void setTrainingMode(bool doTraining);
        void createSensingDatabase(std::vector<KUKADU_SHARED_PTR<SensingController> > sensingControllers);

        std::map<std::string, std::tuple<double, double, std::vector<double> > > computeEntropyMeanAndVariance(std::vector<KUKADU_SHARED_PTR<SensingController> > sensingIds);

        bool isTrained();
        bool setUseCreativity(bool useCreativity);

        void setSensingControllers(std::vector<KUKADU_SHARED_PTR<kukadu::SensingController> > sensingControllers);
        void setPreparatoryControllers(std::vector<KUKADU_SHARED_PTR<kukadu::Controller> > preparatoryControllers);

        void clearPreparatoryControllers();
        void addPreparatoryController(KUKADU_SHARED_PTR<kukadu::Controller> prepCont);

        int getDimensionality();

        // overwrite this virtual function if the next idx should be created randomly
        virtual int getNextSimulatedGroundTruth(KUKADU_SHARED_PTR<SensingController> sensCont);

        double getStdReward();
        double getPunishReward();

        virtual bool getLastSkillExecutionSuccessful();

        // overwrite this if the ground truth of the sensing classes is known for some sensing actions
        virtual std::string getClassLabel(KUKADU_SHARED_PTR<Clip> sensingClip, KUKADU_SHARED_PTR<Clip> stateClip);

        KUKADU_SHARED_PTR<kukadu_mersenne_twister> getGenerator();

        bool getCleanup();
        bool getGenerateNewGroundTruth();

        void setCleanup(bool cleanup);
        void setGenerateNewGroundTruth(bool groundTruth);

        void setExecuteBasicBehaviourOnly(bool executeBasicBehaviourOnly);
        bool getExecuteBasicBehaviourOnly();

        KUKADU_SHARED_PTR<ProjectiveSimulator> getProjectiveSimulator();
        KUKADU_SHARED_PTR<PerceptClip> generateNextPerceptClip(int immunity);
        KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<ActionClip> > > generateActionClips();
        KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<PerceptClip> > > generatePerceptClips();

        void updateFiles();

        virtual std::string getClassName() { return "ComplexController"; }

        virtual void prepareNextState(KUKADU_SHARED_PTR<kukadu::SensingController> cont, int currentStateIdx) = 0;

        static constexpr auto FILE_SENSING_PREFIX = "***sensing controllers:";
        static constexpr auto FILE_PREP_PREFIX = "***preparatory controllers:";
        static constexpr auto FILE_CONCAT_PREFIX = "***concatenated controllers:";
        static constexpr auto FILE_END_PREFIX = "***end";

        static constexpr auto STAR_VALUE = std::numeric_limits<int>::max();

    };

}

#endif
