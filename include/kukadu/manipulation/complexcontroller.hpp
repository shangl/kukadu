#ifndef KUKADU_COMPLEXCONTROLLER_H
#define KUKADU_COMPLEXCONTROLLER_H

#include <vector>
#include <string>
#include <kukadu/types/kukadutypes.hpp>
#include <kukadu/types/controllerresult.hpp>
#include <kukadu/manipulation/controller.hpp>
#include <kukadu/robot/arm/kukiecontrolqueue.hpp>
#include <kukadu/manipulation/sensingcontroller.hpp>
#include <kukadu/learning/projective_simulation/core.hpp>
#include <kukadu/manipulation/haptic/controlleractionclip.hpp>
#include <kukadu/manipulation/haptic/intermediateeventclip.hpp>

namespace kukadu {

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

    class ComplexController : public Controller, public Reward, public KUKADU_ENABLE_SHARED_FROM_THIS<ComplexController> {

    private:

        bool cleanup;
        bool storeReward;
        bool useCreativity;
        bool colPrevRewards;

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

        void printPaths(std::vector<std::tuple<double, KUKADU_SHARED_PTR<Clip>, std::vector<KUKADU_SHARED_PTR<Clip> > > >& paths);

        std::tuple<KUKADU_SHARED_PTR<IntermediateEventClip>, KUKADU_SHARED_PTR<Clip>, KUKADU_SHARED_PTR<ControllerActionClip> > extractClipsFromPath(std::vector<int>& hops);

        std::vector<KUKADU_SHARED_PTR<Clip> > getAllStateClips();
        std::vector<KUKADU_SHARED_PTR<Clip> > getStateClipsForSensingId(KUKADU_SHARED_PTR<SensingController> sensingId);

        bool hasDuplicateStatesInPath(std::vector<KUKADU_SHARED_PTR<Clip> >& path);

    protected:

        virtual void setSimulationModeInChain(bool simulationMode);
        virtual double getSimulatedReward(KUKADU_SHARED_PTR<kukadu::IntermediateEventClip> sensingClip,
                                          KUKADU_SHARED_PTR<kukadu::Clip> stateClip,
                                          KUKADU_SHARED_PTR<kukadu::ControllerActionClip> actionClip) = 0;

        virtual double getSimulatedRewardInternal(KUKADU_SHARED_PTR<kukadu::IntermediateEventClip> sensingClip,
                                          KUKADU_SHARED_PTR<kukadu::Clip> stateClip,
                                          KUKADU_SHARED_PTR<kukadu::ControllerActionClip> actionClip);

    public:

        ComplexController(std::string caption, std::string storePath,
                          bool storeReward, double senseStretch, double boredom, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator,
                          int stdReward, double punishReward, double gamma, int stdPrepWeight, bool collectPrevRewards, int simulationFailingProbability,
                          KUKADU_SHARED_PTR<Controller> nothingController,
                          int maxEnvPathLength = 4, double pathLengthCost = 0.01, double stdEnvironmentReward = 10.0,
                          double creativityAlpha1 = 0.1, double creativityAlpha2 = 0.9, double creativityBeta = 0.3, double creativityCthresh = 0.8,
                          double nothingStateProbThresh = 0.8, double creativityMultiplier = 3.0);
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

        // overwrite this if the ground truth of the sensing classes is known for some sensing actions
        virtual std::string getClassLabel(KUKADU_SHARED_PTR<Clip> sensingClip, KUKADU_SHARED_PTR<Clip> stateClip);

        KUKADU_SHARED_PTR<kukadu_mersenne_twister> getGenerator();

        virtual KUKADU_SHARED_PTR<ControllerResult> performAction();
        virtual KUKADU_SHARED_PTR<ControllerResult> performAction(bool cleanup, bool generateNewGroundTruth = true);

        KUKADU_SHARED_PTR<ProjectiveSimulator> getProjectiveSimulator();
        KUKADU_SHARED_PTR<PerceptClip> generateNextPerceptClip(int immunity);
        KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<ActionClip> > > generateActionClips();
        KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<PerceptClip> > > generatePerceptClips();

        void updateFiles();

#ifdef USEBOOST
        static const std::string FILE_SENSING_PREFIX;
        static const std::string FILE_PREP_PREFIX;
        static const std::string FILE_END_PREFIX;
        static const std::string FILE_CONCAT_PREFIX;
#else
        static constexpr auto FILE_SENSING_PREFIX = "***sensing controllers:";
        static constexpr auto FILE_PREP_PREFIX = "***preparatory controllers:";
        static constexpr auto FILE_CONCAT_PREFIX = "***concatenated controllers:";
        static constexpr auto FILE_END_PREFIX = "***end";
#endif

    };

}

#endif
