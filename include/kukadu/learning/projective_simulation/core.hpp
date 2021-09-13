#ifndef KUKADU_PS_CORE_H
#define KUKADU_PS_CORE_H

#include <limits>
#include <string>
#include <vector>
#include <kukadu/types/kukadutypes.hpp>
#include <kukadu/learning/projective_simulation/clips.hpp>

namespace kukadu {

    class Reward {

    private:

        bool collectPrevRewards;

        std::vector<double> previousRewards;

    protected:

        KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator;

        virtual double computeRewardInternal(KUKADU_SHARED_PTR<PerceptClip> providedPercept, KUKADU_SHARED_PTR<ActionClip> takenAction) = 0;

    public:

        Reward(KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, bool collectPrevRewards);

        virtual int getDimensionality() = 0;

        double computeReward(KUKADU_SHARED_PTR<PerceptClip> providedPercept, KUKADU_SHARED_PTR<ActionClip> takenAction);

        std::vector<double> getPreviousRewards();

        virtual KUKADU_SHARED_PTR<PerceptClip> generateNextPerceptClip(int immunity) = 0;
        virtual KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<ActionClip> > > generateActionClips() = 0;
        virtual KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<PerceptClip> > > generatePerceptClips() = 0;

    };

    class ManualReward : public Reward {

    private:

        KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<ActionClip> > > actionClips;
        KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<PerceptClip> > > perceptClips;

        int nextPerceptId;
        int numberOfActions;
        int numberOfPercepts;

        double stdReward;

    protected:

        double computeRewardInternal(KUKADU_SHARED_PTR<PerceptClip> providedPercept, KUKADU_SHARED_PTR<ActionClip> takenAction);

    public:

        ManualReward(KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, int numberOfActions, int numberOfPercepts, bool collectPrevRewards, double stdReward);
        ~ManualReward();

        void setNextPerceptId(int nextId);

        int getDimensionality();

        KUKADU_SHARED_PTR<PerceptClip> generateNextPerceptClip(int immunity);
        KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<ActionClip> > > generateActionClips();
        KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<PerceptClip> > > generatePerceptClips();

    };

    class ProjectiveSimulator {

    private:

        bool useRanking;
        bool doTraining;
        bool loadedFromFile;
        bool lastRunWasBored;

        int levels;
        int maxActionId;
        int maxPerceptId;
        int operationMode;
        int immunityThresh;
        int maxNumberOfClips;

        double gamma;

        std::string psFile;

        std::vector<double> boredomLevels;

        KUKADU_SHARED_PTR<Clip> predefinedFirstHop;
        KUKADU_SHARED_PTR<Clip> lastVisitedClip;

        int lastVisitedPreviousIdx;
        int lastVisitedLevel;
        bool lastBoredomResult;
        bool walkedFurtherSinceLastBoredom;

        std::vector<std::pair<double, KUKADU_SHARED_PTR<Clip> > > rankVec;

        KUKADU_SHARED_PTR<Reward> reward;
        KUKADU_SHARED_PTR<Clip> lastClipBeforeAction;
        KUKADU_SHARED_PTR<ActionClip> lastActionClip;
        KUKADU_SHARED_PTR<PerceptClip> lastPerceptClip;
        KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator;
        KUKADU_SHARED_PTR<PerceptClip> lastGeneralizedPercept;
        KUKADU_SHARED_PTR<std::vector<int> > intermediateHops;
        KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<ActionClip> > > actionClips;
        KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<PerceptClip> > > perceptClips;
        KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<std::set<KUKADU_SHARED_PTR<Clip>, clip_compare> > > > clipLayers;

        kukadu_uniform_distribution intDist;

        void cleanByRank();
        void printRankVec();
        void computeRankVec();
        void construct(KUKADU_SHARED_PTR<Reward> reward, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, double gamma, int operationMode, bool useRanking);

        bool fileExists(const std::string filePath);

        int getIdVecLevel(KUKADU_SHARED_PTR<std::vector<int> > idVec);

        bool computeBoredom(KUKADU_SHARED_PTR<Clip> clip);

        void loadPsConstructor(KUKADU_SHARED_PTR<Reward> reward, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, std::string file,
                               std::function<KUKADU_SHARED_PTR<Clip> (const std::string&, const int&, const int&, KUKADU_SHARED_PTR<kukadu_mersenne_twister>) > createClipFunc);

    public:

        ProjectiveSimulator(KUKADU_SHARED_PTR<Reward> reward, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, std::string file);
        ProjectiveSimulator(KUKADU_SHARED_PTR<Reward> reward, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, std::string file,
                            std::function<KUKADU_SHARED_PTR<Clip> (const std::string&, const int&, const int&, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator) > createClipFunc);
        ProjectiveSimulator(KUKADU_SHARED_PTR<Reward> reward, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, double gamma, int operationMode, bool useRanking);
        ProjectiveSimulator(KUKADU_SHARED_PTR<Reward> reward, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator,
                            KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<PerceptClip> > > network,
                            double gamma, int operationMode, bool useRanking);
        ~ProjectiveSimulator();

        void printWeights();
        void setTrainingMode(bool doTraining);
        void setStandardImmunity(int immunity);
        void setBoredom(double boredom, int level);
        void setMaxNumberOfClips(int maxNumberOfClips);
        void connectNewClip(KUKADU_SHARED_PTR<Clip> conClip);
        void eliminateClip(KUKADU_SHARED_PTR<Clip> currClip);
        void generalize(KUKADU_SHARED_PTR<PerceptClip> nextClip);
        void fillClipLayersFromNetwork(KUKADU_SHARED_PTR<Clip> cl);

        KUKADU_SHARED_PTR<Clip> findClipByIdVec(KUKADU_SHARED_PTR<std::vector<int> > idVec);
        KUKADU_SHARED_PTR<Clip> findClipInLevelByIdVec(KUKADU_SHARED_PTR<std::vector<int> > idVec, int level);
        KUKADU_SHARED_PTR<Clip> findClipInLevelByLabel(std::string label, int level);

        bool compareIdVectors(std::vector<int>& idVec1, std::vector<int>& idVec2);

        int getClipCount();
        int getStandardImmunity();
        int generateNewActionId();
        int generateNewPerceptId();

        void addActionClip(KUKADU_SHARED_PTR<ActionClip> newAction);
        void addPerceptClip(KUKADU_SHARED_PTR<PerceptClip> newPercept);
        void setNextPredefinedPath(std::vector<KUKADU_SHARED_PTR<Clip> > hopPath);

        bool nextHopIsBored();
        bool lastHopWasBored();

        KUKADU_SHARED_PTR<Clip> getLastVisitedClip();

        std::tuple<bool, double, std::vector<int> > performRewarding();

        KUKADU_SHARED_PTR<std::vector<int> > getIntermediateHopIdx();

        std::pair<int, KUKADU_SHARED_PTR<Clip> > performRandomWalk(int untilLevel = PS_WALK_UNTIL_END, bool continueLastWalk = false);

        std::vector<KUKADU_SHARED_PTR<Clip> > retrieveClipsOnLayer(std::vector<int> queryId, int layer);

        // returns list of clips that have to be created
        KUKADU_SHARED_PTR<std::set<KUKADU_SHARED_PTR<Clip>, clip_compare> > createNewClips(KUKADU_SHARED_PTR<PerceptClip> nextClip);
        KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<PerceptClip> > > getPerceptClips();
        KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<ActionClip> > > getActionClips();
        KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<std::set<KUKADU_SHARED_PTR<Clip>, clip_compare> > > > getClipLayers();

        std::vector<KUKADU_SHARED_PTR<Clip> > getClipsOnLayer(int layerId);

        void updatePsFile();
        void storePS(std::string targetFile);

        static constexpr auto IGNORE_ID = INT_MIN;

        static constexpr auto PS_DEFAULT_IMMUNITY = 10000;

        static constexpr auto PS_USE_ORIGINAL = 1;
        static constexpr auto PS_USE_GEN = 2;

        static constexpr auto PS_PRINT_DEBUG_INFO = 0;
        static constexpr auto PS_PRINT_RANKING_DEBUG_INFO = 1;

        static constexpr auto PS_MAX_NUMBER_OF_CLIPS = 1000;

        static constexpr auto PS_WALK_UNTIL_END = -1;

    };

    class PSEvaluator {

    public:

        PSEvaluator();

        static void produceStatistics(KUKADU_SHARED_PTR<ProjectiveSimulator> ps, KUKADU_SHARED_PTR<Reward> reward, int numberOfWalks, int clipImmunity, int rewardValue, std::ostream& outStream);

        static std::vector<double> evaluateStatistics(std::vector<KUKADU_SHARED_PTR<std::ifstream> >& inputStreams);
        static std::pair<std::vector<int>, std::vector<double> > evaluateStatistics(std::vector<std::string> inputFiles, std::vector<int> inputPos);

        static auto constexpr PSEVAL_BUFFER_SIZE = 500;

    };

}

#endif
