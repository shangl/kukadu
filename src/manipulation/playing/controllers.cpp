#include <limits>
#include <fstream>
#include <sstream>
#include <Python.h>
#include <armadillo>
#include <boost/filesystem.hpp>
#include <kukadu/utils/utils.hpp>
#include <kukadu/robot/hardware.hpp>
#include <kukadu/utils/kukadutokenizer.hpp>
#include <kukadu/storage/sensorstorage.hpp>
#include <kukadu/storage/moduleusagesingleton.hpp>
#include <kukadu/manipulation/playing/controllers.hpp>
#include <kukadu/learning/projective_simulation/core.hpp>

#include <kukadu/utils/tictoc.hpp>

using namespace std;
using namespace arma;
namespace pf = boost::filesystem;

#define ASK_GETCHAR_CONF (false)

namespace kukadu {

    ComplexController::ComplexController(StorageSingleton& dbStorage, std::string caption,
                                         std::vector<KUKADU_SHARED_PTR<Hardware> > usedHardware, std::string storePath,
                                         KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator,
                                         KUKADU_SHARED_PTR<Controller> nothingController,
                                         bool storeReward, double senseStretch, double boredom,
                                         int stdReward, double punishReward, double gamma, int stdPrepWeight, bool collectPrevRewards,
                                         int simulationFailingProbability,
                                         int maxEnvPathLength, double pathLengthCost, double stdEnvironmentReward,
                                         double creativityAlpha1, double creativityAlpha2, double creativityBeta, double creativityCthresh,
                                         double nothingStateProbThresh, double creativityMultiplier,
                                         const bool storeSensorData, const bool storeMetaData)
        : Controller(dbStorage, caption, usedHardware, simulationFailingProbability, storeSensorData, storeMetaData), Reward(generator, collectPrevRewards) {

		if(!isShutUp)
			cout << "database path: " << storePath << endl;

        this->useCreativity = false;
        this->executeBasicBehaviourOnly = true;
        this->creativeControllerCreated = false;
        this->creativityAlpha1 = creativityAlpha1;
        this->creativityAlpha2 = creativityAlpha2;
        this->creativityBeta = creativityBeta;
        this->creativityCthresh = creativityCthresh;
        this->creativityMultiplier = creativityMultiplier;
        this->nothingStateProbThresh = nothingStateProbThresh;

        this->creativityGamma = (atanh(1.0 - 2 * creativityAlpha1) - atanh(1.0 - 2 * creativityAlpha2)) / (creativityCthresh * creativityBeta);
        this->creativityDelta = ((2 * creativityBeta - 1.0) * tanh(1.0 - 2 * creativityAlpha1) + atanh(1.0 - 2 * creativityAlpha2)) / creativityBeta;

        this->generator = generator;
        this->nothingController = nothingController;
        consecutiveBoredomCount = 0;

        projSim.reset();
        rewardHistoryStream.reset();

        replace(storePath.begin(), storePath.end(), ' ', '_');

        preparePathString(storePath);

        this->gamma = gamma;
        this->gen = generator;
        this->boredom = boredom;
        this->currentIterationNum = 0;
        this->storeReward = storeReward;
        this->senseStretch = senseStretch;
        this->colPrevRewards = collectPrevRewards;
        this->rewardHistoryPath = storePath + "rewards/";

        this->pathLengthCost = pathLengthCost;
        this->maxEnvPathLength = maxEnvPathLength;

        if(!fileExists(rewardHistoryPath))
            createDirectory(rewardHistoryPath);

        this->stdReward = stdReward;
        this->storePath = storePath;
        this->punishReward = punishReward;
        this->stdPrepWeight = stdPrepWeight;
        
        if(!isShutUp)
			cout << "finally stored path: " << this->storePath << endl;

        cleanup = true;
        generateNewGroundTruth = true;
        lastSkillWasSuccessful = false;

        vector<int> distributionWeights; distributionWeights.push_back(100 - simulationFailingProbability); distributionWeights.push_back(simulationFailingProbability);
        simSuccDist = KUKADU_DISCRETE_DISTRIBUTION<int>(distributionWeights.begin(), distributionWeights.end());

        envReward = KUKADU_SHARED_PTR<EnvironmentReward>(new EnvironmentReward(generator, stdEnvironmentReward));

    }

    void ComplexController::createSkillFromThisInternal(std::string skillName) {
        throw KukaduException("(ComplexController) creating a skill from instance not supported yet");
    }

    bool ComplexController::getCleanup() {
        return cleanup;
    }

    bool ComplexController::getGenerateNewGroundTruth() {
        return generateNewGroundTruth;
    }

    void ComplexController::setCleanup(bool cleanup) {
        this->cleanup = cleanup;
    }

    void ComplexController::setGenerateNewGroundTruth(bool groundTruth) {
        this->generateNewGroundTruth = groundTruth;
    }

    ComplexController::~ComplexController() {
        if(rewardHistoryStream && storeReward)
            rewardHistoryStream->close();
    }

    int ComplexController::getPrepActionCount() {
        return prepActions->size();
    }

    void ComplexController::setSensingControllers(std::vector<KUKADU_SHARED_PTR<kukadu::SensingController> > sensingControllers) {
        this->sensingControllers = sensingControllers;
    }

    void ComplexController::setPreparatoryControllers(std::vector<KUKADU_SHARED_PTR<kukadu::Controller> > preparatoryControllers) {
        this->preparationControllers = preparatoryControllers;
    }

    void ComplexController::clearPreparatoryControllers() {
        this->preparationControllers.clear();
    }

    void ComplexController::addPreparatoryController(KUKADU_SHARED_PTR<kukadu::Controller> prepCont) {
        this->preparationControllers.push_back(prepCont);
    }

    void ComplexController::updateFiles() {

        KUKADU_MODULE_START_USAGE();

        for(auto env : environmentModels)
            env.second->updatePsFile();

        projSim->updatePsFile();

        storeComposition(storePath);

        KUKADU_MODULE_END_USAGE();

    }

    void ComplexController::initialize() {

        creativeControllerCreated = false;
        executeBasicBehaviourOnly = true;

        nothingStateClips.clear();

        psPath = storePath + "ps";
        historyPath = rewardHistoryPath + "history";

        bool existsPs = fileExists(psPath);
        envModelPath = storePath + "envmodels";
        preparePathString(envModelPath);

        if(existsPs) {

            // skill was already initialized and can be loaded again
            if(!isShutUp)
                cout << "(ComplexController) loading existing PS" << endl;

            auto loadLambda = [this] (const std::string& line, const int& level, const int& perceptDimensionality, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator) -> KUKADU_SHARED_PTR<Clip> {

                KukaduTokenizer tok(line, ";");
                string idVec = tok.next();
                string label = tok.next();
                int immunity = atoi(tok.next().c_str());
                if(level == 0) {

                    auto pc = make_shared<PerceptClip>(atoi(tok.next().c_str()), label, generator, idVec, immunity);
                    return pc;

                } else if(level == Clip::CLIP_H_LEVEL_FINAL) {

                    if(this->availablePreparatoryControllers.find(label) != this->availablePreparatoryControllers.end() && (this->availablePreparatoryControllers)[label]) {
                        auto ac = make_shared<ControllerActionClip>(getStorage(), atoi(tok.next().c_str()), (this->availablePreparatoryControllers)[label], generator, getStoreSensorData(), getStoreMetaData());
                        return ac;
                    } else
                        return nullptr;

                } else {

                    if(level == 1) {

                        if(this->availableSensingControllers.find(label) != this->availableSensingControllers.end() && (this->availableSensingControllers)[label]) {

                            return make_shared<IntermediateEventClip>((this->availableSensingControllers)[label],
                                                                                            level, generator, idVec, immunity);

                        } else
                            return nullptr;

                    }

                    return make_shared<Clip>(level, generator, idVec, immunity);

                }

            };

            projSim = make_shared<ProjectiveSimulator>(shared_from_this(), generator, psPath, loadLambda);

            // ugly syntax - i have to kill these shared pointers some day
            prepActions = (*((*(projSim->getClipLayers()->end() - 1))->begin()))->getSubClips();
            prepActionsCasted = projSim->getActionClips();

            root = *(projSim->getPerceptClips()->begin());

        } else {

            int currentId = 0;
            KUKADU_SHARED_PTR<vector<int> > clipDimVal = KUKADU_SHARED_PTR<vector<int> >(new vector<int>());
            clipDimVal->push_back(currentId);
            root = make_shared<PerceptClip>(0, "root", generator, clipDimVal, INT_MAX);

            vector<double> prepWeights;
            prepActions = make_shared<vector<KUKADU_SHARED_PTR<Clip> > >();
            prepActionsCasted = make_shared<vector<KUKADU_SHARED_PTR<ActionClip> > >();

            for(int i = 0; i < preparationControllers.size(); ++i) {

                auto prepActionClip = make_shared<ControllerActionClip>(getStorage(), i, preparationControllers.at(i), generator, getStoreSensorData(), getStoreMetaData());
                prepActions->push_back(prepActionClip);
                prepActionsCasted->push_back(prepActionClip);
                prepWeights.push_back(stdPrepWeight);

            }

            ++currentId;

            for(int i = 0; i < sensingControllers.size(); ++i) {

                KUKADU_SHARED_PTR<SensingController> sensCont = sensingControllers.at(i);

                clipDimVal = make_shared<vector<int> >();
                clipDimVal->push_back(currentId);

                KUKADU_SHARED_PTR<Clip> nextSensClip = make_shared<IntermediateEventClip>(sensCont, 1, generator, clipDimVal, INT_MAX);
                ++currentId;
                for(int j = 0; j < getStateCount(sensCont->getCaption()); ++j, ++currentId) {

                    clipDimVal = make_shared<vector<int> >();
                    clipDimVal->push_back(currentId);
                    KUKADU_SHARED_PTR<Clip> nextSubSensClip = make_shared<Clip>(2, generator, clipDimVal, INT_MAX);
                    // create copy of prepratory actions
                    auto prepActionsCopy = make_shared<vector<KUKADU_SHARED_PTR<Clip> > >(prepActions->begin(), prepActions->end());
                    nextSubSensClip->setChildren(prepActionsCopy, prepWeights);
                    nextSensClip->addSubClip(nextSubSensClip, stdPrepWeight);

                }

                double nextWeight = std::exp(senseStretch * max(0.0, sensingWeights.at(i) - 1.0 / getStateCount(sensCont->getCaption())));
                sensCont->setSimulationClassificationPrecision(min(sensingWeights.at(i) * 100.0, 100.0));

                if(!isShutUp)
                    cout << "(ComplexController) relative weight of sensing action \"" << sensCont->getCaption() << "\" is " << nextWeight << endl;

                root->addSubClip(nextSensClip, nextWeight);

            }

            KUKADU_SHARED_PTR<vector<KUKADU_SHARED_PTR<PerceptClip> > > rootVec = make_shared<vector<KUKADU_SHARED_PTR<PerceptClip> > >();
            rootVec->push_back(root);

            int originalMode = ProjectiveSimulator::PS_USE_ORIGINAL;

            // skill is used the first time; do initialization
            projSim = make_shared<ProjectiveSimulator>(shared_from_this(), generator, rootVec, gamma, originalMode, false);

        }

        environmentModels.clear();
        if(fileExists(envModelPath)) {

            // load environment model
            for(auto sens : sensingControllers) {
                auto envModel = make_shared<ProjectiveSimulator>(envReward, generator, envModelPath + sens->getCaption());
                envModel->setTrainingMode(true);
                environmentModels.insert(std::pair<std::string, KUKADU_SHARED_PTR<kukadu::ProjectiveSimulator> >(sens->getCaption(), envModel));
            }

        } else {

            // create environment model
            createDirectory(envModelPath);

            // load environment model
            for(auto sens : sensingControllers) {

                auto envModel = createEnvironmentModelForSensingAction(sens, projSim);
                envModel->setTrainingMode(true);
                environmentModels.insert(std::pair<std::string, KUKADU_SHARED_PTR<kukadu::ProjectiveSimulator> >(sens->getCaption(), envModel));
                envModel->storePS(envModelPath + sens->getCaption());

            }

        }

        projSim->setBoredom(boredom, 2);

        if(storeReward) {
            rewardHistoryStream = make_shared<std::ofstream>();
            int overWrite = 0;
            if(fileExists(historyPath)) {
                cout << "(ComplexController) should reward history file be overwritten? (0 = no / 1 = yes)" << endl;
                cin >> overWrite;
                if(overWrite != 1)
                    throw KukaduException("(ComplexController) reward file already exists. you chose not to overwrite. stopping");
            }
            rewardHistoryStream->open(rewardHistoryPath.c_str(), ios::trunc);
        }

        stateClips = projSim->getClipsOnLayer(2);
        for(auto sensingClip : *projSim->getPerceptClips()->at(0)->getSubClips()) {
            stateClipsBySensing.push_back(*sensingClip->getSubClips());
            stateClipsPerSensingAction[KUKADU_DYNAMIC_POINTER_CAST<IntermediateEventClip>(sensingClip)->getSensingController()->getCaption()] = *sensingClip->getSubClips();
        }

        // get all sensing clips
        sensingClips = projSim->getClipsOnLayer(1);
        for(auto sClip : sensingClips) {
            auto stateClips = sClip->getSubClips();
            if(stateClips) {
                for(auto states : *stateClips)
                    loadTargetClips(sClip, states);
            }
        }

        if(!isShutUp) cout << "(ComplexController) number of target states: " << nothingStateClips.size() << endl;

    }

    bool ComplexController::getLastSkillExecutionSuccessful() {
        // call base class function for consistency checks
        Controller::getLastSkillExecutionSuccessful();
        return lastSkillWasSuccessful;
    }

    std::vector<KUKADU_SHARED_PTR<Clip> > ComplexController::getStateClipsForSensingId(KUKADU_SHARED_PTR<SensingController> sensingId) {
        return stateClipsPerSensingAction[sensingId->getCaption()];
    }

    std::vector<KUKADU_SHARED_PTR<Clip> > ComplexController::getAllStateClips() {
        return stateClips;
    }

    std::map<std::string, std::tuple<double, double, std::vector<double> > > ComplexController::computeEntropyMeanAndVariance(std::vector<KUKADU_SHARED_PTR<SensingController> > sensingIds) {

        KUKADU_MODULE_START_USAGE();

        std::map<std::string, std::tuple<double, double, std::vector<double> > > retMap;

        for(auto sensingId : sensingIds) {

            double entropyMean = 0.0;
            double entropyVar = 0.0;
            vector<double> entropies;

            auto sensingStateClips = getStateClipsForSensingId(sensingId);
            for(auto stateClip : sensingStateClips) {
                double entropy = stateClip->computeSubEntropy();
                entropyMean += entropy;
                entropies.push_back(entropy);
            }

            entropyMean /= sensingStateClips.size();

            for(auto entropy : entropies)
                entropyVar += std::pow(entropy - entropyMean, 2.0);
            entropyVar /= sensingStateClips.size();

            std::tuple<double, double, std::vector<double> > entropyInf{entropyMean, entropyVar, entropies};
            retMap[sensingId->getCaption()] = entropyInf;

        }

        KUKADU_MODULE_END_USAGE();

        return retMap;

    }

    std::string ComplexController::getClassLabel(KUKADU_SHARED_PTR<Clip> sensingClip, KUKADU_SHARED_PTR<Clip> stateClip) {
        return stateClip->toString();
    }

    void ComplexController::setBoredom(double boredom) {
        KUKADU_MODULE_START_USAGE();
        projSim->setBoredom(boredom, 2);
        KUKADU_MODULE_END_USAGE();
    }

    double ComplexController::getPunishReward() {
        return punishReward;
    }

    double ComplexController::getStdReward() {
        return stdReward;
    }

    void ComplexController::setSimulationModeInChain(bool simulationMode) {

        KUKADU_MODULE_START_USAGE();

        for(auto sensCont : sensingControllers)
            if(sensCont && sensCont->getSimulationMode() != simulationMode)
                sensCont->setSimulationMode(simulationMode);

        for(auto prepCont : preparationControllers)
            if(prepCont && prepCont->getSimulationMode() != simulationMode)
                prepCont->setSimulationMode(simulationMode);

        KUKADU_MODULE_END_USAGE();

    }

    bool ComplexController::isTrained() {
        if(consecutiveBoredomCount > 20)
            return true;
        return false;
    }

    KUKADU_SHARED_PTR<kukadu::ProjectiveSimulator> ComplexController::createEnvironmentModelForSensingAction(KUKADU_SHARED_PTR<kukadu::SensingController> sensingAction,
                                                                                          KUKADU_SHARED_PTR<ProjectiveSimulator> projSim) {

        KUKADU_MODULE_START_USAGE();

        KUKADU_SHARED_PTR<IntermediateEventClip> sensingClip = nullptr;

        auto sensingLayer = projSim->getClipLayers()->at(1);
        for(auto sensingAct : *sensingLayer) {
            if(KUKADU_DYNAMIC_POINTER_CAST<IntermediateEventClip>(sensingAct)->toString() == sensingAction->getCaption()) {
                sensingClip = KUKADU_DYNAMIC_POINTER_CAST<IntermediateEventClip>(sensingAct);
                break;
            }
        }
        if(!sensingClip)
            throw KukaduException("(ComplexController) sensing action not available");
        auto prepClips = sensingClip->getSubClipByIdx(0)->getSubClips();
        int sensingCatCount = sensingClip->getSubClipCount();
        int prepActionsCount = prepClips->size();
        auto stateClips = sensingClip->getSubClips();
        auto environmentPercepts = make_shared<vector<KUKADU_SHARED_PTR<PerceptClip> > >();
        auto resultingStatePercepts = make_shared<vector<KUKADU_SHARED_PTR<Clip> > >();
        auto idVec = KUKADU_SHARED_PTR< vector<int> >(new vector<int>{0, 0});
        for(auto stateClip : *stateClips) {
            auto stateId = stateClip->getClipDimensions()->at(0);
            stringstream s;
            s << "E" << stateId;
            // have to make it -1 because action clip says internally --> -stateId - 1 (i can't remember the reason anymore)
            resultingStatePercepts->push_back(make_shared<ActionClip>(stateId - 1, idVec->size(), s.str(), generator));
        }
        bool containsPrepAction = false;
        for(int stateIdx = 0, overallId = sensingCatCount; stateIdx < sensingCatCount; ++stateIdx) {
            int stateId = stateClips->at(stateIdx)->getClipDimensions()->at(0);
            idVec->at(0) = stateId;
            for(int actId = 0; actId < prepActionsCount; ++actId, ++overallId) {
                // ignore the "nothing" controller -> by definition it does nothing and
                // can't be used to change the environment state
                if(prepClips->at(actId)->toString() != nothingController->getCaption()) {
                    containsPrepAction = true;
                    idVec->at(1) = prepClips->at(actId)->getClipDimensions()->at(0);
                    stringstream s;
                    s << "(E" << stateId << ",P" << idVec->at(1) << ")";
                    auto vecCopy = make_shared<vector<int> >(idVec->begin(), idVec->end());
                    auto newPercept = make_shared<PerceptClip>(overallId, s.str(), generator, vecCopy, INT_MAX);
                    newPercept->setChildren(resultingStatePercepts);
                    environmentPercepts->push_back(newPercept);
                }
            }
        }
        if(!containsPrepAction)
            throw KukaduException("(ComplexController) No preparatory action without the label of the nothing action provided");
        auto psMode = ProjectiveSimulator::PS_USE_ORIGINAL;
        auto retProjSim = make_shared<ProjectiveSimulator>(envReward, generator, environmentPercepts, 0.0, psMode, false);

        retProjSim->setTrainingMode(true);

        KUKADU_MODULE_END_USAGE();

        return retProjSim;

    }

    KUKADU_SHARED_PTR<ProjectiveSimulator> ComplexController::getProjectiveSimulator() {
        return projSim;
    }

    void ComplexController::load(std::string path, std::map<std::string, KUKADU_SHARED_PTR<kukadu::SensingController> > availableSensingControllers, std::map<std::string, KUKADU_SHARED_PTR<kukadu::Controller> > availablePreparatoryControllers) {

        KUKADU_MODULE_START_USAGE();

        sensingControllers.clear();
        if(prepActions)
            prepActions->clear();
        if(prepActionsCasted)
            prepActionsCasted->clear();
        preparationControllers.clear();

        preparePathString(path);
        string compositionDestination = path + "composition";

        ifstream compositionFile;
        compositionFile.open(compositionDestination);

        int controllerMode = -1;
        string line = "";
        while(getline(compositionFile, line)) {

            if(!line.compare(FILE_SENSING_PREFIX))
                controllerMode = 1;
            else if(!line.compare(FILE_PREP_PREFIX))
                controllerMode = 2;
            else if(!line.compare(FILE_CONCAT_PREFIX))
                controllerMode = 3;
            else if(!line.compare(FILE_END_PREFIX))
                controllerMode = 10;
            else if(line.compare("")) {
                // we are reading sensing controllers
                if(controllerMode == 1)
                    sensingControllers.push_back(availableSensingControllers[line]);
                else if(controllerMode == 2 ) {
                    if(!availablePreparatoryControllers[line])
                        cout << "(ComplexController::load) the controller " << line << " is null" << endl;
                    preparationControllers.push_back(availablePreparatoryControllers[line]);
                } else if(controllerMode == 3) {

                    // construct composed controllers here
                    KukaduTokenizer tok(line, "$");
                    auto splits = tok.split();

                    vector<KUKADU_SHARED_PTR<kukadu::Controller> > controllerParts;
                    for(auto cont : splits)
                        controllerParts.push_back(availablePreparatoryControllers[cont]);

                    auto conc = make_shared<ConcatController>(getStorage(), controllerParts);
                    preparationControllers.push_back(conc);

                    availablePreparatoryControllers[line] = conc;

                } else if(controllerMode == 10)
                    break;
            }

        }

        storePath = path;
        this->availableSensingControllers = availableSensingControllers;
        this->availablePreparatoryControllers = availablePreparatoryControllers;

        createSensingDatabase();
        initialize();

        KUKADU_MODULE_END_USAGE();

    }

    void ComplexController::store() {
        KUKADU_MODULE_START_USAGE();
        store(storePath);
        KUKADU_MODULE_END_USAGE();
    }

    void ComplexController::storeComposition(std::string destination) {

        string compositionDestination = destination + "composition";
        ofstream compositionFile;
        compositionFile.open(compositionDestination);
        compositionFile << FILE_SENSING_PREFIX << endl;
        for(auto sensCont : sensingControllers)
            compositionFile << sensCont->getCaption() << endl;

        compositionFile << FILE_PREP_PREFIX << endl;
        for(auto prepCont : preparationControllers) {
            string contCaption = prepCont->getCaption();
            if(contCaption.find(' ') == std::string::npos)
                compositionFile << contCaption << endl;
        }

        compositionFile << FILE_CONCAT_PREFIX << endl;
        for(auto prepCont : preparationControllers) {
            string contCaption = prepCont->getCaption();
            if(contCaption.find(' ') != std::string::npos)
                compositionFile << contCaption << endl;
        }

        compositionFile.close();

    }

    void ComplexController::store(std::string destination) {

        KUKADU_MODULE_START_USAGE();

        preparePathString(destination);
        string psDestination = destination + "ps";
        projSim->storePS(psDestination);

        storeComposition(destination);

        KUKADU_MODULE_END_USAGE();

    }

    void ComplexController::storeNextIteration() {

        KUKADU_MODULE_START_USAGE();

        stringstream s;
        s << psPath << currentIterationNum;
        projSim->storePS(s.str());

        KUKADU_MODULE_END_USAGE();

    }

    int ComplexController::getDimensionality() {
        if(!isShutUp)
            cout << "(ComplexController) get dimensionality got called" << endl;
        return 1;
    }

    // same percept must always have same id
    KUKADU_SHARED_PTR<PerceptClip> ComplexController::generateNextPerceptClip(int immunity) {
        return root;
    }

    KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<ActionClip> > > ComplexController::generateActionClips() {
        return prepActionsCasted;
    }

    KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<PerceptClip> > > ComplexController::generatePerceptClips() {
        vector<KUKADU_SHARED_PTR<PerceptClip> >* rootRet;
        rootRet->push_back(root);
        return KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<PerceptClip> > >(rootRet);
    }

    double ComplexController::getSimulatedRewardInternal(KUKADU_SHARED_PTR<kukadu::IntermediateEventClip> sensingClip,
                                                 KUKADU_SHARED_PTR<kukadu::Clip> stateClip,
                                                 KUKADU_SHARED_PTR<kukadu::ControllerActionClip> actionClip) {

        KUKADU_MODULE_START_USAGE();

        // simulates non-perfect complex controller
        int failed = simSuccDist(*generator);
        if(failed)
            return getPunishReward();

        double simReward = getSimulatedReward(sensingClip, stateClip, actionClip);

        KUKADU_MODULE_END_USAGE();

        return simReward;

    }

    int ComplexController::getNextSimulatedGroundTruth(KUKADU_SHARED_PTR<SensingController> sensCont) {
        return sensCont->createRandomGroundTruthIdx();
    }

    KUKADU_SHARED_PTR<kukadu_mersenne_twister> ComplexController::getGenerator() {
        return gen;
    }

    std::tuple<KUKADU_SHARED_PTR<IntermediateEventClip>, KUKADU_SHARED_PTR<Clip>, KUKADU_SHARED_PTR<ControllerActionClip> > ComplexController::extractClipsFromPath(std::vector<int>& hops) {

        auto firstPercept = projSim->getPerceptClips()->at(0);
        auto currentClip = KUKADU_DYNAMIC_POINTER_CAST<Clip>(firstPercept);
        vector<KUKADU_SHARED_PTR<Clip> > clipPath = {currentClip};

        auto hopsSize = hops.size();
        for(int i = 1; i < hopsSize; ++i) {

            int nextHop = hops.at(i);
            currentClip = currentClip->getSubClipByIdx(nextHop);
            clipPath.push_back(currentClip);

        }

        auto pathSize = clipPath.size();
        KUKADU_SHARED_PTR<Clip> stateClip = (pathSize >= 3) ? clipPath.at(2) : nullptr;
        KUKADU_SHARED_PTR<IntermediateEventClip> sensingClip = (pathSize >= 2) ? KUKADU_DYNAMIC_POINTER_CAST<IntermediateEventClip>(clipPath.at(1)) : nullptr;
        KUKADU_SHARED_PTR<ControllerActionClip> actionClip = (pathSize >= 4) ? KUKADU_DYNAMIC_POINTER_CAST<ControllerActionClip>(clipPath.at(3)) : nullptr;

        return tuple<KUKADU_SHARED_PTR<IntermediateEventClip>, KUKADU_SHARED_PTR<Clip>, KUKADU_SHARED_PTR<ControllerActionClip> >(sensingClip, stateClip, actionClip);

    }

    double ComplexController::computeRewardInternal(KUKADU_SHARED_PTR<PerceptClip> providedPercept, KUKADU_SHARED_PTR<ActionClip> takenAction) {

        KUKADU_MODULE_START_USAGE();

        int worked = 0;
        int executeIt = 0;

        double retReward = 0.0;

        auto intermed = projSim->getIntermediateHopIdx();
        auto nonCastedSenseClip = providedPercept->getSubClipByIdx(intermed->at(1));
        auto sensClip = KUKADU_DYNAMIC_POINTER_CAST<IntermediateEventClip>(nonCastedSenseClip);
        auto sensCont = sensClip->getSensingController();
        auto castedAction = KUKADU_DYNAMIC_POINTER_CAST<ControllerActionClip>(takenAction);
        auto stateClip = sensClip->getSubClipByIdx(intermed->at(2));

        if(!getSimulationMode()) {

            cout << "(ComplexController) do you want to execute complex action now? (0 = no / 1 = yes)" << endl;
            cin >> executeIt;

//            executeIt = 1;

            if(executeIt)
                executeComplexAction();

            cout << "did the complex action succeed? (0 = no / 1 = yes)" << endl;
//            cin >> worked;
            worked = 1;

            if(worked) {
                cout << "preparation action worked; rewarded with " << stdReward << endl;
                retReward = stdReward;
            } else {
                cout << "preparation action didn't work; no reward given" << endl;
                retReward = punishReward;
            }

            if(cleanup && executeIt)
                cleanupAfterAction();

        } else {

            retReward = getSimulatedRewardInternal(sensClip, stateClip, castedAction);

        }

        if(storeReward)
            *rewardHistoryStream << retReward << "\t" << *sensClip << "\t" << sensCont->getSimulationGroundTruthIdx() << "\t" << *stateClip << "\t\t" << *takenAction << endl;

        KUKADU_MODULE_END_USAGE();

        return retReward;

    }

    void ComplexController::loadTargetClips(KUKADU_SHARED_PTR<Clip> sensingClip, KUKADU_SHARED_PTR<Clip> sensedState) {

        // pair contains maximal hop probability of clip and the corresponding clip
        auto maxProbClipPair = sensedState->getMaxProbability();
        double maxProb = get<0>(maxProbClipPair);
        int maxWeight = get<1>(maxProbClipPair);
        auto maxPrepClip = get<2>(maxProbClipPair);

        // if probability of success at some state is high enough, the state clip might be considered in future reasoning
        // e.g. for guided clip creation

        //cout << "load target clips: " << maxWeight << ", " << stdPrepWeight << ", " << maxProb << ", " << nothingStateProbThresh << ", " << sensedState->toString() << endl;

        if(maxWeight > stdPrepWeight) {
            if(maxProb > nothingStateProbThresh &&
                nothingStateClips[sensingClip->toString()].find(sensedState->toString()) == nothingStateClips[sensingClip->toString()].end()) {
                nothingStateClips[sensingClip->toString()][sensedState->toString()] = sensedState;
            // if there is a state where the strongest action is "nothing" but the probability is below 0.8, remove it (even if it is not in there yet - checking
            // this would just make it slower)
            } else if(maxProb <= nothingStateProbThresh && KUKADU_DYNAMIC_POINTER_CAST<ControllerActionClip>(maxPrepClip)->toString() == "nothing")
                nothingStateClips[sensingClip->toString()].erase(sensedState->toString());

        }

    }

    void ComplexController::setExecuteBasicBehaviourOnly(bool executeBasicBehaviourOnly) {
        this->executeBasicBehaviourOnly = executeBasicBehaviourOnly;
    }

    bool ComplexController::getExecuteBasicBehaviourOnly() {
        return executeBasicBehaviourOnly;
    }

    KUKADU_SHARED_PTR<ControllerResult> ComplexController::executeInternal() {

        KUKADU_MODULE_START_USAGE();
        KUKADU_SHARED_PTR<ControllerResult> ret = nullptr;

        if(executeBasicBehaviourOnly) {
            executeComplexAction();
            cleanupAfterAction();
            return ret;
        } else {

            // if simulation - set observed state as ground truth for each sensing action (it is not yet known, which sensing action will be selected)
            if(getSimulationMode() && generateNewGroundTruth) {
                for(auto sensCont : sensingControllers) {
                    auto nextGroundTruth = getNextSimulatedGroundTruth(sensCont);
                    sensCont->setSimulationGroundTruth(nextGroundTruth);
                }
            }

            ++currentIterationNum;

            auto walkRet = projSim->performRandomWalk(2);
            auto wasBored = projSim->nextHopIsBored();

            double reward = 0.0;
            std::tuple<double, KUKADU_SHARED_PTR<Clip>, std::vector<KUKADU_SHARED_PTR<Clip> >, int> selectedPath;

            // if the last clip is an action clip, PS was not bored
            if(!wasBored) {

                auto hopPath = projSim->getIntermediateHopIdx();

                // if not bored, eventually add a new preparatory skill by creative composition before going on with the walk
                auto newClips = extractClipsFromPath(*hopPath);
                auto sensingClip = get<0>(newClips);
                auto stateClip = get<1>(newClips);
                auto stateId = stateClip->getClipDimensions()->at(0);

				if(!isShutUp)
					cout << "(ComplexController::executeInternal) creativity activated: " << useCreativity << endl;

                // block for creativity mode
                if(useCreativity) {

					if(!isShutUp)
						cout << "(ComplexController::executeInternal) i may be creative" << endl;

                    // only add new prep skill if current state is not already a "nothing" state
                    if(nothingStateClips[sensingClip->toString()].find(stateClip->toString()) == nothingStateClips[sensingClip->toString()].end()) {

                        // compute all paths with maximum length of 4 and minimal confidence 0.4
                        auto possiblePaths = computeEnvironmentPaths(sensingClip, stateClip, 3, 0.4);

                        // find out if there is a transition to a "nothing" state
                        // sort them according to confidence
                        std::sort(possiblePaths.begin(), possiblePaths.end(), [] (std::tuple<double, KUKADU_SHARED_PTR<Clip>, std::vector<KUKADU_SHARED_PTR<Clip> >, int> p1, std::tuple<double, KUKADU_SHARED_PTR<Clip>, std::vector<KUKADU_SHARED_PTR<Clip> >, int> p2) {
                                      return std::get<0>(p1) > std::get<0>(p2);
                                  });

						if(!isShutUp)
							cout << "(ComplexController::executeInternal) i enumerated paths" << endl;

                        if(!isShutUp) {
                            cout << "nothing state clips: " << nothingStateClips[sensingClip->toString()].size() << endl;
							for(auto& c : nothingStateClips[sensingClip->toString()]) {
								if(c.second) {
									cout << "nothing state clip: " << c.first << " " << c.second << endl;
									cout << *(c.second) << endl;
								}
							}
                        }

                        // printPaths(possiblePaths);

                        for(auto&  path : possiblePaths) {

                            bool pathCanBeChosen = false;
                            double nothingProb = 0.0;
                            auto& resultingState = std::get<1>(path);
                            auto& clipPath = std::get<2>(path);



                            auto& senseNothingStateClips = nothingStateClips[sensingClip->toString()];
                            //cout << "senseNothingStateClips: " << senseNothingStateClips.size() << endl;

                            for(auto& nothingStateClip : senseNothingStateClips) {

                                /*
                                // found a proper transition - it may be added as a new preparatory controller
                                cout << "nothing state clip: " << nothingStateClip.second->toString() << endl;
                                cout << "resulting state clip: " << resultingState->toString() << endl;
                                cout << "(current) state clip: " << stateClip->toString() << endl;
                                */

                                if(resultingState == nothingStateClip.second && resultingState != stateClip && !hasDuplicateStatesInPath(clipPath)) {

                                    //allPaths.push_back(std::make_tuple(nextConfidence, resultingStateClip, currentPath, std::min(std::get<3>(path), std::get<2>(stateTransition))));
                                    get<2>(path).push_back(resultingState->getLikeliestChild());


                                    // TODO: this info is wrong (must be resultingState --> likeliest action)
                                    get<2>(path).push_back(resultingState);
                                    get<1>(path) = resultingState;


                                    /*
                                    cout << "I got a chosen path: ";
                                    printPath(path);
                                    */

                                    pathCanBeChosen = true;
                                    nothingProb = std::get<1>(resultingState->getMaxProbability());
                                    break;

                                }

                            }

                            // path has to contain more than 1 preparatory action (path here is state -> prep action -> state -> ... -> final state). therefore length
                            // must be at least 4
                            if(clipPath.size() >= 4) {

                                if(!isShutUp) {
                                    cout << "(ComplexController::executeInternal) i found a path that may be chosen with score " << nothingProb << endl;
                                    if(ASK_GETCHAR_CONF) getchar();
                                }

                                if(pathCanBeChosen) {

                                    // compute probability for adding a new clip from clip composition (will be referred as creativity in the paper)
                                    double pathConfidence = std::get<0>(path);
                                    double creativityProb = sigmoid(creativityGamma * pathConfidence * nothingProb + creativityDelta);
                                    //cout << "the clip is going to be created with probability " << creativityProb << endl; if(ASK_GETCHAR_CONF) getchar();
                                    KUKADU_DISCRETE_DISTRIBUTION<int> creativityDist({1.0 - creativityProb, creativityProb});
                                    int beCreative = creativityDist(*generator);

                                    if(!isShutUp) {
                                        cout << "(ComplexController::executeInternal) creative: " << beCreative << endl;
                                    }

                                    if(beCreative) {

                                        // extract controller from clip path
                                        int pathSize = clipPath.size();
                                        vector<KUKADU_SHARED_PTR<Controller> > controllerPath;
                                        for(int i = 1; i < pathSize; i += 2) {
                                            auto currentActionClip = KUKADU_DYNAMIC_POINTER_CAST<ControllerActionClip>(clipPath.at(i));
                                            controllerPath.push_back(currentActionClip->getActionController());
                                        }

                                        auto newConcatController = make_shared<ConcatController>(getStorage(), controllerPath);
                                        const auto& controllerLabel = newConcatController->getCaption();

                                        auto creativeActionClipInEcm = projSim->findClipInLevelByLabel(controllerLabel, Clip::CLIP_H_LEVEL_FINAL);
                                        // if clip is already in ecm, (for now) leave it as it is
                                        if(creativeActionClipInEcm) {

                                        }
                                        // else insert it to the skill ecm and the environment ecm and connect it properly
                                        else {

                                            // first create action clip
                                            auto newConcatClip = make_shared<ControllerActionClip>(getStorage(), projSim->generateNewActionId(), newConcatController, generator, getStoreSensorData(), getStoreMetaData());

                                            if(!isShutUp)
												cout << "creative proposal of " << newConcatClip->toString() << endl;

                                            bool isCorrectSensingClip = false;
                                            auto senseLayer = projSim->getClipLayers()->at(1);
                                            for(auto& currentSenseClip : *senseLayer) {

                                                // if the sensing clip is the same as the currently observed one, the new action should be connected strongly in here
                                                if(currentSenseClip == sensingClip) isCorrectSensingClip = true;
                                                // connect it with h_init
                                                else isCorrectSensingClip = false;

                                                for(auto& currentStateClip : *currentSenseClip->getSubClips()) {

                                                    // first add it to skill ecm...
                                                    // if we are at the currently observed state --> connect it strongly
                                                    if(isCorrectSensingClip && currentStateClip == stateClip)
                                                        currentStateClip->addSubClip(newConcatClip, stdPrepWeight * (1.0 + creativityMultiplier * pathConfidence));
                                                    else
                                                        currentStateClip->addSubClip(newConcatClip, stdPrepWeight);

                                                    projSim->addActionClip(newConcatClip);

                                                    // ...then add it to environment model ecm
                                                    auto idVec = make_shared<vector<int> >(2);
                                                    idVec->at(0) = currentStateClip->getClipDimensions()->at(0);
                                                    idVec->at(1) = newConcatClip->getClipDimensions()->at(0);

                                                    auto& currentEnvModel = environmentModels[currentSenseClip->toString()];
                                                    stringstream s;
                                                    s << "(E" << idVec->at(0) << ",P" << idVec->at(1) << ")";
                                                    auto newPercept = make_shared<PerceptClip>(currentEnvModel->generateNewPerceptId(), s.str(), generator, idVec, INT_MAX);
                                                    auto firstEnvModelPercept = currentEnvModel->getPerceptClips()->front();
                                                    auto children = firstEnvModelPercept->getSubClips();
                                                    auto childrenCopy = make_shared<vector<KUKADU_SHARED_PTR<Clip> > >(children->begin(), children->end());
                                                    newPercept->setChildren(childrenCopy);
                                                    currentEnvModel->addPerceptClip(newPercept);

                                                    stringstream s2; s2 << "E" << idVec->at(0);
                                                    auto envChildEnhanceClip = currentEnvModel->findClipInLevelByLabel(s2.str(), 1);
                                                    newPercept->setSpecificWeight(envChildEnhanceClip, get<3>(path));

                                                }

                                            }

                                            creativeControllerCreated = true;
                                            break;

                                        }

                                    } else {
                                        // it didn't chose to be creative
                                    }

                                    // only the most confident path can be selected as a creatively created clip (so after the first one that fulfills the properties, the execution is stopped)
                                    break;

                                }

                            }

                        }

                    }

                }

                // continue last walk if it was not bored
                walkRet = projSim->performRandomWalk(ProjectiveSimulator::PS_WALK_UNTIL_END, true);
                consecutiveBoredomCount = 0;

                hopPath = projSim->getIntermediateHopIdx();
                newClips = extractClipsFromPath(*hopPath);
                sensingClip = get<0>(newClips);
                stateClip = get<1>(newClips);
                stateId = stateClip->getClipDimensions()->at(0);
                auto actionClip = get<2>(newClips);
                auto actionId = actionClip->getClipDimensions()->at(0);

                int executeIt = 0;
                if(!getSimulationMode()) {
                    auto sensedLabel = getClassLabel(sensingClip, stateClip);
                    
                    if(!isShutUp)
						cout << "(ComplexController::execute) selected sensing action \"" << *sensingClip << "\" predicted class " << sensedLabel <<
								" and selected preparation action \"" << *actionClip << "\"" << endl;

                    cout << "(ComplexController::execute) do you want to execute it? (0 = no, 1 = yes)" << endl;
                    cin >> executeIt;
//                    executeIt = 1;
                } else
                    executeIt = 1;

                auto sensedState = stateClip;
                if(executeIt == 1) {

                    auto actionController = KUKADU_DYNAMIC_POINTER_CAST<ControllerActionClip>(actionClip);
                    if(!isShutUp)
						cout << actionController->getCaption() << endl;
                    actionController->execute();

                    // no environment model measuring in case the nothing controller was used
                    // by definition it does not change the state
                    // also don't do environment model update for controller that change the grasp type (would destroy a used grasp again)
                    if(actionClip->toString() != nothingController->getCaption() && this->requiresGrasp() == actionController->producesGrasp()) {

                        auto sensingController = sensingClip->getSensingController();

                        // if simulation mode, retrieve ground truth
                        KUKADU_SHARED_PTR<Clip> groundTruthStartClip;
                        if(getSimulationMode()) {
                            auto groundTruthIdx = sensingController->getSimulationGroundTruthIdx();
                            groundTruthStartClip = sensingClip->getSubClipByIdx(groundTruthIdx);
                        }

                        // not bored, the preparatory action was executed --> so sense again in order to improve environment model
                        // if simulation mode --> set new ground truth after the execution
                        if(getSimulationMode()) {

                            auto groundTruthStateClip = computeGroundTruthTransition(sensingClip, groundTruthStartClip, actionClip);

                            if(!isShutUp)
                                cout << "ground truth: " << *groundTruthStartClip << " (predicted: " << *stateClip << ") + " << *actionClip << " = " << *groundTruthStateClip << endl;

                            sensingClip->getSensingController()->setSimulationGroundTruth(sensingClip->getSubClipIdx(groundTruthStateClip));

                        }

                        // check state after preparatory action
                        int resultingStateChildIdx = sensingController->performClassification();

                        sensedState = sensingClip->getSubClipByIdx(resultingStateChildIdx);
                        int resultingStateId = sensedState->getClipDimensions()->at(0);

                        if(!getSimulationMode()) {
                            auto sensedLabel = getClassLabel(sensingClip, sensedState);
                            cout << "(ComplexController::execute) classifier result is category " << sensedLabel << endl;
                        }

                        vector<int> stateVector{stateId, actionId};

						if(!isShutUp)
							cout << "updating environment model for: " << sensingClip->toString() << endl;

                        auto currentEnvModel = environmentModels[sensingClip->toString()];
                        auto environmentClip = currentEnvModel->retrieveClipsOnLayer(stateVector, 0).at(0);

                        if(!isShutUp)
                            cout << "(" << stateId << ", " << actionId << ") - " << *environmentClip << " --> " << "E" << resultingStateId << " (idx: " << resultingStateChildIdx << ")" << endl;

                        auto resultingEnvironmentClip = currentEnvModel->retrieveClipsOnLayer({-resultingStateId, -resultingStateId}, 1).at(0);

						if(!isShutUp)
							cout << "rewarding clip: " << *resultingEnvironmentClip << endl;

                        vector<KUKADU_SHARED_PTR<Clip> > envClipPath{environmentClip, resultingEnvironmentClip};
                        currentEnvModel->setNextPredefinedPath(envClipPath);
                        currentEnvModel->performRandomWalk();
                        currentEnvModel->performRewarding();
                        if(!isShutUp)
							cout << "i am updating" << endl;
                        currentEnvModel->updatePsFile();
                        if(!isShutUp)
							cout << "updating done" << endl;

                    }

                    executeComplexAction();
                    cleanupAfterAction();

                }

                // after doing everything --> perform the complex action and reward it accordingly
                auto rewRet = projSim->performRewarding();
                reward = get<1>(rewRet);

                lastSkillWasSuccessful = (reward > 0) ? true : false;

                // block for determining the "nothing" states is only required if creativity is switched on
                if(useCreativity) loadTargetClips(sensingClip, sensedState);

            } else {

                if(!isShutUp)
					cout << "(ComplexController) i am bored - i will try to produce another perceptual state" << endl;

                ++consecutiveBoredomCount;
                walkRet = projSim->performRandomWalk(ProjectiveSimulator::PS_WALK_UNTIL_END, true);

                auto stateClip = walkRet.second;
                auto sensingClip = *(stateClip->getParents()->begin());
                auto sensingController = KUKADU_DYNAMIC_POINTER_CAST<IntermediateEventClip>(sensingClip)->getSensingController();

                // it was bored
                auto possiblePaths = computeEnvironmentPaths(sensingClip, stateClip, maxEnvPathLength, 0.4);
                computeTotalPathCost(KUKADU_DYNAMIC_POINTER_CAST<IntermediateEventClip>(sensingClip), possiblePaths);
                std::sort(possiblePaths.begin(), possiblePaths.end(), [] (std::tuple<double, KUKADU_SHARED_PTR<Clip>, std::vector<KUKADU_SHARED_PTR<Clip> >, int> p1, std::tuple<double, KUKADU_SHARED_PTR<Clip>, std::vector<KUKADU_SHARED_PTR<Clip> >, int> p2) {
                              return std::get<0>(p1) > std::get<0>(p2);
                          });

                selectedPath = possiblePaths.at(0);
                for(auto cl : possiblePaths) {
                    auto targetPercept = get<1>(cl);
                    if(*targetPercept != *stateClip) {
                        selectedPath = cl;
                        break;
                    }
                }

                if(!isShutUp) {
                    cout << "(ComplexController) got bored" << endl;
                    cout << "selected path info:" << endl;
                    cout << "source clip: " << *stateClip << endl;
                    cout << "target clip: " << *get<1>(selectedPath) << endl;
                    cout << "selected path: ";
                    for(auto cl : get<2>(selectedPath))
                        cout << *cl << " - ";
                    cout << endl;
                }

                // if controller is in real execution mode, execute the preparatory path
                if(!getSimulationMode()) {

                    // executing path
                    auto path = get<2>(selectedPath);
                    for(int i = 0; i < path.size(); ++i) {
                        auto cl = path.at(i);
                        // if clip number is odd, that clip is a preparatory action
                        if(i % 2) {
                            if(!isShutUp)
                                cout << "(ComplexController) next action is " << *cl;
                            KUKADU_DYNAMIC_POINTER_CAST<ControllerActionClip>(cl)->execute();
                        } else {
                            if(!isShutUp)
                                cout << "(ComplexController) current state should be " << *cl;
                        }
                    }

                } else {

                    // if controller is in simulation mode, check ground truth

                    // retrieve ground truth
                    KUKADU_SHARED_PTR<Clip> groundTruthStartClip;
                    if(getSimulationMode()) {
                        auto groundTruthIdx = sensingController->getSimulationGroundTruthIdx();
                        groundTruthStartClip = sensingClip->getSubClipByIdx(groundTruthIdx);
                    }

                    auto groundTruthStateClip = groundTruthStartClip;
                    auto path = get<2>(selectedPath);
                    for(int i = 0; i < path.size(); ++i) {
                        auto cl = path.at(i);
                        // if clip number is odd, that clip is a preparatory action
                        if(i % 2) {
                            if(!isShutUp)
                                cout << "(ComplexController) next action is " << *cl;
                            groundTruthStateClip = computeGroundTruthTransition(sensingClip, groundTruthStateClip, cl);
                        } else {
                            if(!isShutUp)
                                cout << "(ComplexController) current state should be " << *cl;
                        }
                    }

                    // setting new ground truth state
                    auto castedSensingClip = KUKADU_DYNAMIC_POINTER_CAST<IntermediateEventClip>(sensingClip);
                    auto sensingController = castedSensingClip->getSensingController();
                    int groundTruthIdx = sensingClip->getSubClipIdx(groundTruthStateClip);
                    sensingController->setSimulationGroundTruth(groundTruthIdx);

                }

                // new state is present now --> perform action again without boredom should be enough
                this->setBoredom(false);

                // perform action again
                auto prevGenerateNewGroundTruth = generateNewGroundTruth;
                setGenerateNewGroundTruth(false);
                ret = execute();

                setGenerateNewGroundTruth(prevGenerateNewGroundTruth);
                KUKADU_DYNAMIC_POINTER_CAST<HapticControllerResult>(ret)->setWasBored(true);

                // switching on boredom again
                this->setBoredom(true);

            }

            auto selectedPathPointer = KUKADU_SHARED_PTR<std::tuple<double, KUKADU_SHARED_PTR<kukadu::Clip>, std::vector<KUKADU_SHARED_PTR<kukadu::Clip> > > >(new std::tuple<double, KUKADU_SHARED_PTR<kukadu::Clip>, std::vector<KUKADU_SHARED_PTR<kukadu::Clip> > >(get<0>(selectedPath), get<1>(selectedPath), get<2>(selectedPath)));

            // this behaviour could be improved --> TODO
            if(!ret)
                ret = make_shared<HapticControllerResult>(vec(), vector<vec>(), (reward > 0.0) ? true : false, false, *(projSim->getIntermediateHopIdx()), selectedPathPointer);

        }

        KUKADU_MODULE_END_USAGE();
        return ret;

    }

    bool ComplexController::hasDuplicateStatesInPath(std::vector<KUKADU_SHARED_PTR<Clip> >& path) {

        int pathLength = path.size();
        std::set<KUKADU_SHARED_PTR<Clip> > containedClips;
        for(int i = 0; i < pathLength; i += 2) {
            auto& stateClip = path.at(i);
            // if element is already in the set, containedClips.insert(stateClip).second will be false
            if(!containedClips.insert(stateClip).second)
                return true;
        }
        return false;

    }

    void ComplexController::printPath(std::tuple<double, KUKADU_SHARED_PTR<Clip>, std::vector<KUKADU_SHARED_PTR<Clip> >, int>& path) {
        cout << get<0>(path) << "; " << *get<1>(path) << ": ";
        for(auto cl : get<2>(path))
            cout << *cl << " - ";
        cout << endl;
    }

    void ComplexController::printPaths(std::vector<std::tuple<double, KUKADU_SHARED_PTR<Clip>, std::vector<KUKADU_SHARED_PTR<Clip> >, int> >& paths) {
        for(auto path : paths) {
            printPath(path);
        }
    }

    void ComplexController::computeTotalPathCost(KUKADU_SHARED_PTR<IntermediateEventClip> sensingClip,
                                                 std::vector<std::tuple<double, KUKADU_SHARED_PTR<Clip>,
                                                 std::vector<KUKADU_SHARED_PTR<Clip> >, int> >& paths) {

        KUKADU_MODULE_START_USAGE();

        for(auto& path : paths) {

            double& pathCost = std::get<0>(path);
            auto finalStateClip = std::get<1>(path);
            auto& clipPath = std::get<2>(path);

            // active learning version
            double finalClipEntropy = finalStateClip->computeSubEntropy();

            // intrinsic motivation version
            //auto sensingController = sensingClip->getSensingController();
            //auto meanAndVariance = computeEntropyMeanAndVariance({sensingController})[sensingController->getCaption()];
            //double finalClipEntropy = std::exp(-std::pow(finalStateClip->computeSubEntropy() - get<0>(meanAndVariance), 2.0) / (2.0 * get<1>(meanAndVariance)));

            // there are state clips and action clips mixed - the first and the last clips are state clips, so the
            // size has to be reduced by one --> then the number of taken actions is half of that size
            int pathLength = (clipPath.size() - 1) / 2 + 1;

            // update the cost of the path
            pathCost = finalClipEntropy * pathCost + pathLengthCost / pathLength;

        }

        KUKADU_MODULE_END_USAGE();

    }

    std::vector<std::tuple<double, KUKADU_SHARED_PTR<Clip>, std::vector<KUKADU_SHARED_PTR<Clip> >, int> > ComplexController::computeEnvironmentPaths(
            KUKADU_SHARED_PTR<Clip> sensingClip, KUKADU_SHARED_PTR<Clip> stateClip, int maxPathLength, double confidenceCut) {

        KUKADU_MODULE_START_USAGE();

        auto sensingId = sensingClip->toString();
        auto stateId = stateClip->getClipDimensions()->at(0);
        std::vector<std::tuple<double, KUKADU_SHARED_PTR<Clip>, std::vector<KUKADU_SHARED_PTR<Clip> >, int> > allPaths;
        std::vector<std::tuple<double, KUKADU_SHARED_PTR<Clip>, std::vector<KUKADU_SHARED_PTR<Clip> >, int> > lastIterationPaths;
        std::vector<std::tuple<double, KUKADU_SHARED_PTR<Clip>, std::vector<KUKADU_SHARED_PTR<Clip> >, int> > lastIterationPathsOld;
        // initialize with paths of length 0
        std::vector<KUKADU_SHARED_PTR<Clip> > path = {stateClip};
        allPaths.push_back(std::make_tuple(1.0, stateClip, path, INT_MAX));
        lastIterationPaths.push_back(std::make_tuple(1.0, stateClip, path, INT_MAX));
        for(int i = 0; i < maxPathLength; ++i) {
            lastIterationPathsOld = lastIterationPaths;
            lastIterationPaths.clear();
            // check every path and see how it can be made longer
            for(auto path : lastIterationPathsOld) {
                double currentConfidence = std::get<0>(path);
                auto currentState = std::get<1>(path);
                stateId = currentState->getClipDimensions()->at(0);
                // for every possible transition, analyse how confidently another state can be reached
                auto stateClips =  environmentModels[sensingId]->retrieveClipsOnLayer({stateId, ProjectiveSimulator::IGNORE_ID}, 0);
                for(auto state : stateClips) {
                    // copy old path again
                    auto currentPath = std::get<2>(path);
                    auto stateTransition = computeEnvironmentTransitionConfidence(state);
                    double transitionConfidence = std::get<0>(stateTransition);
                    auto resultingStateId = std::get<1>(stateTransition);
                    auto resultingStateClip = projSim->retrieveClipsOnLayer({resultingStateId}, 2).at(0);
                    int actionId = state->getClipDimensions()->at(1);
                    auto usedActionClip = projSim->retrieveClipsOnLayer({actionId}, 3).at(0);
                    currentPath.push_back(usedActionClip);
                    currentPath.push_back(resultingStateClip);
                    double nextConfidence = currentConfidence * transitionConfidence;
                    if(nextConfidence > confidenceCut) {
                        allPaths.push_back(std::make_tuple(nextConfidence, resultingStateClip, currentPath, std::min(std::get<3>(path), std::get<2>(stateTransition))));
                        lastIterationPaths.push_back(std::make_tuple(nextConfidence, resultingStateClip, currentPath, std::min(std::get<3>(path), std::get<2>(stateTransition))));
                    }
                }
            }
        }

        KUKADU_MODULE_END_USAGE();

        return allPaths;

    }

    std::tuple<double, int, int> ComplexController::computeEnvironmentTransitionConfidence(KUKADU_SHARED_PTR<Clip> stateClip) {

        KUKADU_MODULE_START_USAGE();

        auto likeliest = stateClip->getLikeliestChildWithWeight();
        double stateEntropy = stateClip->computeSubEntropy();
        double confidenceProb = 1.0 - stateEntropy / log2(stateClip->getSubClipCount());
        auto likeliestResult = likeliest.second;
        int envId = atoi(likeliestResult->toString().substr(1).c_str());

        KUKADU_MODULE_END_USAGE();

        return std::make_tuple(confidenceProb, envId, likeliest.first);

    }

    bool ComplexController::setUseCreativity(bool useCreativity) {
        KUKADU_MODULE_START_USAGE();
        this->useCreativity = useCreativity;
        KUKADU_MODULE_END_USAGE();
    }

    void ComplexController::setTrainingMode(bool doTraining) {
        KUKADU_MODULE_START_USAGE();
        projSim->setTrainingMode(doTraining);
        KUKADU_MODULE_END_USAGE();
    }

    void ComplexController::createSensingDatabase() {

        KUKADU_MODULE_START_USAGE();

        createSensingDatabase(sensingControllers);

        KUKADU_MODULE_END_USAGE();

    }

    void ComplexController::createSensingDatabase(std::vector<KUKADU_SHARED_PTR<SensingController> > sensingControllers) {

        KUKADU_MODULE_START_USAGE();

        sensingWeights.clear();
        for(int i = 0; i < sensingControllers.size(); ++i) {
            double validationScore = sensingControllers.at(i)->createDataBase();
            sensingWeights.push_back(validationScore);
        }

        KUKADU_MODULE_END_USAGE();

    }

    EnvironmentReward::EnvironmentReward(KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, double stdReward) : Reward(generator, false) {
        reward = stdReward;
    }

    double EnvironmentReward::computeRewardInternal(KUKADU_SHARED_PTR<PerceptClip> providedPercept, KUKADU_SHARED_PTR<ActionClip> takenAction) {
        // all paths are rewarded, because only observed paths are performed
        return reward;
    }

    int EnvironmentReward::getDimensionality() {
        return 2;
    }

    KUKADU_SHARED_PTR<PerceptClip> EnvironmentReward::generateNextPerceptClip(int immunity) {
        return nullptr;
    }

    KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<ActionClip> > > EnvironmentReward::generateActionClips() {
        return KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<ActionClip> > >(new std::vector<KUKADU_SHARED_PTR<ActionClip> >());
    }

    KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<PerceptClip> > > EnvironmentReward::generatePerceptClips() {
        return KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<PerceptClip> > >(new std::vector<KUKADU_SHARED_PTR<PerceptClip> >());
    }

    HapticControllerResult::HapticControllerResult(arma::vec t, std::vector<arma::vec> ys, bool success, bool bored, std::vector<int> walkedPath,
                                                   KUKADU_SHARED_PTR<std::tuple<double, KUKADU_SHARED_PTR<kukadu::Clip>, std::vector<KUKADU_SHARED_PTR<kukadu::Clip> > > > environmentTransition)
        : ControllerResult(t, ys, success) {

        this->bored = bored;
        this->walkedPath = walkedPath;
        this->environmentTransition = environmentTransition;

    }

    bool HapticControllerResult::wasBored() {
        return bored;
    }

    std::vector<int> HapticControllerResult::getWalkedPath() {
        return walkedPath;
    }

    void HapticControllerResult::setEntropyMeanAndVariance(std::map<std::string, std::pair<double, double> > meanAndVar) {
        this->meanAndVar = meanAndVar;
    }

    std::map<std::string, std::pair<double, double> > HapticControllerResult::getMeanAndVar() {
        return meanAndVar;
    }

    void HapticControllerResult::setEntropies(std::map<std::string, std::vector<double> > entropies) {
        this->entropies = entropies;
    }

    std::map<std::string, std::vector<double> > HapticControllerResult::getEntropies() {
        return entropies;
    }

    int HapticControllerResult::getFinalStateClass() {
        return *(getWalkedPath().end() - 2);
    }

    void HapticControllerResult::setWasBored(bool wasBored) {
        bored = wasBored;
    }

    void ConcatController::createSkillFromThisInternal(std::string skillName) {
        throw KukaduException("(ConcatController) creating a skill from instance not supported yet");
    }

    std::string ConcatController::generateLabelFromControllers(std::vector<KUKADU_SHARED_PTR<kukadu::Controller> >& controllers) {

        stringstream retLabel;
        bool first = true;
        for(auto& cont : controllers) {
            if(first)
                first = false;
            else retLabel << "$";
            retLabel << cont->getCaption();
        }
        return retLabel.str();

    }

    int computeSimulationFailingProbability(std::vector<KUKADU_SHARED_PTR<kukadu::Controller> >& controllers) {

        int totalSuccProb = 1.0;
        for(auto cont : controllers)
            if(cont)
                totalSuccProb *= 1.0 - cont->getSimFailingProb();
        return 1.0 - totalSuccProb;

    }

    ConcatController::ConcatController(StorageSingleton& dbStorage, std::vector<KUKADU_SHARED_PTR<kukadu::Controller> > controllers, const bool storeSensorData, const bool storeMetaData)
        : Controller(dbStorage, generateLabelFromControllers(controllers), mergeHardware(controllers), computeSimulationFailingProbability(controllers), storeSensorData, storeMetaData) {
        this->controllers = controllers;
    }

    bool ConcatController::getSimulationMode() {
        for(auto cont : controllers)
            if(!cont->getSimulationMode())
                return false;
        return true;
    }

    bool ConcatController::requiresGraspInternal() {

        return controllers.front()->producesGrasp();

    }

    bool ConcatController::producesGraspInternal() {

        return controllers.back()->producesGrasp();

    }

    KUKADU_SHARED_PTR<ControllerResult> ConcatController::executeInternal() {

        KUKADU_MODULE_START_USAGE();

        KUKADU_SHARED_PTR<ControllerResult> lastResult;
        // execute all controllers
        for(auto& cont : controllers)
            lastResult = cont->execute();

        KUKADU_MODULE_END_USAGE();

        // dummy for now takes only the last result
        return lastResult;

    }

    IntermediateEventClip::IntermediateEventClip(KUKADU_SHARED_PTR<SensingController> sensingEvent,
                      int level, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, std::string clipValues, int immunity) : Clip(level, generator, clipValues, immunity) {

        this->caption = sensingEvent->getCaption();
        this->sensingEvent = sensingEvent;

    }

    IntermediateEventClip::IntermediateEventClip(KUKADU_SHARED_PTR<SensingController> sensingEvent,
                                                 int level, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, KUKADU_SHARED_PTR<std::vector<int> > clipValues, int immunity)
        : Clip(level, generator, clipValues, immunity) {

        this->caption = sensingEvent->getCaption();
        this->sensingEvent = sensingEvent;

    }

    std::pair<int, KUKADU_SHARED_PTR<Clip> > IntermediateEventClip::jumpNextRandom() {

        KUKADU_MODULE_START_USAGE();

        pair<int, KUKADU_SHARED_PTR<Clip> > retVal;
        visitedSubNode = retVal.first = sensingEvent->performClassification();
        retVal.second = getSubClipByIdx(retVal.first);

        KUKADU_MODULE_END_USAGE();

        return retVal;

    }

    std::string IntermediateEventClip::toString() const {
        return sensingEvent->getCaption();
    }

    KUKADU_SHARED_PTR<SensingController> IntermediateEventClip::getSensingController() {
        return sensingEvent;
    }

    ControllerActionClip::ControllerActionClip(StorageSingleton& dbStorage, int actionId, KUKADU_SHARED_PTR<Controller> actionController,
                          KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator,
                          const bool storeSensorData, const bool storeMetaData) :
        ActionClip(actionId, 1, actionController->getCaption(), generator),
        Controller(dbStorage, actionController->getCaption(), actionController->getUsedHardware(), actionController->getSimFailingProb(), storeSensorData, storeMetaData) {

        this->actionController = actionController;

    }

    void ControllerActionClip::createSkillFromThisInternal(std::string skillName) {
        throw KukaduException("(ControllerActionClip) creating a skill from instance not supported yet");
    }

    bool ControllerActionClip::requiresGraspInternal() {
        return actionController->requiresGrasp();
    }

    bool ControllerActionClip::producesGraspInternal() {
        return actionController->producesGrasp();
    }

    KUKADU_SHARED_PTR<ControllerResult> ControllerActionClip::executeInternal() {

        KUKADU_MODULE_START_USAGE();

        KUKADU_SHARED_PTR<ControllerResult> retVal;

        if(!actionController->getSimulationMode()) {

            int executeIt = 1;
            cout << "(ControllerActionClip) selected preparation action is \"" << actionController->getCaption() << "\"" << endl;
            cin >> executeIt;

            if(executeIt == 1)
                retVal = actionController->execute();
            else
                cout << "(ControllerActionClip) you decided not to perform the action; continue" << endl;

        }

        KUKADU_MODULE_END_USAGE();

        return retVal;

    }

    KUKADU_SHARED_PTR<Controller> ControllerActionClip::getActionController() {
        return actionController;
    }

    std::string ControllerActionClip::toString() const {
        return actionController->getCaption();
    }

    SensingController::SensingController(StorageSingleton& storage, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, int hapticMode, string caption,
                                         std::vector<KUKADU_SHARED_PTR<ControlQueue> > queues, vector<KUKADU_SHARED_PTR<GenericHand> > hands,
                                         std::string tmpPath, double simClassificationPrecision,
                                         std::vector<KUKADU_SHARED_PTR<Hardware> > allHardware, const bool storeSensorData, const bool storeMetaData, const bool requiresDatabaseFlag)
        : Controller(storage,
                     caption,
                     (allHardware.size()) ? allHardware : flatten<KUKADU_SHARED_PTR<Hardware> >({castVector<KUKADU_SHARED_PTR<ControlQueue>, KUKADU_SHARED_PTR<Hardware> >(queues)}),
                     0.0,
                     storeSensorData,
                     storeMetaData),
          dbStorage(storage) {

        this->requiresDatabaseFlag = requiresDatabaseFlag;
        currentIterationNum = 0;
        simulationGroundTruth = 0;
        simulatedClassificationPrecision = simClassificationPrecision;

        this->generator = generator;

        this->hands = hands;
        this->queues = queues;
        this->tmpPath = tmpPath;
        this->hapticMode = hapticMode;
        this->classifierFile = classifierFile;
        this->classifierPath = classifierPath;
        this->classifierFunction = classifierFunction;

        this->stateCount = 4;

        databaseAlreadySet = false;

    }

    void SensingController::createSkillFromThisInternal(std::string skillName) {
    }

    KUKADU_SHARED_PTR<kukadu_mersenne_twister> SensingController::getGenerator() {
        return generator;
    }

    std::vector<KUKADU_SHARED_PTR<ControlQueue> > SensingController::getQueues() {
        return queues;
    }

    std::vector<KUKADU_SHARED_PTR<GenericHand> > SensingController::getHands() {
        return hands;
    }

    std::string SensingController::getTmpPath() {
        return tmpPath;
    }

    int SensingController::getHapticMode() {
        return hapticMode;
    }

    double SensingController::getSimClassificationPrecision() {
        return simulatedClassificationPrecision;
    }

    void SensingController::gatherData(std::string dataBasePath, std::string dataName) {

        KUKADU_MODULE_START_USAGE();
        
        if(getSimulationMode()) {

			if(!fileExists(dataBasePath))
				createDirectory(dataBasePath);

			gatherData(dataBasePath + dataName);
		}

        KUKADU_MODULE_END_USAGE();

    }

    std::string SensingController::getDatabasePath() {
        return databasePath;
    }

    void SensingController::setDatabasePath(std::string databasePath) {
        this->databasePath = databasePath;
        databaseAlreadySet = true;
    }

    void SensingController::gatherData(std::string completePath) {

        KUKADU_MODULE_START_USAGE();
        
        if(getSimulationMode()) {

			vector<KUKADU_SHARED_PTR<ControlQueue> > castedQueues;
			for(int i = 0; i < queues.size(); ++i) {
				KUKADU_SHARED_PTR<ControlQueue> queue = queues.at(i);
				castedQueues.push_back(queue);
			}

			SensorStorage store(dbStorage, castedQueues, hands, 100);

			prepare();

			store.startDataStorage(completePath);
			performCore();
			store.stopDataStorage();
		}

        KUKADU_MODULE_END_USAGE();

    }

    int SensingController::getStateCount() {
        return stateCount;
    }

    void SensingController::setStateCount(const int& stateCount) {
        this->stateCount = stateCount;
    }

    std::string SensingController::getFirstRobotFileName() {
        return queues.at(0)->getRobotFileName();
    }

    int SensingController::callClassifier() {

        if(!databaseAlreadySet)
            throw KukaduException("(SensingController::callClassifier) database not defined yet");

        return callClassifier(tmpPath + "hapticTest/" + queues.at(0)->getRobotFileName() + "_0");

    }

    int SensingController::performClassification() {

        KUKADU_MODULE_START_USAGE();

        if(!databaseAlreadySet)
            throw KukaduException("(SensingController::performClassification) database not defined yet");

        int classifierRes = -1;

        KUKADU_SHARED_PTR<kukadu_thread> cleanupThread;
        
        if(!isShutUp)
			cout << "simulation mode: " << getSimulationMode() << endl;
        
        if(!getSimulationMode()) {

            int executeIt = 1;
            int temporaryHapticMode = hapticMode;
            
            if(!isShutUp)
				cout << "(SensingController) selected sensing action is \"" << getCaption() << "\"" << endl;

            if(executeIt == 1) {

                pf::remove_all(tmpPath + "hapticTest");
                gatherData(tmpPath, "hapticTest");

                // start clean up in a separate thread
                cleanupThread = make_shared<kukadu_thread>(&SensingController::cleanUp, this);

                stringstream s;
                s << tmpPath << "hapticTest_" << queues.at(0)->getRobotFileName() << "_0_" << currentIterationNum;
                copyFile(tmpPath + "hapticTest/" + queues.at(0)->getRobotFileName() + "_0", s.str());

            } else {
                if(!isShutUp) {
                    cout << "(SensinController) you decided not to perform the action" << endl;
                    cout << "(SensinController) switching temporarily to haptic mode HAPTIC_MODE_TERMINAL; continue" << endl;
                }
                temporaryHapticMode = SensingController::HAPTIC_MODE_TERMINAL;
            }

            if(temporaryHapticMode == SensingController::HAPTIC_MODE_TERMINAL) {
                cout << "(SensingController) what was the haptic result? [0, " << (getStateCount() - 1) << "]" << endl;
                cin >> classifierRes;
            } else if(temporaryHapticMode == SensingController::HAPTIC_MODE_CLASSIFIER) {
                auto res = callClassifier(tmpPath + "hapticTest/" + queues.at(0)->getRobotFileName() + "_0");
                classifierRes = res;

/*
                cout << "is the result " << classifierRes << " correct? (1 = yes, 0 = no)" << endl;
                int corr;
                cin >> corr;
                if(!corr) {
                    cout << "what was the correct result?" << endl;
                    cin >> classifierRes;
                }
*/

            } else {
                throw KukaduException("haptic mode not known");
            }

            pf::remove_all(tmpPath + "hapticTest");
            ++currentIterationNum;


        } else {

            // this is here for simulating a non-perfect classifier
            vector<double> precisionProbVec;
            precisionProbVec.push_back((double) simulatedClassificationPrecision);
            precisionProbVec.push_back((double) (100 - simulatedClassificationPrecision));
            KUKADU_DISCRETE_DISTRIBUTION<int> precisionProb(precisionProbVec.begin(), precisionProbVec.end());

            int correctClass = precisionProb(*generator);

            // simulate correct classification
            if(!correctClass)
                classifierRes = simulationGroundTruth;
            else {
                // classify it wrongly (random)
                classifierRes = simulationGroundTruth;
                while(classifierRes == simulationGroundTruth)
                    classifierRes = createRandomGroundTruthIdx();
            }

        }

        if(cleanupThread && cleanupThread->joinable())
            cleanupThread->join();

        KUKADU_MODULE_END_USAGE();

        return classifierRes;

    }

    int SensingController::getSimulationGroundTruthIdx() {
        return simulationGroundTruth;
    }

    int SensingController::createRandomGroundTruthIdx() {
        vector<int> randValues;
        for(int i = 0; i < getStateCount(); ++i)
            randValues.push_back(1);
        classifierDist = KUKADU_DISCRETE_DISTRIBUTION<int>(randValues.begin(),randValues.end());
        return classifierDist(*generator);
    }

    void SensingController::setSimulationClassificationPrecision(int percent) {
        simulatedClassificationPrecision = percent;
    }

    void SensingController::setSimulationGroundTruth(int idx) {
        simulationGroundTruth = idx;
    }

    std::pair<std::vector<int>, std::vector<arma::mat> > loadClassificationData(std::vector<int> classIds, std::vector<std::string> fileNames) {

        vector<mat> sampleVectors;

        for(auto& fileName : fileNames) {

            auto sensorData = readDmpData(fileName, false).second;

            mat concatenatedData;
            // not very efficient
            for(int i = 0; i < sensorData.n_rows; ++i)
                concatenatedData = join_rows(concatenatedData, sensorData.row(i));
            sampleVectors.push_back(concatenatedData);

        }

        // find the minimum length of all files (thats the length we will be able to use)
        int minDim = std::numeric_limits<int>::max();
        for(auto& sample : sampleVectors)
            minDim = std::min(minDim, (int) sample.n_cols);

        vector<int> uniqueClassIds;
        vector<mat> samples;
        // initialize the data for all classes
        for(int i = 0; i < classIds.size(); ++i) {

            mat reducedSample = sampleVectors.at(i).cols(0, minDim - 1);
            auto currentClassId = classIds.at(i);
            auto currentIdIt = std::find(uniqueClassIds.begin(), uniqueClassIds.end(), currentClassId);

            if(currentIdIt == uniqueClassIds.end()) {

                uniqueClassIds.push_back(currentClassId);
                currentIdIt = uniqueClassIds.end() - 1;
                samples.push_back(reducedSample);

            } else {

                int currentIdIdx = (int) (currentIdIt - uniqueClassIds.begin());
                samples.at(currentIdIdx) = join_cols(samples.at(currentIdIdx), reducedSample);

            }

        }

        return {uniqueClassIds, samples};

    }

    void SensingController::setParentComplexController(KUKADU_SHARED_PTR<kukadu::Controller> parent) {
        parentController = KUKADU_DYNAMIC_POINTER_CAST<ComplexController>(parent);
    }

    double SensingController::createDataBase(int perceptualStateId) {

        KUKADU_MODULE_START_USAGE();

        if(requiresDatabase()) {

            if(!databaseAlreadySet)
                throw KukaduException("(SensingController::createDataBase) database not defined yet");

            int numClasses = getStateCount();
            
            if(!isShutUp)
				cout << "state count: " << numClasses << endl;
            
            string path = getDatabasePath();
            vector<pair<int, string> > collectedSamples;
            if(!isShutUp)
                cout << "(SensingController) data is stored to " << path << endl;

            bool databaseExisted = fileExists(path);
            if(!databaseExisted) {
                if(!isShutUp)
                    cout << "(SensingController) folder doesn't exist - create" << endl;
                createDirectory(path);
            }

            vector<int> sampleIds;
            for(int i = 0; i < numClasses; ++i)
                sampleIds.push_back(0);

            if(databaseExisted) {

				if(!isShutUp)
					cout << "labels path: " << (path + "/labels") << endl;

                ifstream labelsFile;
                labelsFile.open((path + "/labels").c_str());

                string line;
                while(getline(labelsFile, line)) {
                    KukaduTokenizer tok(line);
                    tok.next();
                    auto idStr = tok.next();
                    int classId = atoi(idStr.c_str()) - 1;
                    if(classId >= sampleIds.size()) throw runtime_error("Labels do not fit to number of classes");
                    sampleIds.at(classId)++;
                }

                labelsFile.close();

            }
            
            if(!isShutUp)
				cout << "sample id size: " << sampleIds.size() << endl;

            int minSampleCount = 10;
            for(int i = 0; i < sampleIds.size(); ++i) {
                if(sampleIds.at(i) == -1) {
                    perceptualStateId = i;
                    break;
                } else if(sampleIds.at(i) < minSampleCount) {
                    cout << "(SensingController) you have less than " << minSampleCount << " samples for state " << i << " under sensing action " << getCaption() <<
                            ". do you want to collect more? (0 = no, 1 = yes)" << endl;
                    int answer = 1;
//                    cin >> answer;
                    if(answer == 1)
                        perceptualStateId = i;
                    break;
                }

            }
            
            if(!isShutUp)
				cout << "creating database: " << databaseExisted << ", " << perceptualStateId << ", " << isShutUp << endl;

            if(!databaseExisted || perceptualStateId != -1) {

                // create the database
                if(!isShutUp)
                    cout << "(SensingController) " << getCaption() << " offers " << numClasses << " classes" << endl;

                ofstream labelFile;
                labelFile.open((path + "labels").c_str(), std::ios_base::app);

                if(perceptualStateId == -1)
                    perceptualStateId = 0;

                for(int currClass = perceptualStateId; currClass < numClasses; ++currClass) {

                    if(currClass != perceptualStateId)
                        KUKADU_DYNAMIC_POINTER_CAST<ComplexController>(parentController)->prepareNextState(shared_from_this(), currClass);

                    int cont = 1;
                    for(int sampleNum = sampleIds.at(currClass) + 1; cont == 1; ++sampleNum) {
						
						if(!isShutUp)
							cout << "(SensingController) collecting sample number " << sampleNum << " for class " << currClass << " with sensing controller " << this->getCaption() << endl;

                        stringstream s;
                        s << "class_" << currClass << "_sample_" << sampleNum;
                        string relativePath = s.str();
                        string relativeClassifyPath = relativePath + "/" + getFirstRobotFileName() + "_0";
                        string nextSamplePath = path + relativePath;
                        
                        if(!isShutUp)
							cout << "calling to gather data" << endl;
							
                        gatherData(nextSamplePath);
                        
                        if(!isShutUp)
							cout << "done gathering data" << endl;
                        cleanUp();
                        
                        if(!isShutUp)
							cout << "done cleaning up" << endl;

                        collectedSamples.push_back(pair<int, string>(currClass, relativeClassifyPath));
                        labelFile << relativeClassifyPath << " " << currClass << endl;

                        cout << "(SensingController) want to collect another sample for class " << currClass << "? (0 = no / 1 = yes): ";
                        cin >> cont;

                    }

                }

                labelFile.close();

            } else
                if(!isShutUp)
                    cout << "(SensingController) database for controller " << getCaption() << " exists - no collection required" << endl;

            ifstream labelsFile;
            labelsFile.open((path + "/labels").c_str());

            string line;
            vector<int> classIds;
            vector<string> fileNames;

            // load all the files and concatenated the rows for each file
            while(getline(labelsFile, line)) {
                KukaduTokenizer tok(line);
                auto filePath = tok.next();
                if(filePath != "") {
                    auto classId = atoi(tok.next().c_str());
                    classIds.push_back(classId);
                    fileNames.push_back(path + "/" + filePath);
                }
            }

            auto loadedData = loadClassificationData(classIds, fileNames);
            auto uniqueClassIds = loadedData.first;
            auto samples = loadedData.second;

            classifier = make_shared<LibSvm>(uniqueClassIds, samples);

            // if no classifier file exists
            if(!fileExists(path + "classRes")) {
				
				if(!isShutUp)
					cout << "(SensingController) determinining confidence..." << endl;
                double confidence = classifier->crossValidate();
                
                if(!isShutUp)
					cout << "(SensingController) found a confidence of " << confidence << endl;

                ofstream ofile;
                ofile.open((path + "classRes").c_str());
                ofile << confidence << endl;
                ofile.close();

            }

            classifier->train();

            ifstream infile;
            infile.open((path + "classRes").c_str());
            double confidence = 0.0;
            infile >> confidence;

            if(!isShutUp)
                cout << "(SensingController) determined a confidence of " << confidence << endl;

            return confidence;

        }

        return 1.0;

        KUKADU_MODULE_END_USAGE();

    }

    KUKADU_SHARED_PTR<ControllerResult> SensingController::executeInternal() {

        KUKADU_MODULE_START_USAGE();

		if(!getSimulationMode()) {

			prepare();
			performCore();
			cleanUp();
			
		}

        KUKADU_MODULE_END_USAGE();

        return KUKADU_SHARED_PTR<ControllerResult>();

    }

    arma::vec fitToDim(arma::mat v, int dim) {

        if(v.n_cols > dim)
            return v.cols(0, dim - 1).t();
        else {
            arma::vec retVec(dim);
            for(int i = 0; i < dim; ++i)
                if(i < v.n_cols)
                    retVec(i) = v(i);
                else
                    retVec(i) = v(v.n_cols - 1);
            return retVec;
        }

    }

    int SensingController::hardcodedClassification() {
        throw KukaduException("(SensingController) custom classification procedure has to be written if no database is needed");
        return 0;
    }

    int SensingController::callClassifier(std::string passedFilePath) {

        KUKADU_MODULE_START_USAGE();

        int res = 0;
        if(requiresDatabase()) {
            auto toClassify = loadClassificationData({0}, {passedFilePath});
            vec toClassifyVec = fitToDim(toClassify.second.front(), classifier->getSampleDimensionality());
            res = classifier->classify(toClassifyVec);
        } else {
            res = hardcodedClassification();
        }

        KUKADU_MODULE_END_USAGE();

        return res;

    }

    void SensingController::writeLabelFile(std::string baseFolderPath, std::vector<std::pair<int, std::string> > collectedSamples) {

        ofstream outFile;
        outFile.open((baseFolderPath + "labels").c_str());

        for(int i = 0; i < collectedSamples.size(); ++i) {
            pair<int, string> sample = collectedSamples.at(i);
            outFile << sample.second << " " << sample.first << endl;
        }

    }

    bool SensingController::requiresDatabase() {
        // by default sensing requires a database
        return requiresDatabaseFlag;
    }

}
