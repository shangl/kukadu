#include <tuple>
#include <armadillo>
#include <kukadu/utils/kukadutokenizer.hpp>
#include <kukadu/manipulation/controller.hpp>
#include <kukadu/manipulation/concatcontroller.hpp>
#include <kukadu/manipulation/complexcontroller.hpp>
#include <kukadu/manipulation/haptic/controlleractionclip.hpp>
#include <kukadu/manipulation/haptic/intermediateeventclip.hpp>
#include <kukadu/learning/projective_simulation/core/clip.hpp>
#include <kukadu/learning/projective_simulation/core/perceptclip.hpp>

using namespace std;
using namespace arma;

namespace kukadu {

#ifdef USEBOOST
    const std::string ComplexController::FILE_SENSING_PREFIX = "***sensing controllers:";
    const std::string ComplexController::FILE_PREP_PREFIX = "***preparatory controllers:";
    const std::string ComplexController::FILE_END_PREFIX = "***end";
    const std::string ComplexController::FILE_CONCAT_PREFIX = "***concatenated controllers:";
#endif

    ComplexController::ComplexController(std::string caption, std::string storePath,
                                         bool storeReward, double senseStretch, double boredom, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator,
                                         int stdReward, double punishReward, double gamma, int stdPrepWeight, bool collectPrevRewards,
                                         int simulationFailingProbability,
                                         KUKADU_SHARED_PTR<Controller> nothingController,
                                         int maxEnvPathLength, double pathLengthCost, double stdEnvironmentReward,
                                         double creativityAlpha1, double creativityAlpha2, double creativityBeta, double creativityCthresh,
                                         double nothingStateProbThresh, double creativityMultiplier)
        : Controller(caption, simulationFailingProbability), Reward(generator, collectPrevRewards) {

        this->useCreativity = false;
        this->creativityAlpha1 = creativityAlpha1;
        this->creativityAlpha2 = creativityAlpha2;
        this->creativityBeta = creativityBeta;
        this->creativityCthresh = creativityCthresh;
        this->nothingStateProbThresh = nothingStateProbThresh;
        this->creativityMultiplier = creativityMultiplier;

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

        vector<int> distributionWeights; distributionWeights.push_back(100 - simulationFailingProbability); distributionWeights.push_back(simulationFailingProbability);
        simSuccDist = KUKADU_DISCRETE_DISTRIBUTION<int>(distributionWeights.begin(), distributionWeights.end());

        envReward = KUKADU_SHARED_PTR<EnvironmentReward>(new EnvironmentReward(generator, stdEnvironmentReward));

    }

    ComplexController::~ComplexController() {
        if(rewardHistoryStream && storeReward)
            rewardHistoryStream->close();
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

        for(auto env : environmentModels)
            env.second->updatePsFile();

        projSim->updatePsFile();

    }

    void ComplexController::initialize() {

        nothingStateClips.clear();

        // create / load sensing database
        createSensingDatabase();

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

                    auto pc = KUKADU_SHARED_PTR<PerceptClip>(new PerceptClip(atoi(tok.next().c_str()), label, generator, idVec, immunity));
                    return pc;

                } else if(level == Clip::CLIP_H_LEVEL_FINAL) {

                    auto ac = KUKADU_SHARED_PTR<ActionClip>(new ControllerActionClip(atoi(tok.next().c_str()), (this->availablePreparatoryControllers)[label], generator));
                    return ac;

                } else {

                    if(level == 1)
                        return KUKADU_SHARED_PTR<Clip>(new IntermediateEventClip((this->availableSensingControllers)[label],
                                                                                        level, generator, idVec, immunity));

                    return KUKADU_SHARED_PTR<Clip>(new Clip(level, generator, idVec, immunity));

                }

            };

            projSim = KUKADU_SHARED_PTR<ProjectiveSimulator>(new ProjectiveSimulator(shared_from_this(), generator, psPath, loadLambda));
            // ugly syntax - i have to kill these shared pointers some day
            prepActions = (*((*(projSim->getClipLayers()->end() - 1))->begin()))->getSubClips();
            prepActionsCasted = projSim->getActionClips();
            root = *(projSim->getPerceptClips()->begin());

        } else {

            int currentId = 0;
            KUKADU_SHARED_PTR<vector<int> > clipDimVal = KUKADU_SHARED_PTR<vector<int> >(new vector<int>());
            clipDimVal->push_back(currentId);
            root = KUKADU_SHARED_PTR<PerceptClip>(new PerceptClip(0, "root", generator, clipDimVal, INT_MAX));

            vector<double> prepWeights;
            prepActions = KUKADU_SHARED_PTR<vector<KUKADU_SHARED_PTR<Clip> > >(new vector<KUKADU_SHARED_PTR<Clip> >());
            prepActionsCasted = KUKADU_SHARED_PTR<vector<KUKADU_SHARED_PTR<ActionClip> > >(new vector<KUKADU_SHARED_PTR<ActionClip> >());
            for(int i = 0; i < preparationControllers.size(); ++i) {

                auto prepActionClip = KUKADU_SHARED_PTR<ActionClip>(new ControllerActionClip(i, preparationControllers.at(i), generator));
                prepActions->push_back(prepActionClip);
                prepActionsCasted->push_back(prepActionClip);
                prepWeights.push_back(stdPrepWeight);

            }
            ++currentId;

            for(int i = 0; i < sensingControllers.size(); ++i) {

                KUKADU_SHARED_PTR<SensingController> sensCont = sensingControllers.at(i);

                clipDimVal = KUKADU_SHARED_PTR<vector<int> >(new vector<int>());
                clipDimVal->push_back(currentId);

                KUKADU_SHARED_PTR<Clip> nextSensClip = KUKADU_SHARED_PTR<Clip>(new IntermediateEventClip(sensCont, 1, generator, clipDimVal, INT_MAX));
                ++currentId;
                for(int j = 0; j < sensCont->getSensingCatCount(); ++j, ++currentId) {

                    clipDimVal = KUKADU_SHARED_PTR<vector<int> >(new vector<int>());
                    clipDimVal->push_back(currentId);
                    KUKADU_SHARED_PTR<Clip> nextSubSensClip = KUKADU_SHARED_PTR<Clip>(new Clip(2, generator, clipDimVal, INT_MAX));
                    // create copy of prepratory actions
                    auto prepActionsCopy = make_shared<vector<KUKADU_SHARED_PTR<Clip> > >(prepActions->begin(), prepActions->end());
                    nextSubSensClip->setChildren(prepActionsCopy, prepWeights);
                    nextSensClip->addSubClip(nextSubSensClip, stdPrepWeight);

                }

                double nextWeight = std::exp(senseStretch * max(0.0, sensingWeights.at(i) - 1.0 / sensCont->getSensingCatCount()));
                sensCont->setSimulationClassificationPrecision(min(sensingWeights.at(i) * 100.0, 100.0));
                if(!isShutUp)
                    cout << "(ComplexController) relative weight of sensing action \"" << sensCont->getCaption() << "\" is " << nextWeight << endl;
                root->addSubClip(nextSensClip, nextWeight);

            }

            KUKADU_SHARED_PTR<vector<KUKADU_SHARED_PTR<PerceptClip> > > rootVec = KUKADU_SHARED_PTR<vector<KUKADU_SHARED_PTR<PerceptClip> > >(new vector<KUKADU_SHARED_PTR<PerceptClip> >());
            rootVec->push_back(root);

            // skill is used the first time; do initialization
            projSim = KUKADU_SHARED_PTR<ProjectiveSimulator>(new ProjectiveSimulator(shared_from_this(), generator, rootVec, gamma, ProjectiveSimulator::PS_USE_ORIGINAL, false));

        }

        environmentModels.clear();

        if(fileExists(envModelPath)) {

            // load environment model
            for(auto sens : sensingControllers) {
                auto envModel = KUKADU_SHARED_PTR<ProjectiveSimulator>(new ProjectiveSimulator(envReward, generator, envModelPath + sens->getCaption()));
                environmentModels.insert(std::pair<std::string, KUKADU_SHARED_PTR<kukadu::ProjectiveSimulator> >(sens->getCaption(), envModel));
            }

        } else {

            // create environment model
            createDirectory(envModelPath);

            // load environment model
            for(auto sens : sensingControllers) {
                auto envModel = createEnvironmentModelForSensingAction(sens, projSim);
                environmentModels.insert(std::pair<std::string, KUKADU_SHARED_PTR<kukadu::ProjectiveSimulator> >(sens->getCaption(), envModel));
                envModel->storePS(envModelPath + sens->getCaption());
            }

        }

        projSim->setBoredom(boredom, 2);

        if(storeReward) {
            rewardHistoryStream = KUKADU_SHARED_PTR<std::ofstream>(new std::ofstream());
            int overWrite = 0;
            if(fileExists(historyPath)) {
                cout << "(ComplexController) should reward history file be overwritten? (0 = no / 1 = yes)" << endl;
                cin >> overWrite;
                if(overWrite != 1) {
                    string err = "(ComplexController) reward file already exists. you chose not to overwrite. stopping";
                    cerr << err << endl;
                    throw err;
                }
            }
            rewardHistoryStream->open(rewardHistoryPath.c_str(), ios::trunc);
        }

        stateClips = projSim->getClipsOnLayer(2);
        for(auto sensingClip : *projSim->getPerceptClips()->at(0)->getSubClips()) {
            stateClipsBySensing.push_back(*sensingClip->getSubClips());
            stateClipsPerSensingAction[KUKADU_DYNAMIC_POINTER_CAST<IntermediateEventClip>(sensingClip)->getSensingController()->getCaption()] = *sensingClip->getSubClips();
        }

    }

    std::vector<KUKADU_SHARED_PTR<Clip> > ComplexController::getStateClipsForSensingId(KUKADU_SHARED_PTR<SensingController> sensingId) {
        return stateClipsPerSensingAction[sensingId->getCaption()];
    }

    std::vector<KUKADU_SHARED_PTR<Clip> > ComplexController::getAllStateClips() {
        return stateClips;
    }

    std::map<std::string, std::tuple<double, double, std::vector<double> > > ComplexController::computeEntropyMeanAndVariance(std::vector<KUKADU_SHARED_PTR<SensingController> > sensingIds) {

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

        return retMap;

    }

    std::string ComplexController::getClassLabel(KUKADU_SHARED_PTR<Clip> sensingClip, KUKADU_SHARED_PTR<Clip> stateClip) {
        return stateClip->toString();
    }

    void ComplexController::setBoredom(double boredom) {
        projSim->setBoredom(boredom, 2);
    }

    double ComplexController::getPunishReward() {
        return punishReward;
    }

    double ComplexController::getStdReward() {
        return stdReward;
    }

    void ComplexController::setSimulationModeInChain(bool simulationMode) {

        for(auto sensCont : sensingControllers)
            sensCont->setSimulationMode(simulationMode);

        for(auto prepCont : preparationControllers)
            prepCont->setSimulationMode(simulationMode);

    }

    bool ComplexController::isTrained() {
        if(consecutiveBoredomCount > 20)
            return true;
        return false;
    }

    KUKADU_SHARED_PTR<kukadu::ProjectiveSimulator> ComplexController::createEnvironmentModelForSensingAction(KUKADU_SHARED_PTR<kukadu::SensingController> sensingAction,
                                                                                          KUKADU_SHARED_PTR<ProjectiveSimulator> projSim) {

        KUKADU_SHARED_PTR<IntermediateEventClip> sensingClip = nullptr;
        auto sensingLayer = projSim->getClipLayers()->at(1);
        for(auto sensingAct : *sensingLayer) {
            if(KUKADU_DYNAMIC_POINTER_CAST<IntermediateEventClip>(sensingAct)->toString() == sensingAction->getCaption()) {
                sensingClip = KUKADU_DYNAMIC_POINTER_CAST<IntermediateEventClip>(sensingAct);
                break;
            }
        }

        if(!sensingClip)
            throw KukaduException("(createEnvironmentModel) sensing action not available");

        auto prepClips = sensingClip->getSubClipByIdx(0)->getSubClips();

        int sensingCatCount = sensingClip->getSubClipCount();
        int prepActionsCount = prepClips->size();

        auto stateClips = sensingClip->getSubClips();

        auto environmentPercepts = KUKADU_SHARED_PTR<vector<KUKADU_SHARED_PTR<PerceptClip> > >(new vector<KUKADU_SHARED_PTR<PerceptClip> >());
        auto resultingStatePercepts = KUKADU_SHARED_PTR<vector<KUKADU_SHARED_PTR<Clip> > >(new vector<KUKADU_SHARED_PTR<Clip> >());

        auto idVec = KUKADU_SHARED_PTR< vector<int> >(new vector<int>{0, 0});
        for(auto stateClip : *stateClips) {
            auto stateId = stateClip->getClipDimensions()->at(0);
            stringstream s;
            s << "E" << stateId;
            // have to make it -1 because action clip says internally --> -stateId - 1 (i can't remember the reason anymore)
            resultingStatePercepts->push_back(KUKADU_SHARED_PTR<ActionClip>(new ActionClip(stateId - 1, idVec->size(), s.str(), generator)));
        }

        for(int stateIdx = 0, overallId = sensingCatCount; stateIdx < sensingCatCount; ++stateIdx) {

            int stateId = stateClips->at(stateIdx)->getClipDimensions()->at(0);
            idVec->at(0) = stateId;

            for(int actId = 0; actId < prepActionsCount; ++actId, ++overallId) {

                // ignore the "nothing" controller -> by definition it does nothing and
                // can't be used to change the environment state
                if(prepClips->at(actId)->toString() != nothingController->getCaption()) {

                    idVec->at(1) = prepClips->at(actId)->getClipDimensions()->at(0);

                    stringstream s;
                    s << "(E" << stateId << ",P" << idVec->at(1) << ")";
                    auto vecCopy = KUKADU_SHARED_PTR<vector<int> >(new vector<int>(idVec->begin(), idVec->end()));
                    auto newPercept = KUKADU_SHARED_PTR<PerceptClip>(new PerceptClip(overallId, s.str(), generator, vecCopy, INT_MAX));
                    newPercept->setChildren(resultingStatePercepts);
                    environmentPercepts->push_back(newPercept);

                }

            }

        }

        auto retProjSim = KUKADU_SHARED_PTR<ProjectiveSimulator>(new ProjectiveSimulator(envReward, generator, environmentPercepts, 0.0, ProjectiveSimulator::PS_USE_ORIGINAL, false));
        return retProjSim;

    }

    KUKADU_SHARED_PTR<ProjectiveSimulator> ComplexController::getProjectiveSimulator() {
        return projSim;
    }

    void ComplexController::load(std::string path, std::map<std::string, KUKADU_SHARED_PTR<kukadu::SensingController> > availableSensingControllers, std::map<std::string, KUKADU_SHARED_PTR<kukadu::Controller> > availablePreparatoryControllers) {

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
                else if(controllerMode == 2 )
                    preparationControllers.push_back(availablePreparatoryControllers[line]);
                else if(controllerMode == 3) {

                    // construct composed controllers here
                    KukaduTokenizer tok(line);
                    auto splits = tok.split();
                    vector<KUKADU_SHARED_PTR<kukadu::Controller> > controllerParts;
                    for(auto cont : splits)
                        controllerParts.push_back(availablePreparatoryControllers[cont]);
                    auto conc = make_shared<ConcatController>(controllerParts);
                    preparationControllers.push_back(conc);

                } else if(controllerMode == 10) {
                    break;
                }
            }

        }

        storePath = path;
        this->availableSensingControllers = availableSensingControllers;
        this->availablePreparatoryControllers = availablePreparatoryControllers;

        initialize();

    }

    void ComplexController::store() {
        store(storePath);
    }

    void ComplexController::store(std::string destination) {

        preparePathString(destination);
        string psDestination = destination + "ps";
        string compositionDestination = destination + "composition";
        projSim->storePS(psDestination);
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

    void ComplexController::storeNextIteration() {
        stringstream s;
        s << psPath << currentIterationNum;
        projSim->storePS(s.str());
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

        // simulates non-perfect complex controller
        int failed = simSuccDist(*generator);
        if(failed)
            return getPunishReward();

        double simReward = getSimulatedReward(sensingClip, stateClip, actionClip);

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

            if(executeIt)
                executeComplexAction();

            cout << "did the complex action succeed? (0 = no / 1 = yes)" << endl;
            cin >> worked;

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

        return retReward;

    }

    KUKADU_SHARED_PTR<ControllerResult> ComplexController::performAction() {
        return performAction(false);
    }

    KUKADU_SHARED_PTR<ControllerResult> ComplexController::performAction(bool cleanup, bool generateNewGroundTruth) {

        this->cleanup = cleanup;
        KUKADU_SHARED_PTR<ControllerResult> ret = nullptr;

        // if simulation - set observed state as ground truth for each sensing action (it is not yet know, which sensing action will be selected)
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

            // block for creativity mode
            if(useCreativity) {

                // only add new prep skill if current state is not already a "nothing" skill
                if(!nothingStateClips[sensingClip->toString()][stateClip->toString()]) {

                    // compute all paths with maximum length of 4 and minimal confidence 0.4
                    auto possiblePaths = computeEnvironmentPaths(sensingClip, stateClip, 4, 0.4);
                    // find out if there is a transition to a "nothing" state
                    // sort them according to confidence
                    std::sort(possiblePaths.begin(), possiblePaths.end(), [] (std::tuple<double, KUKADU_SHARED_PTR<Clip>, std::vector<KUKADU_SHARED_PTR<Clip> >, int> p1, std::tuple<double, KUKADU_SHARED_PTR<Clip>, std::vector<KUKADU_SHARED_PTR<Clip> >, int> p2) {
                                  return std::get<0>(p1) > std::get<0>(p2);
                              });
                    for(auto&  path : possiblePaths) {
                        bool pathCanBeChosen = false;
                        auto& resultingState = std::get<1>(path);
                        auto& clipPath = std::get<2>(path);
                        // path has to contain more than 1 preparatory action (path here is state -> prep action -> state -> ... -> final state). therefore length
                        // must be at least 4
                        if(clipPath.size() >= 4) {
                            auto& senseNothingStateClips = nothingStateClips[sensingClip->toString()];
                            for(auto& nothingStateClip : senseNothingStateClips) {
                                // found a proper transition - it may be added as a new preparatory controller
                                if(resultingState == nothingStateClip.second && resultingState != stateClip && !hasDuplicateStatesInPath(clipPath)) {// && !projSim->findClipInLevelByIdVec()) {
                                    pathCanBeChosen = true;
                                    break;
                                }
                            }

                            if(pathCanBeChosen) {

                                // compute probability for adding a new clip from clip composition (will be referred as creativity in the paper)
                                double pathConfidence = std::get<0>(path);
                                double creativityProb = sigmoid(creativityGamma * pathConfidence + creativityDelta);
                                KUKADU_DISCRETE_DISTRIBUTION<int> creativityDist({1.0 - creativityProb, creativityProb});
                                int beCreative = creativityDist(*generator);

                                if(beCreative) {

                                    // extract controller from clip path
                                    int pathSize = clipPath.size();
                                    vector<KUKADU_SHARED_PTR<Controller> > controllerPath;
                                    for(int i = 1; i < pathSize; i += 2) {
                                        auto currentActionClip = KUKADU_DYNAMIC_POINTER_CAST<ControllerActionClip>(clipPath.at(i));
                                        controllerPath.push_back(currentActionClip->getActionController());
                                    }
                                    auto newConcatController = make_shared<ConcatController>(controllerPath);
                                    const auto& controllerLabel = newConcatController->getCaption();

                                    auto creativeActionClipInEcm = projSim->findClipInLevelByLabel(controllerLabel, Clip::CLIP_H_LEVEL_FINAL);
                                    // if clip is already in ecm, (for now) leave it as it is
                                    if(creativeActionClipInEcm) {

                                    }
                                    // else insert it to the skill ecm and the environment ecm and connect it properly
                                    else {

                                        // first create action clip
                                        auto newConcatClip = make_shared<ControllerActionClip>(projSim->generateNewActionId(), newConcatController, generator);

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

            if(!getSimulationMode()) {
                auto sensedLabel = getClassLabel(sensingClip, stateClip);
                cout << "(ComplexController::performAction) selected sensing action \"" << *sensingClip << "\" resulted in predicted class " << sensedLabel << " and preparation action \"" << *actionClip << "\"" << endl;
            }

            KUKADU_DYNAMIC_POINTER_CAST<ControllerActionClip>(actionClip)->performAction();

            auto sensedState = stateClip;

            // no environment model measuring in case the nothing controller was used
            // by definition it does not change the state
            if(actionClip->toString() != nothingController->getCaption()) {

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
                    cout << "(ComplexController::performAction) classifier result is category " << sensedLabel << endl;
                }

                vector<int> stateVector{stateId, actionId};
                auto currentEnvModel = environmentModels[sensingClip->toString()];
                auto environmentClip = currentEnvModel->retrieveClipsOnLayer(stateVector, 0).at(0);

                if(!isShutUp)
                    cout << "(" << stateId << ", " << actionId << ") - " << *environmentClip << " --> " << "E" << resultingStateId << " (idx: " << resultingStateChildIdx << ")" << endl;

                auto resultingEnvironmentClip = currentEnvModel->retrieveClipsOnLayer({-resultingStateId, -resultingStateId}, 1).at(0);

                vector<KUKADU_SHARED_PTR<Clip> > envClipPath{environmentClip, resultingEnvironmentClip};
                currentEnvModel->setNextPredefinedPath(envClipPath);
                currentEnvModel->performRandomWalk();
                currentEnvModel->performRewarding();

            }

            // after doing everything --> perform the complex action and reward it accordingly
            auto rewRet = projSim->performRewarding();
            reward = get<1>(rewRet);

            // block for determining the "nothing" states is only required if creativity is switched on
            if(useCreativity) {

                // pair contains maximal hop probability of clip and the corresponding clip
                auto maxProbClipPair = sensedState->getMaxProbability();
                double maxProb = get<0>(maxProbClipPair);
                int maxWeight = get<1>(maxProbClipPair);
                auto maxPrepClip = get<2>(maxProbClipPair);

                // if probability of success after using "nothing" action is high enough, the state clip might be considered in future reasoning
                // e.g. for guided clip creation
                if(maxWeight > stdPrepWeight) {
                    if(maxProb > nothingStateProbThresh && KUKADU_DYNAMIC_POINTER_CAST<ControllerActionClip>(maxPrepClip)->toString() == nothingController->getCaption() && !nothingStateClips[sensingClip->toString()][sensedState->toString()]) {
                        nothingStateClips[sensingClip->toString()][sensedState->toString()] = sensedState;

                    // if there is a state where the strongest action is "nothing" but the probability is below 0.8, remove it (even if it is not in there yet - checking
                    // this would just make it slower)
                    } else if(maxProb <= nothingStateProbThresh && KUKADU_DYNAMIC_POINTER_CAST<ControllerActionClip>(maxPrepClip)->toString() == "nothing")
                        nothingStateClips[sensingClip->toString()][sensedState->toString()] = nullptr;
                }

            }

        } else {

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
                        KUKADU_DYNAMIC_POINTER_CAST<ControllerActionClip>(cl)->performAction();
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
            ret = this->performAction(cleanup, false);
            KUKADU_DYNAMIC_POINTER_CAST<HapticControllerResult>(ret)->setWasBored(true);

            // switching on boredom again
            this->setBoredom(true);

        }

        auto selectedPathPointer = KUKADU_SHARED_PTR<std::tuple<double, KUKADU_SHARED_PTR<kukadu::Clip>, std::vector<KUKADU_SHARED_PTR<kukadu::Clip> > > >(new std::tuple<double, KUKADU_SHARED_PTR<kukadu::Clip>, std::vector<KUKADU_SHARED_PTR<kukadu::Clip> > >(get<0>(selectedPath), get<1>(selectedPath), get<2>(selectedPath)));

        // this behaviour could be improved --> TODO
        if(!ret)
            ret = make_shared<HapticControllerResult>(vec(), vector<vec>(), (reward > 0.0) ? true : false, false, *(projSim->getIntermediateHopIdx()), selectedPathPointer);

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

    void ComplexController::printPaths(std::vector<std::tuple<double, KUKADU_SHARED_PTR<Clip>, std::vector<KUKADU_SHARED_PTR<Clip> > > >& paths) {
        for(auto path : paths) {
            cout << get<0>(path) << "; " << *get<1>(path) << ": ";
            for(auto cl : get<2>(path))
                cout << *cl << " - ";
            cout << endl;
        }
    }

    void ComplexController::computeTotalPathCost(KUKADU_SHARED_PTR<IntermediateEventClip> sensingClip,
                                                 std::vector<std::tuple<double, KUKADU_SHARED_PTR<Clip>,
                                                 std::vector<KUKADU_SHARED_PTR<Clip> >, int> >& paths) {

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

    }

    std::vector<std::tuple<double, KUKADU_SHARED_PTR<Clip>, std::vector<KUKADU_SHARED_PTR<Clip> >, int> > ComplexController::computeEnvironmentPaths(
            KUKADU_SHARED_PTR<Clip> sensingClip, KUKADU_SHARED_PTR<Clip> stateClip, int maxPathLength, double confidenceCut) {

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

        return allPaths;

    }

    std::tuple<double, int, int> ComplexController::computeEnvironmentTransitionConfidence(KUKADU_SHARED_PTR<Clip> stateClip) {

        auto likeliest = stateClip->getLikeliestChildWithWeight();
        double stateEntropy = stateClip->computeSubEntropy();
        double confidenceProb = 1.0 - stateEntropy / log2(stateClip->getSubClipCount());
        auto likeliestResult = likeliest.second;
        int envId = atoi(likeliestResult->toString().substr(1).c_str());
        return std::make_tuple(confidenceProb, envId, likeliest.first);

    }

    bool ComplexController::setUseCreativity(bool useCreativity) {
        this->useCreativity = useCreativity;
    }

    void ComplexController::setTrainingMode(bool doTraining) {
        projSim->setTrainingMode(doTraining);
    }

    void ComplexController::createSensingDatabase() {

        createSensingDatabase(sensingControllers);

    }

    void ComplexController::createSensingDatabase(std::vector<KUKADU_SHARED_PTR<SensingController> > sensingControllers) {

        sensingWeights.clear();
        for(int i = 0; i < sensingControllers.size(); ++i) {
            double validationScore = sensingControllers.at(i)->createDataBase();
            sensingWeights.push_back(validationScore);
        }

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

}
