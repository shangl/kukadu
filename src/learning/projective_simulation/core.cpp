#include <fstream>
#include <sstream>
#include <iostream>
#include <kukadu/utils/utils.hpp>
#include <kukadu/utils/kukadutokenizer.hpp>
#include <kukadu/storage/moduleusagesingleton.hpp>
#include <kukadu/learning/projective_simulation/core.hpp>

using namespace std;

namespace kukadu {

    Reward::Reward(KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, bool collectPrevRewards) {
        this->generator = generator;
        this->collectPrevRewards = collectPrevRewards;
    }

    double Reward::computeReward(KUKADU_SHARED_PTR<PerceptClip> providedPercept, KUKADU_SHARED_PTR<ActionClip> takenAction) {
        double retrievedReward = computeRewardInternal(providedPercept, takenAction);
        if(collectPrevRewards)
            previousRewards.push_back(retrievedReward);
        return retrievedReward;
    }

    std::vector<double> Reward::getPreviousRewards() {
        return previousRewards;
    }

    ManualReward::ManualReward(KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, int numberOfActions, int numberOfPercepts, bool collectPrevRewards, double stdReward) : Reward(generator, collectPrevRewards) {

        this->numberOfActions = numberOfActions;
        this->numberOfPercepts = numberOfPercepts;
        this->nextPerceptId = 0;
        this->stdReward = stdReward;

        perceptClips = KUKADU_SHARED_PTR<vector<KUKADU_SHARED_PTR<PerceptClip> > >(new vector<KUKADU_SHARED_PTR<PerceptClip> >());
        for(int i = 0; i < numberOfPercepts; ++i) {
            stringstream s;
            s << i;
            KUKADU_SHARED_PTR<std::vector<int> > clipDimensionValues = KUKADU_SHARED_PTR<std::vector<int> >(new vector<int>());
            clipDimensionValues->push_back(i);
            perceptClips->push_back(KUKADU_SHARED_PTR<PerceptClip>(new PerceptClip(i, s.str(), generator, clipDimensionValues, Clip::PS_DEFAULT_IMMUNITY)));
        }

        actionClips = KUKADU_SHARED_PTR<vector<KUKADU_SHARED_PTR<ActionClip> > >(new vector<KUKADU_SHARED_PTR<ActionClip> >());
        for(int i = 0; i < numberOfActions; ++i) {
            stringstream s;
            s << i;
            KUKADU_SHARED_PTR<std::vector<int> > clipDimensionValues = KUKADU_SHARED_PTR<std::vector<int> >(new vector<int>());
            clipDimensionValues->push_back(i);
            actionClips->push_back(KUKADU_SHARED_PTR<ActionClip>(new ActionClip(i, 1, s.str(), generator)));
        }

    }

    ManualReward::~ManualReward() {

    }

    void ManualReward::setNextPerceptId(int nextId) {
        this->nextPerceptId = nextId;
    }

    int ManualReward::getDimensionality() {

        return 1;

    }

    KUKADU_SHARED_PTR<PerceptClip> ManualReward::generateNextPerceptClip(int immunity) {

        return perceptClips->at(nextPerceptId);

    }

    double ManualReward::computeRewardInternal(KUKADU_SHARED_PTR<PerceptClip> providedPercept, KUKADU_SHARED_PTR<ActionClip> takenAction) {

        KUKADU_MODULE_START_USAGE();

        int worked = 0;
        double retReward = 0.0;
        cout << "selected percept " << *providedPercept << " resulted in action " << *takenAction << endl;
        cout << "was it the correction action (0 = no / 1 = yes)" << endl;
        cin >> worked;

        if(worked) {
            cout << "preparation action worked; rewarded with " << stdReward << endl;
            retReward = stdReward;
        } else {
            cout << "preparation action didn't work; no reward given" << endl;
            retReward = 0.0;
        }

        KUKADU_MODULE_END_USAGE();

        return retReward;

    }

    KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<ActionClip> > > ManualReward::generateActionClips() {
        return actionClips;
    }

    KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<PerceptClip> > > ManualReward::generatePerceptClips() {
        return perceptClips;
    }

    PSEvaluator::PSEvaluator() {
    }

    std::pair<std::vector<int>, std::vector<double> > PSEvaluator::evaluateStatistics(std::vector<string> inputFiles, std::vector<int> inputPos) {

        KUKADU_MODULE_START_USAGE();

        vector<double> retSuccess;
        vector<double> walkSuccess;
        int usedFilesCount = 0;
        int lowestMaxWalkIdx = INT32_MAX;
        char memblock[PSEVAL_BUFFER_SIZE + 1];

        for(int streamNum = 0; streamNum < inputFiles.size(); ++streamNum) {
            string lastLine = "";
            string dataLine = "";
            string currentStreamFile = inputFiles.at(streamNum);
            ifstream currentStream;
            currentStream.open(currentStreamFile.c_str());
            currentStream.seekg(inputPos.at(streamNum));
            memblock[0] = '\0';
            size_t endPos = 0;

            while(currentStream && (endPos = (lastLine + string(memblock)).find("number of cats:")) == string::npos) {

                lastLine = string(memblock);
                dataLine += lastLine;

                currentStream.read(memblock, PSEVAL_BUFFER_SIZE);
                memblock[PSEVAL_BUFFER_SIZE] = '\0';

            }

            dataLine += string(memblock).substr(0, endPos);
            currentStream.seekg((endPos - lastLine.size()) - PSEVAL_BUFFER_SIZE, currentStream.cur);

            int walkIdx = 0;
            for(int currentSignIdx = 0; currentSignIdx < dataLine.size(); currentSignIdx += 2) {

                int currentSign = dataLine[currentSignIdx];
                for(int i = 0; i < 8; ++i) {

                    int currentWalk = ((currentSign & 128) == 128) ? 1 : 0;
                    currentSign = currentSign << 1;

                    if(streamNum == 0)
                        walkSuccess.push_back(currentWalk);
                    else if(walkIdx < walkSuccess.size())
                        walkSuccess.at(walkIdx) += currentWalk;

                    ++walkIdx;

                }

            }

            if(walkIdx > 0)
                ++usedFilesCount;
            else
                walkIdx = INT32_MAX;

            if(walkIdx < lowestMaxWalkIdx)
                lowestMaxWalkIdx = walkIdx;

            inputPos.at(streamNum) = currentStream.tellg();
            currentStream.close();

        }

        if(lowestMaxWalkIdx < INT32_MAX)
            for(int i = 0; i < lowestMaxWalkIdx; ++i) {
                retSuccess.push_back(walkSuccess.at(i) / usedFilesCount);
            }

        auto retVal = std::pair<std::vector<int>, std::vector<double> >(inputPos, retSuccess);

        KUKADU_MODULE_END_USAGE();

        return retVal;

    }

    vector<double> PSEvaluator::evaluateStatistics(std::vector<KUKADU_SHARED_PTR<std::ifstream> >& inputStreams) {

        KUKADU_MODULE_START_USAGE();

        vector<double> retSuccess;
        vector<double> walkSuccess;
        int usedFilesCount = 0;
        int lowestMaxWalkIdx = INT32_MAX;
        char memblock[PSEVAL_BUFFER_SIZE + 1];

        for(int streamNum = 0; streamNum < inputStreams.size(); ++streamNum) {
            string lastLine = "";
            string dataLine = "";
            KUKADU_SHARED_PTR<ifstream> currentStream = inputStreams.at(streamNum);
            memblock[0] = '\0';
            size_t endPos = 0;

            while(*currentStream && (endPos = (lastLine + string(memblock)).find("number of cats:")) == string::npos) {
                lastLine = string(memblock);
                dataLine += lastLine;
                currentStream->read(memblock, PSEVAL_BUFFER_SIZE);
                memblock[PSEVAL_BUFFER_SIZE] = '\0';
            }

            dataLine += string(memblock).substr(0, endPos);
            currentStream->seekg((endPos - lastLine.size()) - PSEVAL_BUFFER_SIZE, currentStream->cur);

            int walkIdx = 0;
            for(int currentSignIdx = 0; currentSignIdx < dataLine.size(); currentSignIdx += 2) {

                int currentSign = dataLine[currentSignIdx];
                for(int i = 0; i < 8; ++i) {

                    int currentWalk = ((currentSign & 128) == 128) ? 1 : 0;
                    currentSign = currentSign << 1;

                    if(streamNum == 0)
                        walkSuccess.push_back(currentWalk);
                    else if(walkIdx < walkSuccess.size()) {
                        walkSuccess.at(walkIdx) += currentWalk;
                    }

                    ++walkIdx;

                }

            }

            if(walkIdx > 0)
                ++usedFilesCount;
            else
                walkIdx = INT32_MAX;

            if(walkIdx < lowestMaxWalkIdx)
                lowestMaxWalkIdx = walkIdx;

        }

        for(int i = 0; i < lowestMaxWalkIdx; ++i) {
            retSuccess.push_back(walkSuccess.at(i) / usedFilesCount);
        }

        KUKADU_MODULE_END_USAGE();

        return retSuccess;

    }

    void PSEvaluator::produceStatistics(KUKADU_SHARED_PTR<ProjectiveSimulator> ps, KUKADU_SHARED_PTR<Reward> reward, int numberOfWalks, int clipImmunity, int rewardValue, std::ostream& outStream) {

        KUKADU_MODULE_START_USAGE();

        char currentOutput = 0;
        int fieldsInCurrentOutput = 0;
        for(int j = 0; j < numberOfWalks; ++j, ++fieldsInCurrentOutput) {

            KUKADU_SHARED_PTR<PerceptClip> nextClip = reward->generateNextPerceptClip(clipImmunity);
            ps->generalize(nextClip);
            ps->performRandomWalk();

            auto rewRes = ps->performRewarding();
            auto reward = std::get<1>(rewRes);
            auto bored = std::get<0>(rewRes);
            int lastResult = reward / rewardValue;
            if(!bored) {
                if(fieldsInCurrentOutput < 8) {
                    currentOutput = currentOutput << 1;
                } else {
                    outStream << currentOutput << ",";
                    fieldsInCurrentOutput = 0;
                    currentOutput = 0;
                }
                currentOutput = currentOutput | lastResult;
            } else {
                --fieldsInCurrentOutput;
            }

        }

        KUKADU_MODULE_END_USAGE();

    }

    void ProjectiveSimulator::loadPsConstructor(KUKADU_SHARED_PTR<Reward> reward, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, std::string file,
                       std::function<KUKADU_SHARED_PTR<Clip> (const std::string&, const int&, const int&, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator) > createClipFunc) {

        KUKADU_MODULE_START_USAGE();

        maxActionId = 0;
        maxPerceptId = 0;
        lastBoredomResult = false;
        walkedFurtherSinceLastBoredom = true;
        lastRunWasBored = false;

        this->psFile = file;
        this->loadedFromFile = true;
        this->reward = reward;
        this->generator = generator;

        this->lastActionClip;
        this->lastPerceptClip;

        actionClips = KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<ActionClip> > >(new std::vector<KUKADU_SHARED_PTR<ActionClip> >());
        perceptClips = KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<PerceptClip> > >(new std::vector<KUKADU_SHARED_PTR<PerceptClip> >());
        clipLayers = KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<std::set<KUKADU_SHARED_PTR<Clip>, clip_compare> > > >(new std::vector<KUKADU_SHARED_PTR<std::set<KUKADU_SHARED_PTR<Clip>, clip_compare> > >());

        intermediateHops = KUKADU_SHARED_PTR<vector<int> >(new vector<int>());

        string line = "";
        ifstream inputFile;
        inputFile.open(file.c_str());

        // check version
        getline(inputFile, line);
        if(!line.compare("V1.0")) {

            // ignore "general properties" line
            getline(inputFile, line);

            // operation mode
            getline(inputFile, line);
            KukaduTokenizer tok(line, "=");
            tok.next(); operationMode = atoi(tok.next().c_str());

            // use ranking?
            getline(inputFile, line);
            tok = KukaduTokenizer(line, "=");
            tok.next(); useRanking = (atoi(tok.next().c_str()))?true:false;

            // gamma
            getline(inputFile, line);
            tok = KukaduTokenizer(line, "=");
            tok.next(); gamma = atof(tok.next().c_str());

            // max number of clips (ignored without ranking)
            getline(inputFile, line);
            tok = KukaduTokenizer(line, "=");
            tok.next(); maxNumberOfClips = atoi(tok.next().c_str());

            // immunity threshhold (ignored without ranking)
            getline(inputFile, line);
            tok = KukaduTokenizer(line, "=");
            tok.next(); immunityThresh = atoi(tok.next().c_str());

            // levels
            getline(inputFile, line);
            tok = KukaduTokenizer(line, "=");
            tok.next(); levels = atoi(tok.next().c_str());

            for(int i = 0; i < levels; ++i)
                clipLayers->push_back(KUKADU_SHARED_PTR<set<KUKADU_SHARED_PTR<Clip>, clip_compare> >(new set<KUKADU_SHARED_PTR<Clip>, clip_compare>()));

            getline(inputFile, line);
            getline(inputFile, line);

            // load the clips
            int currentLayer = 0;
            bool isFirstPercept = true;
            int perceptDimensionality = 0;
            while(getline(inputFile, line) && line.compare("")) {

                // check if its a layer line
                tok = KukaduTokenizer(line, "=");
                string nextToken = tok.next();

                if(!nextToken.compare("layer")) {

                    string layerString = tok.next();
                    currentLayer = atoi(layerString.c_str());

                } else {

                    // it is not a layer line (must be a clip line)
                    tok = KukaduTokenizer(line, ";");
                    KUKADU_SHARED_PTR<Clip> nextClip;

                    // first line is the id vector
                    if(currentLayer == 0) {

                        if(nextClip = createClipFunc(line, currentLayer, 0, generator)) {

                            auto pc = KUKADU_DYNAMIC_POINTER_CAST<PerceptClip>(nextClip);
                            maxPerceptId = std::max(maxPerceptId, pc->getPerceptId());

                            if(isFirstPercept) {

                                isFirstPercept = false;
                                perceptDimensionality = pc->getDimensionality();

                            }

                            perceptClips->push_back(pc);

                        } else {
                            cerr << "(ProjectiveSimulator) warning: not all clips were created" << endl;
                            continue;
                        }

                    } else if(currentLayer == Clip::CLIP_H_LEVEL_FINAL) {

                        if(line != "") {

                            if(nextClip = createClipFunc(line, currentLayer, perceptDimensionality, generator)) {

                                auto nextActionClip = KUKADU_DYNAMIC_POINTER_CAST<ActionClip>(nextClip);
                                maxActionId = std::max(maxActionId, nextActionClip->getActionId());
                                actionClips->push_back(nextActionClip);

                            } else {
                                cerr << "(ProjectiveSimulator) warning: not all clips were created" << endl;
                                continue;
                            }
                        } else
                            continue;

                    } else {

                        if(line != "")
                            if(nextClip = createClipFunc(line, currentLayer, perceptDimensionality, generator)) {}
                            else
                                continue;
                        else
                            continue;

                    }

                    int clipLevel = currentLayer;
                    if(clipLevel != Clip::CLIP_H_LEVEL_FINAL)
                        clipLayers->at(clipLevel)->insert(nextClip);
                    else
                        clipLayers->at(clipLayers->size() - 1)->insert(nextClip);

                }

            }

            getline(inputFile, line);

            // connect the clips
            currentLayer = 0;
            KUKADU_SHARED_PTR<Clip> currentParent;
            std::vector<double> newChildrenWeights;
            KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<Clip> > > newChildren = KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<Clip> > >(new std::vector<KUKADU_SHARED_PTR<Clip> >());
            while(getline(inputFile, line)) {

                // check if its a layer line
                tok = KukaduTokenizer(line, "=");
                string nextToken = tok.next();
                if(!nextToken.compare("layer")) {

                    string layerString = tok.next();
                    currentLayer = atoi(layerString.c_str());

                } else {

                    // a new parent will be provided next
                    if(!line.compare("")) {

                        if(currentParent)
                            currentParent->setChildren(newChildren, newChildrenWeights);

                        currentParent.reset();
                        newChildrenWeights.clear();
                        newChildren = make_shared<std::vector<KUKADU_SHARED_PTR<Clip> > >();

                    }
                    // its a new parent clip
                    else if(line.find(':') != string::npos) {

                        currentParent = findClipByIdVec(Clip::getIdVectorFromString(line.substr(0, line.size() - 1)));

                    } else if(line.find(';') != string::npos) {

                        // it must be a new child clip
                        tok = KukaduTokenizer(line, ";");
                        string idVecString = tok.next();
                        double connectionWeight = atof(tok.next().c_str());
                        KUKADU_SHARED_PTR<Clip> currentChild = findClipByIdVec(Clip::getIdVectorFromString(idVecString));

                        if(currentChild) {
                            newChildren->push_back(currentChild);
                            newChildrenWeights.push_back(connectionWeight);
                        }

                    }

                }

            }

        } else
            throw KukaduException("(ProjectiveSimulator) PS file version cannot be handled");

        boredomLevels.clear();
        for(int i = 0; i < clipLayers->size(); ++i)
            boredomLevels.push_back(0.0);

        KUKADU_MODULE_END_USAGE();

    }

    ProjectiveSimulator::ProjectiveSimulator(KUKADU_SHARED_PTR<Reward> reward, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, std::string file,
                    std::function<KUKADU_SHARED_PTR<Clip> (const std::string&, const int&, const int&, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator) > createClipFunc) {

        maxActionId = 0;
        maxPerceptId = 0;
        this->loadedFromFile = true;
        immunityThresh = 0;
        loadPsConstructor(reward, generator, file, createClipFunc);
        lastRunWasBored = false;
        walkedFurtherSinceLastBoredom = true;
        lastBoredomResult = false;

    }

    ProjectiveSimulator::ProjectiveSimulator(KUKADU_SHARED_PTR<Reward> reward, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, std::string file) {

        maxActionId = 0;
        maxPerceptId = 0;
        this->loadedFromFile = true;
        walkedFurtherSinceLastBoredom = true;
        lastBoredomResult = false;
        immunityThresh = 0;

        loadPsConstructor(reward, generator, file, [] (const std::string& line, const int& level, const int& perceptDimensionality, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator) -> KUKADU_SHARED_PTR<Clip> {

            KukaduTokenizer tok(line, ";");
            string idVec = tok.next();
            string label = tok.next();
            int immunity = atoi(tok.next().c_str());
            if(level == 0) {

                auto pc = make_shared<PerceptClip>(atoi(tok.next().c_str()), label, generator, idVec, immunity);
                return pc;

            } else if(level == Clip::CLIP_H_LEVEL_FINAL) {

                auto ac = make_shared<ActionClip>(atoi(tok.next().c_str()), perceptDimensionality, label, generator);
                return ac;

            } else {


                return make_shared<Clip>(level, generator, idVec, immunity);

            }

        });
        lastRunWasBored = false;

    }

    std::vector<KUKADU_SHARED_PTR<Clip> > ProjectiveSimulator::getClipsOnLayer(int layerId) {
        return vector<KUKADU_SHARED_PTR<Clip> >(getClipLayers()->at(layerId)->begin(), getClipLayers()->at(layerId)->end());
    }

    void ProjectiveSimulator::updatePsFile() {

        if(loadedFromFile) {
            deleteFile(psFile);
            storePS(psFile);
        } else
            throw KukaduException("(ProjectiveSimulator) PS model was not loaded from file");

    }

    void ProjectiveSimulator::setNextPredefinedPath(std::vector<KUKADU_SHARED_PTR<Clip> > hopPath) {

        predefinedFirstHop = hopPath.at(0);
        for(int i = 0; i < hopPath.size() - 1; ++i)
            hopPath.at(i)->setNextHop(hopPath.at(i + 1));

    }

    int ProjectiveSimulator::getIdVecLevel(KUKADU_SHARED_PTR<std::vector<int> > idVec) {

        int retCount = 0;

        for(int i = 0; i < idVec->size(); ++i) {
            int val = idVec->at(i);
            auto clipHashVal = Clip::CLIP_H_HASH_VAL;
            if(val == clipHashVal)
                retCount++;
        }

        return retCount;

    }

    void ProjectiveSimulator::addActionClip(KUKADU_SHARED_PTR<ActionClip> newAction) {
        actionClips->push_back(newAction);
        clipLayers->at(clipLayers->size() - 1)->insert(newAction);
    }

    void ProjectiveSimulator::addPerceptClip(KUKADU_SHARED_PTR<PerceptClip> newPercept) {
        perceptClips->push_back(newPercept);
        clipLayers->at(0)->insert(newPercept);
        intDist = kukadu_uniform_distribution(0, perceptClips->size() - 1);
    }

    KUKADU_SHARED_PTR<Clip> ProjectiveSimulator::findClipInLevelByLabel(std::string label, int level) {

        if(level == Clip::CLIP_H_LEVEL_FINAL)
            level = clipLayers->size() - 1;

        KUKADU_SHARED_PTR<std::set<KUKADU_SHARED_PTR<Clip>, clip_compare> > currentLayer = clipLayers->at(level);

        std::set<KUKADU_SHARED_PTR<Clip>, clip_compare>::iterator it;
        for(auto& clip : *currentLayer) {

            const auto& currentClipLabel = clip->toString();
            if(currentClipLabel == label)
                return clip;

        }

        return nullptr;

    }

    KUKADU_SHARED_PTR<Clip> ProjectiveSimulator::findClipInLevelByIdVec(KUKADU_SHARED_PTR<std::vector<int> > idVec, int level) {

        if(level == Clip::CLIP_H_LEVEL_FINAL)
            level = clipLayers->size() - 1;

        KUKADU_SHARED_PTR<std::set<KUKADU_SHARED_PTR<Clip>, clip_compare> > currentLayer = clipLayers->at(level);

        std::set<KUKADU_SHARED_PTR<Clip>, clip_compare>::iterator it;
        for(it = currentLayer->begin(); it != currentLayer->end(); ++it) {
            KUKADU_SHARED_PTR<Clip> c = *it;

            KUKADU_SHARED_PTR<vector<int> > clipDim = c->getClipDimensions();
            if(Clip::compareIdVecs(clipDim, idVec))
                return c;

        }

        return nullptr;

    }

    KUKADU_SHARED_PTR<Clip> ProjectiveSimulator::findClipByIdVec(KUKADU_SHARED_PTR<std::vector<int> > idVec) {

        int originalMode = PS_USE_ORIGINAL;
        int useGenMode = PS_USE_GEN;

        if(operationMode == originalMode) {

            for(int level = 0; level < clipLayers->size() - 1; ++level) {

                KUKADU_SHARED_PTR<Clip> retVal = findClipInLevelByIdVec(idVec, level);
                if(retVal)
                    return retVal;

            }

        } else if(operationMode == useGenMode) {
            // this works only for generalization where the number of wildcards says something about the level
            int level = getIdVecLevel(idVec);
            return findClipInLevelByIdVec(idVec, level);
        }

        for(int i = 0; i < actionClips->size(); ++i) {
            KUKADU_SHARED_PTR<Clip> ac = actionClips->at(i);
            KUKADU_SHARED_PTR<vector<int> > clipDim = ac->getClipDimensions();
            if(Clip::compareIdVecs(clipDim, idVec)) {
                return ac;
            }
        }

        return nullptr;

    }

    void ProjectiveSimulator::setBoredom(double boredom, int level) {

        KUKADU_MODULE_START_USAGE();

        if(boredom <= 0.0)
            this->boredomLevels.at(level) = 0.0;
        else
            this->boredomLevels.at(level) = boredom;

        KUKADU_MODULE_END_USAGE();

    }

    void ProjectiveSimulator::setTrainingMode(bool doTraining) {
        this->doTraining = doTraining;
    }

    void ProjectiveSimulator::construct(KUKADU_SHARED_PTR<Reward> reward, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, double gamma, int operationMode, bool useRanking) {

        lastBoredomResult = false;
        lastRunWasBored = false;
        this->doTraining = true;

        this->useRanking = useRanking;
        this->operationMode = operationMode;
        this->gamma = gamma;
        intermediateHops = KUKADU_SHARED_PTR<vector<int> >(new vector<int>());

        this->lastActionClip.reset();
        this->lastPerceptClip.reset();

        this->reward = reward;
        this->maxNumberOfClips = PS_MAX_NUMBER_OF_CLIPS;

        this->generator = generator;
        intDist = kukadu_uniform_distribution(0, perceptClips->size() - 1);

        clipLayers = KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<std::set<KUKADU_SHARED_PTR<Clip>, clip_compare> > > >(new std::vector<KUKADU_SHARED_PTR<std::set<KUKADU_SHARED_PTR<Clip>, clip_compare> > >());

        for(int i = 0; i < levels + 1; ++i)
            clipLayers->push_back(KUKADU_SHARED_PTR<set<KUKADU_SHARED_PTR<Clip>, clip_compare> >(new set<KUKADU_SHARED_PTR<Clip>, clip_compare>()));

        clipLayers->at(0)->insert(perceptClips->begin(), perceptClips->end());
        clipLayers->at(clipLayers->size() - 1)->insert(actionClips->begin(), actionClips->end());

        boredomLevels.clear();
        for(int i = 0; i < clipLayers->size(); ++i)
            boredomLevels.push_back(0.0);

        lastGeneralizedPercept.reset();

    }

    ProjectiveSimulator::ProjectiveSimulator(KUKADU_SHARED_PTR<Reward> reward, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, double gamma, int operationMode, bool useRanking) {

        maxActionId = 0;
        maxPerceptId = 0;
        this->loadedFromFile = false;
        immunityThresh = 0;

        int originalMode = PS_USE_ORIGINAL;
        int genMode = PS_USE_GEN;

        this->perceptClips = reward->generatePerceptClips();
        this->actionClips = reward->generateActionClips();

        levels = 0;
        if(operationMode == originalMode)
            levels = 1;
        else if(operationMode == genMode)
            // + 1 for the (#, #, #, ...) layer
            levels = reward->getDimensionality() + 1;

        KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<Clip> > > clipActionClips = make_shared<std::vector<KUKADU_SHARED_PTR<Clip> > >();
        for(int i = 0; i < actionClips->size(); ++i) {
            auto t = actionClips->at(i);
            maxActionId = std::max(maxActionId, t->getActionId());
            clipActionClips->push_back(t);
        }

        for(int i = 0; i < perceptClips->size(); ++i) {
            auto currentClip = perceptClips->at(i);
            maxPerceptId = std::max(maxPerceptId, KUKADU_DYNAMIC_POINTER_CAST<PerceptClip>(currentClip)->getPerceptId());
            currentClip->setChildren(clipActionClips);
        }

        construct(reward, generator, gamma, operationMode, useRanking);

    }

    bool ProjectiveSimulator::compareIdVectors(std::vector<int>& idVec1, std::vector<int>& idVec2) {

        if(idVec1.size() == idVec2.size()) {

            for(int i = 0; i < idVec1.size(); ++i) {
                if(idVec1.at(i) != IGNORE_ID && idVec2.at(i) != IGNORE_ID && idVec1.at(i) != idVec2.at(i))
                    return false;
            }

        } else {
            throw KukaduException("(retrieveClipsOnLayer) id vector dimension is not correct");
        }

        return true;

    }

    std::vector<KUKADU_SHARED_PTR<Clip> > ProjectiveSimulator::retrieveClipsOnLayer(std::vector<int> queryId, int layer) {

        std::vector<KUKADU_SHARED_PTR<Clip> > queriedClips;
        auto requestedLayer = clipLayers->at(layer);
        for(auto clip : *requestedLayer) {

            auto clipDims = clip->getClipDimensions();

            if(compareIdVectors(queryId, *clipDims))
                queriedClips.push_back(clip);

        }
        return queriedClips;

    }

    ProjectiveSimulator::ProjectiveSimulator(KUKADU_SHARED_PTR<Reward> reward, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator,
                        KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<PerceptClip> > > network,
                        double gamma, int operationMode, bool useRanking) {

        maxActionId = 0;
        maxPerceptId = 0;
        this->loadedFromFile = false;
        this->perceptClips = network;
        immunityThresh = 0;

        // set levels and action clips
        // walk down to last clip
        levels = 0;
        KUKADU_SHARED_PTR<Clip> lastClip;
        lastClip.reset();
        KUKADU_SHARED_PTR<Clip> currClip = perceptClips->at(0);
        maxPerceptId = KUKADU_DYNAMIC_POINTER_CAST<PerceptClip>(currClip)->getPerceptId();
        while(currClip->getSubClipCount()) {
            lastClip = currClip;
            currClip = currClip->getSubClipByIdx(0);
            ++levels;
        }

        actionClips = KUKADU_SHARED_PTR<vector<KUKADU_SHARED_PTR<ActionClip> > >(new vector<KUKADU_SHARED_PTR<ActionClip> >());
        for(int i = 0; i < lastClip->getSubClipCount(); ++i) {
            KUKADU_SHARED_PTR<ActionClip> nextActClip = KUKADU_DYNAMIC_POINTER_CAST<ActionClip>(lastClip->getSubClipByIdx(i));
            actionClips->push_back(nextActClip);
            maxActionId = std::max(maxActionId, nextActClip->getActionId());
        }

        construct(reward, generator, gamma, operationMode, useRanking);

        for(int i = 0; i < perceptClips->size(); ++i) {
            KUKADU_SHARED_PTR<PerceptClip> pc = perceptClips->at(i);
            maxPerceptId = std::max(maxPerceptId, pc->getPerceptId());
            fillClipLayersFromNetwork(pc);
        }

        boredomLevels.clear();
        for(int i = 0; i < clipLayers->size(); ++i)
            boredomLevels.push_back(0.0);

    }

    void ProjectiveSimulator::fillClipLayersFromNetwork(KUKADU_SHARED_PTR<Clip> cl) {

        int level = cl->getLevel();
        if(level != Clip::CLIP_H_LEVEL_FINAL) {
            clipLayers->at(level)->insert(cl);
            for(int i = 0; i < cl->getSubClipCount(); ++i)
                fillClipLayersFromNetwork(cl->getSubClipByIdx(i));

        } else {
            clipLayers->at(clipLayers->size() - 1)->insert(cl);
        }

        boredomLevels.clear();
        for(int i = 0; i < clipLayers->size(); ++i)
            boredomLevels.push_back(0.0);

    }

    KUKADU_SHARED_PTR<std::vector<int> > ProjectiveSimulator::getIntermediateHopIdx() {
        return intermediateHops;
    }

    ProjectiveSimulator::~ProjectiveSimulator() {

        rankVec.clear();
        intermediateHops->clear();
        actionClips->clear();
        perceptClips->clear();
        for(auto& layer : *clipLayers) {
            for(auto& cl : *layer)
                // required to get rid of the memory leak (removing pointer cycles)
                cl->clearClip();
            layer->clear();
        }
        clipLayers->clear();

        reward = nullptr;
        predefinedFirstHop = nullptr;
        lastVisitedClip = nullptr;
        reward = nullptr;
        lastClipBeforeAction = nullptr;

        lastActionClip = nullptr;
        lastPerceptClip = nullptr;
        generator = nullptr;
        lastGeneralizedPercept = nullptr;
        intermediateHops = nullptr;
        actionClips = nullptr;
        perceptClips = nullptr;
        clipLayers = nullptr;

    }

    void ProjectiveSimulator::eliminateClip(KUKADU_SHARED_PTR<Clip> currClip) {

        KUKADU_MODULE_START_USAGE();

        int level = currClip->getLevel();
        KUKADU_SHARED_PTR<set<KUKADU_SHARED_PTR<Clip>, clip_compare> > currLayer = clipLayers->at(level);
        currLayer->erase(currClip);
        set<KUKADU_SHARED_PTR<Clip> > parents = std::set<KUKADU_SHARED_PTR<Clip> >(currClip->getParents()->begin(), currClip->getParents()->end());

        set<KUKADU_SHARED_PTR<Clip> >::iterator it;
        for(it = parents.begin(); it != parents.end(); ++it) {
            KUKADU_SHARED_PTR<Clip> parent = *it;
            parent->removeSubClip(currClip);
        }

        int originalMode = PS_USE_ORIGINAL;

        if(level == 0 && operationMode != originalMode)
            perceptClips->erase(std::find(perceptClips->begin(), perceptClips->end() + 1, currClip));

        currClip->removeAllSubClips();

        KUKADU_MODULE_END_USAGE();

    }

    int ProjectiveSimulator::getStandardImmunity() {
        return immunityThresh;
    }

    void ProjectiveSimulator::cleanByRank() {

        KUKADU_MODULE_START_USAGE();

        int clipNumber = rankVec.size();

        int alreadyDeleted = 0;
        int toDelete = clipNumber - maxNumberOfClips;

        if(clipNumber > maxNumberOfClips) {

            for(int i = 0; i < clipNumber; ++i) {

                KUKADU_SHARED_PTR<Clip> currClip = rankVec.at(i).second;
                if(!currClip->isImmune()) {

                    ++alreadyDeleted;
                    eliminateClip(currClip);
                    rankVec.erase(rankVec.begin() + i);
                    --i;
                    --clipNumber;

                    if(alreadyDeleted >= toDelete)
                        return;

                }
            }
        }

        // if too many clips were immune, then start deleting the rest
        if(alreadyDeleted < toDelete) {

            for(int i = 0; alreadyDeleted < toDelete; ++i, ++alreadyDeleted) {

                KUKADU_SHARED_PTR<Clip> currClip = rankVec.at(i).second;
                eliminateClip(currClip);

                rankVec.erase(rankVec.begin() + i);
                --i;

            }

        }

        KUKADU_MODULE_END_USAGE();

    }

    void ProjectiveSimulator::setStandardImmunity(int immunity) {
        this->immunityThresh = immunity;
    }

    void ProjectiveSimulator::setMaxNumberOfClips(int maxNumberOfClips) {
        this->maxNumberOfClips = maxNumberOfClips;
    }

    KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<ActionClip> > > ProjectiveSimulator::getActionClips() {
        return actionClips;
    }

    bool ProjectiveSimulator::lastHopWasBored() {
        return lastRunWasBored;
    }

    bool ProjectiveSimulator::nextHopIsBored() {

        // if nextHopIsBored was already called for the same hop before without walking in between
        // then this function should deliver the same result
        if(!walkedFurtherSinceLastBoredom)
            return lastBoredomResult;

        if(lastVisitedClip)
            lastBoredomResult = computeBoredom(lastVisitedClip);
        walkedFurtherSinceLastBoredom = false;

        return lastBoredomResult;

    }

    KUKADU_SHARED_PTR<Clip> ProjectiveSimulator::getLastVisitedClip() {
        return lastVisitedClip;
    }

    std::pair<int, KUKADU_SHARED_PTR<Clip> > ProjectiveSimulator::performRandomWalk(int untilLevel, bool continueLastWalk) {

        KUKADU_MODULE_START_USAGE();

        KUKADU_SHARED_PTR<Clip> previousClip;
        KUKADU_SHARED_PTR<Clip> currentClip;

        int previousIdx = 0;
        int currentLevel = 0;

        int originalMode = PS_USE_ORIGINAL;
        int genMode = PS_USE_GEN;

        if(!continueLastWalk) {

            currentLevel = 0;

            walkedFurtherSinceLastBoredom = true;
            lastRunWasBored = false;
            lastClipBeforeAction.reset();
            intermediateHops->clear();

            if(!predefinedFirstHop) {
                if(operationMode == genMode) {
                    if(!lastGeneralizedPercept) {
                        cerr << "(ProjectiveSimulator) you have to generalize before you walk" << endl;
                        throw KukaduException("(ProjectiveSimulator) you have to generalize before you walk");
                    } else {
                        currentClip = lastGeneralizedPercept;
                    }
                } else if(operationMode == originalMode) {
                    currentClip = reward->generateNextPerceptClip(immunityThresh);
                }
            } else
                currentClip = predefinedFirstHop;

            std::vector<KUKADU_SHARED_PTR<PerceptClip> >::iterator it = std::find(perceptClips->begin(), perceptClips->end() + 1, currentClip);
            lastVisitedPreviousIdx = previousIdx = it - perceptClips->begin();
            intermediateHops->push_back(previousIdx);

            lastPerceptClip = KUKADU_DYNAMIC_POINTER_CAST<PerceptClip>(currentClip);

        } else {

            currentLevel = lastVisitedLevel;
            currentClip = lastVisitedClip;
            previousIdx = lastVisitedPreviousIdx;

        }

        auto startLevel = currentLevel;
        lastVisitedClip = currentClip;
        auto isBored = nextHopIsBored();

        while(previousClip != currentClip && currentLevel != untilLevel) {

            pair<int, KUKADU_SHARED_PTR<Clip> > nextHop;
            lastClipBeforeAction = previousClip;
            previousClip = currentClip;

            if(!isBored) {

                nextHop = currentClip->jumpNextRandom();
                lastVisitedPreviousIdx = previousIdx = nextHop.first;
                currentClip = nextHop.second;

            } else {

                lastRunWasBored = true;
                return pair<int, KUKADU_SHARED_PTR<Clip> > (currentClip->getLevel(), currentClip);

            }
            isBored = nextHopIsBored();

            if(currentLevel != startLevel)
                walkedFurtherSinceLastBoredom = true;

            ++currentLevel;
            lastVisitedClip = currentClip;
            lastVisitedLevel = currentLevel;

            intermediateHops->push_back(previousIdx);

        }

        // if previous and current clip are the same, then the last entry is duplicate in the intermediate hops --> remove it
        if(previousClip == currentClip)
            intermediateHops->resize(intermediateHops->size() - 1);

        lastActionClip = KUKADU_DYNAMIC_POINTER_CAST<ActionClip>(currentClip);

        KUKADU_MODULE_END_USAGE();

        return {currentClip->getLevel(), currentClip};

    }

    bool ProjectiveSimulator::computeBoredom(KUKADU_SHARED_PTR<Clip> clip) {

        KUKADU_MODULE_START_USAGE();

        bool beingBored = false;
        auto clipLevel = clip->getLevel();

        if(clipLevel != Clip::CLIP_H_LEVEL_FINAL) {

            auto boredom = boredomLevels.at(clipLevel);
            if(boredomLevels.at(clipLevel) > 0.0) {

                double entropy = clip->computeSubEntropy();
                double numberOfSubclips = clip->getSubClipCount();

                // b * (1 - H / H_max) = 1 - b * H / log2(N)
                double boredomScore = 1.0 - boredom * entropy / log2(numberOfSubclips);

                // cout << "boredom score: " << boredomScore << endl;

                vector<double> boredomDistWeights;
                boredomDistWeights.push_back(boredomScore);
                boredomDistWeights.push_back(1 - boredomScore);

                KUKADU_DISCRETE_DISTRIBUTION<int> boredomDist = KUKADU_DISCRETE_DISTRIBUTION<int>(boredomDistWeights.begin(), boredomDistWeights.end());

                beingBored =  (1 - boredomDist(*generator)) ? true : false;

            }

        }

        KUKADU_MODULE_END_USAGE();

        return beingBored;

    }

    std::tuple<bool, double, vector<int> > ProjectiveSimulator::performRewarding() {

        KUKADU_MODULE_START_USAGE();

        double computedReward = 0.0;
        if(!lastRunWasBored) {

            computedReward = reward->computeReward(lastPerceptClip, lastActionClip);

            if(doTraining) {

                KUKADU_SHARED_PTR<std::set<KUKADU_SHARED_PTR<Clip>, clip_compare> > currLevel;
                for(int i = 0; i < clipLayers->size(); ++i) {

                    currLevel = clipLayers->at(i);

                    set<KUKADU_SHARED_PTR<Clip> >::iterator currIt;
                    for(currIt = currLevel->begin(); currIt != currLevel->end(); ++currIt) {
                        KUKADU_SHARED_PTR<Clip> currClip = *currIt;
                        currClip->updateWeights(computedReward, gamma);

                        // decrease immunity
                        if(useRanking)
                            currClip->decreaseImmunity();
                    }

                }

                if(useRanking) {

                    computeRankVec();
                    cleanByRank();

                }
            }

        }

        lastRunWasBored = false;

        KUKADU_MODULE_END_USAGE();

        return make_tuple(lastRunWasBored, computedReward, *getIntermediateHopIdx());

    }

    void ProjectiveSimulator::generalize(KUKADU_SHARED_PTR<PerceptClip> nextClip) {

        KUKADU_MODULE_START_USAGE();

        lastGeneralizedPercept = nextClip;

        KUKADU_SHARED_PTR<std::set<KUKADU_SHARED_PTR<Clip>, clip_compare> > toConnect = createNewClips(nextClip);

        // first connect everything...
        set<KUKADU_SHARED_PTR<Clip>, clip_compare>::iterator it;
        for(it = toConnect->begin(); it != toConnect->end(); ++it) {
            KUKADU_SHARED_PTR<Clip> con = *it;
            if(PS_PRINT_DEBUG_INFO)
                cout << "(ProjectiveSimulator) calling connect function for " << *con << endl;

            if(useRanking)
                con->setPreviousRank();

            connectNewClip(con);
        }

        toConnect.reset();

        KUKADU_MODULE_END_USAGE();

    }

    void ProjectiveSimulator::printWeights() {

        int level = 0;

        std::vector<KUKADU_SHARED_PTR<std::set<KUKADU_SHARED_PTR<Clip>, clip_compare> > >::iterator it;
        for(it = clipLayers->begin(); it != clipLayers->end(); ++it) {

            KUKADU_SHARED_PTR<set<KUKADU_SHARED_PTR<Clip>, clip_compare> > currLevel = *it;
            cout << "(ProjectiveSimulator) clips on layer " << level << endl << "=========================" << endl;

            set<KUKADU_SHARED_PTR<Clip>, clip_compare>::iterator currIt;
            for(currIt = currLevel->begin(); currIt != currLevel->end(); ++currIt) {
                KUKADU_SHARED_PTR<Clip> currClip = *currIt;
                int subClipCount = currClip->getSubClipCount();
                for(int i = 0; i < subClipCount; ++i) {
                    cout << "(ProjectiveSimulator) " << *currClip << " --> " << *currClip->getSubClipByIdx(i) << " with weight " << currClip->getWeightByIdx(i) << endl;
                }
            }

            ++level;

        }

    }

    void ProjectiveSimulator::connectNewClip(KUKADU_SHARED_PTR<Clip> conClip) {

        KUKADU_MODULE_START_USAGE();

        int currentLevel = 0;

        std::vector<KUKADU_SHARED_PTR<std::set<KUKADU_SHARED_PTR<Clip>, clip_compare> > >::iterator it;
        for(it = clipLayers->begin(); it != clipLayers->end(); ++it) {
            KUKADU_SHARED_PTR<set<KUKADU_SHARED_PTR<Clip>, clip_compare> > currLevel = *it;
            if(currLevel->size() > 0 && conClip->getLevel() != currentLevel) {

                set<KUKADU_SHARED_PTR<Clip>, clip_compare>::iterator currIt;
                for(currIt = currLevel->begin(); currIt != currLevel->end(); ++currIt) {

                    KUKADU_SHARED_PTR<Clip> currClip = *currIt;

                    // if they are compatible
                    if(currClip->isCompatibleSubclip(conClip)) {
                        currClip->addSubClip(conClip, Clip::CLIP_H_STD_WEIGHT);
                        conClip->addParent(currClip);
                    } else if(conClip->isCompatibleSubclip(currClip)) {
                        conClip->addSubClip(currClip, Clip::CLIP_H_STD_WEIGHT);
                        currClip->addParent(conClip);
                    } else {
                        // dont connect, they are not compatible
                    }

                }

            }

            ++currentLevel;
        }

        KUKADU_MODULE_END_USAGE();

    }

    KUKADU_SHARED_PTR<std::set<KUKADU_SHARED_PTR<Clip>, clip_compare> > ProjectiveSimulator::createNewClips(KUKADU_SHARED_PTR<PerceptClip> newClip) {

        KUKADU_MODULE_START_USAGE();

        KUKADU_SHARED_PTR<set<KUKADU_SHARED_PTR<Clip>, clip_compare> > conClips = KUKADU_SHARED_PTR<set<KUKADU_SHARED_PTR<Clip>, clip_compare> >(new set<KUKADU_SHARED_PTR<Clip>, clip_compare>());

        // insert new percept clip
        pair<set<KUKADU_SHARED_PTR<Clip>, clip_compare>::iterator, bool> inserted = clipLayers->at(0)->insert(newClip);

        // check if new clip was already there
        if(inserted.second) {

            // if it wasnt there yet, add it to the new clips that have to be freshly connected
            conClips->insert(newClip);
            perceptClips->push_back(KUKADU_DYNAMIC_POINTER_CAST<PerceptClip>(newClip));

            // check in each level, if there will be new clips
            std::vector<KUKADU_SHARED_PTR<std::set<KUKADU_SHARED_PTR<Clip>, clip_compare> > >::iterator it;
            for(it = clipLayers->begin(); it != clipLayers->end(); ++it) {

                KUKADU_SHARED_PTR<set<KUKADU_SHARED_PTR<Clip>, clip_compare> > currLevel = *it;
                KUKADU_SHARED_PTR<Clip> firstClipOnLevel;
                firstClipOnLevel.reset();
                if(currLevel->size())
                    firstClipOnLevel = *(currLevel->begin());
                if(currLevel->size() && firstClipOnLevel->getLevel() != Clip::CLIP_H_LEVEL_FINAL) {

                    set<KUKADU_SHARED_PTR<Clip>, clip_compare>::iterator currIt;
                    for(currIt = currLevel->begin(); currIt != currLevel->end(); ++currIt) {

                        KUKADU_SHARED_PTR<Clip> currClip = *currIt;

                        // create new clip that gets generated as a cascade
                        KUKADU_SHARED_PTR<Clip> nextClip = currClip->compareClip(newClip);

                        // check if already there and insert it if not
                        pair<set<KUKADU_SHARED_PTR<Clip>, clip_compare>::iterator, bool> inserted2 = clipLayers->at(nextClip->getLevel())->insert(nextClip);

                        // if new && not already there
                        if(inserted2.second && nextClip != currClip) {

                            nextClip = *inserted2.first;

                            // insert to the new set of clips that should be connected
                            pair<set<KUKADU_SHARED_PTR<Clip>, clip_compare>::iterator, bool> inserted3 = conClips->insert(nextClip);


                        }
                        // if same clip is there already, but new instance, then delete the instance
                        else if(newClip != nextClip) {
                            nextClip.reset();
                        }

                    }

                }
                else break;
            }

        }

        KUKADU_MODULE_END_USAGE();

        return conClips;

    }

    int ProjectiveSimulator::generateNewActionId() {
        return ++maxActionId;
    }

    int ProjectiveSimulator::generateNewPerceptId() {
        return ++maxPerceptId;
    }

    KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<std::set<KUKADU_SHARED_PTR<Clip>, clip_compare> > > > ProjectiveSimulator::getClipLayers() {
        return clipLayers;
    }

    KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<PerceptClip> > > ProjectiveSimulator::getPerceptClips() {
        return perceptClips;
    }

    bool compareRanks(std::pair<double, KUKADU_SHARED_PTR<Clip> > p1, std::pair<double, KUKADU_SHARED_PTR<Clip> > p2) {
        return (p1.first < p2.first);
    }

    void ProjectiveSimulator::computeRankVec() {

        KUKADU_MODULE_START_USAGE();

        rankVec.clear();
        for(int i = 0; i < clipLayers->size() - 1; i++) {

            KUKADU_SHARED_PTR<set<KUKADU_SHARED_PTR<Clip>, clip_compare> > currLevel = clipLayers->at(i);

            set<KUKADU_SHARED_PTR<Clip>, clip_compare>::iterator currIt;
            for(currIt = currLevel->begin(); currIt != currLevel->end(); ++currIt) {

                KUKADU_SHARED_PTR<Clip> currClip = *currIt;
                rankVec.push_back(pair<double, KUKADU_SHARED_PTR<Clip> >(currClip->computeRank(), currClip));

            }
        }
        std::sort(rankVec.begin(), rankVec.end(), compareRanks);

        if(PS_PRINT_RANKING_DEBUG_INFO)
            printRankVec();

        KUKADU_MODULE_END_USAGE();

    }

    int ProjectiveSimulator::getClipCount() {
        return rankVec.size();
    }

    void ProjectiveSimulator::printRankVec() {

        cout << rankVec.size() << endl;

    }

    bool ProjectiveSimulator::fileExists(const std::string filePath) {
        ifstream f(filePath.c_str());
        if (f.good()) {
            f.close();
            return true;
        } else {
            f.close();
            return false;
        }
    }

    void ProjectiveSimulator::storePS(std::string targetFile) {

        KUKADU_MODULE_START_USAGE();

        int deleteFile = 0;
        if(fileExists(targetFile)) {
            cout << "(ProjectiveSimulator) file already exists. do you want to overwrite it? (0 = no / 1 = yes)" << endl;
            cin >> deleteFile;
            if(deleteFile == 1) {
                // do nothing and overwrite it later
            } else {
                cout << "(ProjectiveSimulator) no overwriting selected. skip storing" << endl;
                return;
            }
        }

        psFile = targetFile;
        loadedFromFile = true;

        ofstream outFile;
        outFile.open(targetFile.c_str(), ios::trunc);

        outFile << "V1.0" << endl;
        outFile << "general properties" << endl;
        outFile << "operationMode=" << operationMode << endl;
        outFile << "ranking=" << useRanking << endl;
        outFile << "gamma=" << gamma << endl;
        outFile << "maxNumberOfClips=" << maxNumberOfClips << endl;
        outFile << "immunityThresh=" << immunityThresh << endl;
        outFile << "levels=" << clipLayers->size() << endl;
        outFile << endl;

        outFile << "clips" << endl;
        std::vector<KUKADU_SHARED_PTR<std::set<KUKADU_SHARED_PTR<Clip>, clip_compare> > >::iterator it;
        for(it = clipLayers->begin(); it != clipLayers->end(); ++it) {

            KUKADU_SHARED_PTR<set<KUKADU_SHARED_PTR<Clip>, clip_compare> > layer = *it;
            if(layer->size() > 0) {

                int currentLevel = (*layer->begin())->getLevel();
                outFile << "layer=" << currentLevel << endl;

                set<KUKADU_SHARED_PTR<Clip>, clip_compare>::iterator currIt;
                for(currIt = layer->begin(); currIt != layer->end(); ++currIt) {

                    KUKADU_SHARED_PTR<Clip> cClip = *currIt;
                    outFile << cClip->getIdVecString() << ";" << *cClip << ";" << cClip->getCurrentImmunity();
                    if(currentLevel == 0) {
                        KUKADU_SHARED_PTR<PerceptClip> cpc = KUKADU_DYNAMIC_POINTER_CAST<PerceptClip>(cClip);
                        outFile << ";" << cpc->getPerceptId();
                    } else if(currentLevel == Clip::CLIP_H_LEVEL_FINAL) {
                        KUKADU_SHARED_PTR<ActionClip> cpc = KUKADU_DYNAMIC_POINTER_CAST<ActionClip>(cClip);
                        outFile << ";" << cpc->getActionId();
                    }

                    outFile << endl;

                }

            }

        }
        outFile << endl;

        outFile << "connections" << endl;
        for(it = clipLayers->begin(); it != clipLayers->end(); ++it) {

            KUKADU_SHARED_PTR<set<KUKADU_SHARED_PTR<Clip>, clip_compare> > layer = *it;
            if(layer->size() > 0) {
                int currentLevel = (*layer->begin())->getLevel();
                if(currentLevel != Clip::CLIP_H_LEVEL_FINAL) {
                    outFile << "layer=" << currentLevel << endl;

                    set<KUKADU_SHARED_PTR<Clip>, clip_compare>::iterator currIt;
                    for(currIt = layer->begin(); currIt != layer->end(); ++currIt) {

                        KUKADU_SHARED_PTR<Clip> cClip = *currIt;
                        int subClipCount = cClip->getSubClipCount();
                        outFile << cClip->getIdVecString() << ":" << endl;
                        for(int j = 0;  j < subClipCount; ++j) {
                            outFile << cClip->getSubClipByIdx(j)->getIdVecString() << ";" << cClip->getWeightByIdx(j) << endl;
                        }
                        outFile << endl;
                    }
                }
            }
        }

        KUKADU_MODULE_END_USAGE();

    }

    PerceptClip::PerceptClip(int perceptId, std::string label, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, std::string clipDimensionValues, int immunity) : Clip(0, generator, clipDimensionValues, immunity) {
        construct(perceptId, label);
    }

    PerceptClip::PerceptClip(int perceptId, std::string label, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, KUKADU_SHARED_PTR<std::vector<int> > clipDimensionValues, int immunity) : Clip(0, generator, clipDimensionValues, immunity) {
        construct(perceptId, label);
    }

    void PerceptClip::construct(int perceptId, std::string label) {
        this->perceptId = perceptId;
        this->label = label;
    }

    int PerceptClip::getPerceptId() {
        return perceptId;
    }

    std::string PerceptClip::getLabel() {
        return label;
    }

    std::string PerceptClip::toString() const {
        return label;
    }

}
