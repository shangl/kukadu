#include <vector>
#include <utility>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <kukadu/utils/kukadutokenizer.hpp>
#include <kukadu/storage/moduleusagesingleton.hpp>
#include <kukadu/learning/projective_simulation/clips.hpp>

using namespace std;

namespace kukadu {

    ActionClip::ActionClip(int actionId, int perceptDimensionality, std::string label, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator) : Clip(CLIP_H_LEVEL_FINAL, generator, KUKADU_SHARED_PTR<vector<int> >(new vector<int>()), INT_MAX) {
        this->actionId = actionId;
        this->label = label;
        KUKADU_SHARED_PTR<vector<int> > dummyVals = KUKADU_SHARED_PTR<vector<int> >(new vector<int>());
        for(int i = 0; i < perceptDimensionality; ++i)
            dummyVals->push_back(-actionId - 1);
        setClipDimensionValues(dummyVals);
    }

    int ActionClip::getActionId() {
        return actionId;
    }

    std::string ActionClip::getLabel() {
        return label;
    }

    std::pair<int, KUKADU_SHARED_PTR<Clip> > ActionClip::jumpNextRandom() {
        KUKADU_MODULE_START_USAGE();
        pair<int, KUKADU_SHARED_PTR<Clip> > retPair{0, shared_from_this()};
        KUKADU_MODULE_END_USAGE();
        return retPair;
    }

    std::string ActionClip::toString() const {
        return label;
    }

    Clip::Clip(int level, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, std::string clipValues, int immunity) {

        construct(level, generator, Clip::getIdVectorFromString(clipValues), immunity);

    }

    Clip::Clip(int level, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, KUKADU_SHARED_PTR<std::vector<int> > clipDimensionValues, int immunity) {

        construct(level, generator, clipDimensionValues, immunity);

    }

    void Clip::construct(int level, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, KUKADU_SHARED_PTR<std::vector<int> > clipValues, int immunity) {

        this->level = level;
        this->generator = generator;
        this->clipDimensionValues = clipValues;
        this->subClips = KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<Clip> > >(new std::vector<KUKADU_SHARED_PTR<Clip> >());
        this->parents = KUKADU_SHARED_PTR<std::set<KUKADU_SHARED_PTR<Clip> > >(new std::set<KUKADU_SHARED_PTR<Clip> >());
        this->subClipsSet = KUKADU_SHARED_PTR<std::set<KUKADU_SHARED_PTR<Clip> > >(new std::set<KUKADU_SHARED_PTR<Clip> >());
        this->visitedSubNode = CLIP_H_NOT_WALKED_YET;
        this->previousRank = -1;
        this->initialImmunity = this->immunity = immunity;
        this->gotDeleted = 0;

        nextHop = NEXT_HOP_NOT_PREDEF;

    }

    void Clip::setNextHop(int hopIdx) {
        nextHop = hopIdx;
    }

    void Clip::setNextHop(KUKADU_SHARED_PTR<Clip> hopClip) {
        setNextHop(getSubClipIdx(hopClip));
    }

    KUKADU_SHARED_PTR<Clip> Clip::getSubClipByLabel(std::string idx) {
        for(auto cl : *subClips) {
            if(cl->toString() == idx)
                return cl;
        }
        return nullptr;
    }

    void Clip::setSpecificWeight(KUKADU_SHARED_PTR<Clip> child, double weight) {
        subH.at(getSubClipIdx(child)) = weight;
    }

    std::tuple<double, int, KUKADU_SHARED_PTR<Clip> > Clip::getMaxProbability() {

        KUKADU_MODULE_START_USAGE();

        int maxWeight = 0;
        double maxWeightSum = 0.0;
        double maxProbability = 0.0;
        KUKADU_SHARED_PTR<Clip> maxProbClip = subClips->front();

        int subClipCount = subClips->size();

        for(int i = 0; i < subClipCount; ++i) {

            double currProb = subH.at(i);
            maxWeightSum += currProb;
            if(currProb > maxProbability) {
                maxProbClip = subClips->at(i);
                maxProbability = currProb;
                maxWeight = currProb;
            }

        }

        auto retVal = make_tuple(maxProbability / maxWeightSum, maxWeight, maxProbClip);

        KUKADU_MODULE_END_USAGE();

        return retVal;

    }

    KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<Clip> > > Clip::getSubClips() {
        return subClips;
    }

    int Clip::getSubClipIdx(KUKADU_SHARED_PTR<Clip> subClip) {

        KUKADU_MODULE_START_USAGE();

        int retVal = 0;

        auto foundClipIt = std::find(subClips->begin(), subClips->end(), subClip);
        if(subClip == *foundClipIt)
            retVal = (foundClipIt - subClips->begin());
        else
            throw KukaduException("(Clip) clip not found");

        KUKADU_MODULE_END_USAGE();

        return retVal;

    }

    KUKADU_SHARED_PTR<std::vector<int> > Clip::getIdVectorFromString(std::string str) {

        KUKADU_SHARED_PTR<std::vector<int> > retVec = KUKADU_SHARED_PTR<std::vector<int> >(new std::vector<int>());
        KukaduTokenizer tok(str, "(),");

        string t = "";

        while((t = tok.next()).compare(""))
            retVec->push_back(atoi(t.c_str()));

        return retVec;

    }

    void Clip::clearClip() {

        subClips->clear();
        parents->clear();
        subClipsSet->clear();
        clipDimensionValues->clear();

    }

    Clip::~Clip() {

        clearClip();

        subClips.reset();
        parents.reset();
        subClipsSet.reset();
        clipDimensionValues.reset();

    }

    double Clip::computeSubEntropy() const {

        KUKADU_MODULE_START_USAGE();

        double entropy = 0.0;
        vector<double> probs = discDist.probabilities();

        for(int i = 0; i < probs.size(); ++i) {
            double prob = probs.at(i);
            entropy -= prob * log2(prob);
        }

        KUKADU_MODULE_END_USAGE();

        return entropy;

    }

    void Clip::setImmunity(int immunity) {
        this->immunity = immunity;
    }

    bool Clip::isImmune() {
        return immunity;
    }

    int Clip::getCurrentImmunity() {
        return immunity;
    }

    void Clip::decreaseImmunity() {
        if(immunity > 0)
            --immunity;
    }

    void Clip::addParent(KUKADU_SHARED_PTR<Clip> par) {
        parents->insert(par);
    }

    void Clip::addParentDownwards(KUKADU_SHARED_PTR<Clip> par) {

        addParent(par);
        par->addSubClip(shared_from_this(), CLIP_H_STD_WEIGHT);

        for(int i = 0; i < subClips->size(); ++i) {
            KUKADU_SHARED_PTR<Clip> child = subClips->at(i);
            child->addParentDownwards(par);
        }

    }

    void Clip::addChildUpwards(KUKADU_SHARED_PTR<Clip> sub) {

        this->addSubClip(sub, CLIP_H_STD_WEIGHT);

        set<KUKADU_SHARED_PTR<Clip> >::iterator it;
        for(it = parents->begin(); it != parents->end(); ++it) {
            KUKADU_SHARED_PTR<Clip> parent = *it;
            parent->addChildUpwards(sub);
        }

    }

    std::pair<int, KUKADU_SHARED_PTR<Clip> > Clip::jumpNextRandom() {

        KUKADU_MODULE_START_USAGE();

        if(nextHop == NEXT_HOP_NOT_PREDEF)
            visitedSubNode = discDist(*generator);
        else {
            visitedSubNode = nextHop;
            nextHop = NEXT_HOP_NOT_PREDEF;
        }

        pair<int, KUKADU_SHARED_PTR<Clip> > retVal{visitedSubNode, subClips->at(visitedSubNode)};

        KUKADU_MODULE_END_USAGE();

        return retVal;

    }

    void Clip::initRandomGenerator() {

        discDist = KUKADU_DISCRETE_DISTRIBUTION<int>(subH.begin(), subH.end());

    }

    void Clip::addSubClip(KUKADU_SHARED_PTR<Clip> sub, int weight) {

        pair<set<KUKADU_SHARED_PTR<Clip> >::iterator, bool> inserted = subClipsSet->insert(sub);
        if(inserted.second) {
            subClips->push_back(sub);
            subH.push_back(weight);
            sub->addParent(shared_from_this());
            initRandomGenerator();
        }

    }

    void Clip::setChildren(KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<Clip> > > children) {

        int childCount = children->size();
        std::vector<double> newH;

        for(int i = 0; i < childCount; ++i) {
            newH.push_back(CLIP_H_STD_WEIGHT);
        }

        setChildren(children, newH);

    }

    void Clip::setChildren(KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<Clip> > > children, std::vector<double> weights) {

        subClips.reset();
        this->subClips = children;
        this->subH = weights;

        for(auto child : *children)
            child->addParent(shared_from_this());

        initRandomGenerator();

    }

    void Clip::addHAndInitialize(int idx, int addWeight) {

        subH.at(idx) = addWeight;
        initRandomGenerator();

    }

    int Clip::getSubClipCount() {
        return subH.size();

    }

    void Clip::setPreviousRank() {
        previousRank = computeRank();
    }

    int Clip::getPreviousRank() {
        return previousRank;
    }

    // sum outgoing weights - initial weights * number of outgoing edges
    double Clip::computeRank() const {

        KUKADU_MODULE_START_USAGE();

        double outWeights = std::accumulate(subH.begin(), subH.end(), 0.0);
        double outNum = subH.size();

        double retVal = outWeights - CLIP_H_STD_WEIGHT * outNum;

        KUKADU_MODULE_END_USAGE();

        return retVal;

    }

    void Clip::updateWeights(double reward, double gamma) {

        KUKADU_MODULE_START_USAGE();

        int subClipCount = subH.size();
        for(int i = 0; i < subClipCount; ++i) {

            double currentWeight = subH.at(i);
            if(visitedSubNode != CLIP_H_NOT_WALKED_YET && i == visitedSubNode)
                subH.at(i) = max(1.0, currentWeight - gamma * (currentWeight - 1) + reward);
            else
                subH.at(i) = currentWeight - gamma * (currentWeight - 1);

        }
        visitedSubNode = CLIP_H_NOT_WALKED_YET;
        initRandomGenerator();

        KUKADU_MODULE_END_USAGE();

    }

    void Clip::printSubWeights() {
        for(int i = 0; i < subH.size(); ++i) {
            cout << subH.at(i) << " (" << *subClips->at(i) << ") ";
        }
        cout << endl;
    }

    bool Clip::compareIdVecs(const KUKADU_SHARED_PTR<std::vector<int> > vec1, const KUKADU_SHARED_PTR<std::vector<int> > vec2) {

        int s1 = vec1->size();
        if(s1 == vec2->size()) {
            for(int i = 0; i < s1; ++i)
                if(vec1->at(i) != vec2->at(i))
                    return false;
            return true;
        }
        throw KukaduException("(Clip==) clip dimensions do not match");

    }

    bool operator== (const Clip &o1, const Clip &o2) {

        return Clip::compareIdVecs(o1.getClipDimensions(), o2.getClipDimensions());

    }

    bool operator!= (const Clip &o1, const Clip &o2) {
        return !(o1 == o2);
    }

    bool operator< (const Clip &o1, const Clip &o2) {

        int s1 = o1.clipDimensionValues->size();
        if(s1 == o2.clipDimensionValues->size()) {
            for(int i = 0; i < s1; ++i)
                if(o1.clipDimensionValues->at(i) == o2.clipDimensionValues->at(i)) {}
                else return o1.clipDimensionValues->at(i) < o2.clipDimensionValues->at(i);
        }
        return false;
        throw KukaduException("(Clip==) clip dimensions do not match");

    }

    bool Clip::isCompatibleSubclip(KUKADU_SHARED_PTR<Clip> c) {

        // if c is action clip --> its always a compatible subclip
        if(c->getLevel() == CLIP_H_LEVEL_FINAL)
            return true;
        else if(this->getLevel() == CLIP_H_LEVEL_FINAL)
            return false;

        KUKADU_SHARED_PTR<std::vector<int> > cClips = c->clipDimensionValues;
        int clipSize = cClips->size();
        if(this->clipDimensionValues->size() == clipSize) {
            for(int i = 0; i < cClips->size(); ++i) {
                auto clipHashVal = Clip::CLIP_H_HASH_VAL;
                if(this->clipDimensionValues->at(i) != cClips->at(i) && cClips->at(i) != clipHashVal)
                    return false;
            }
            return true;
        }
        throw KukaduException("(Clip checkCompatible) clip dimensions do not match");

    }

    void Clip::setClipDimensionValues(KUKADU_SHARED_PTR<std::vector<int> > vals) {
        this->clipDimensionValues = vals;
    }

    KUKADU_SHARED_PTR<Clip> Clip::getSubClipByIdx(int idx) {
        return subClips->at(idx);
    }

    double Clip::getWeightByIdx(int idx) {
        return subH.at(idx);
    }

    string Clip::toString() const {

        stringstream ss;
        ss << "(";
        for(int i = 0; i < clipDimensionValues->size(); ++i) {
            int dim = clipDimensionValues->at(i);
            auto clipHashVal = Clip::CLIP_H_HASH_VAL;
            if(dim != clipHashVal)
                ss << dim << ", ";
            else
                ss << "#, ";
        }

        string retStr = ss.str();
        return retStr.substr(0, retStr.length() - 2) + ")";

    }

    std::ostream& operator<<(std::ostream &strm, const Clip &c) {
        return strm << c.toString();
    }

    KUKADU_SHARED_PTR<std::vector<int> > Clip::getClipDimensions() const {
        return clipDimensionValues;
    }

    KUKADU_SHARED_PTR<std::set<KUKADU_SHARED_PTR<Clip> > > Clip::getParents() {
        return parents;
    }

    void Clip::removeAllSubClips() {
        vector<KUKADU_SHARED_PTR<Clip> > subClipCopy(*subClips);
        for(int i = 0; i < subClipCopy.size(); ++i) {
            KUKADU_SHARED_PTR<Clip> subClip = subClipCopy.at(i);
            removeSubClipWoRand(subClip);
        }
        gotDeleted = 1;
    }

    void Clip::removeParentClip(KUKADU_SHARED_PTR<Clip> c) {
        parents->erase(c);
    }

    void Clip::removeSubClip(KUKADU_SHARED_PTR<Clip> clip) {

        removeSubClipWoRand(clip);
        initRandomGenerator();

    }

    void Clip::removeSubClipWoRand(KUKADU_SHARED_PTR<Clip> clip) {

        subClipsSet->erase(clip);
        vector<KUKADU_SHARED_PTR<Clip> >::iterator foundIt = std::find(subClips->begin(), subClips->end() + 1, clip);
        int elIdx = foundIt - subClips->begin();
        subClips->erase(foundIt);
        subH.erase(subH.begin() + elIdx);
        clip->removeParentClip(shared_from_this());

    }

    int Clip::getInitialImmunity() {
        return initialImmunity;
    }

    int Clip::getMaxH() {
        return getLikeliestChildWithWeight().first;
    }

    KUKADU_SHARED_PTR<Clip> Clip::getLikeliestChild() {
        return getLikeliestChildWithWeight().second;
    }

    std::pair<int, KUKADU_SHARED_PTR<Clip> > Clip::getLikeliestChildWithWeight() {

        KUKADU_MODULE_START_USAGE();

        auto maxIt = std::max_element(subH.begin(), subH.end());
        int maxIdx = maxIt - subH.begin();
        pair<int, KUKADU_SHARED_PTR<Clip> > retVal{*maxIt, subClips->at(maxIdx)};

        KUKADU_MODULE_END_USAGE();

        return retVal;

    }

    KUKADU_SHARED_PTR<Clip> Clip::compareClip(KUKADU_SHARED_PTR<Clip> c) {

        KUKADU_SHARED_PTR<vector<int> > cVec = c->clipDimensionValues;
        int cVecSize = cVec->size();
        int thisVecSize = clipDimensionValues->size();
        if(cVecSize != thisVecSize)
            return KUKADU_SHARED_PTR<Clip>();

        int hashCount = 0;
        KUKADU_SHARED_PTR<vector<int> > clipValues = KUKADU_SHARED_PTR<vector<int> >(new vector<int>());
        for(int i = 0, val = 0; i < cVecSize; ++i) {
            auto clipHashVal = Clip::CLIP_H_HASH_VAL;
            if((val = cVec->at(i)) == clipDimensionValues->at(i) && cVec->at(i) != clipHashVal)
                clipValues->push_back(val);
            else {
                clipValues->push_back(clipHashVal);
                ++hashCount;
            }
        }

        // level is given by number of hashes
        if(hashCount)
            return KUKADU_SHARED_PTR<Clip>(new Clip(hashCount, generator, clipValues, this->getInitialImmunity()));
        else {
            // with shared pointers it is important that this line is never executed --> return shared_from_this instead
            return shared_from_this();
        }

    }

    int Clip::getDimensionality() {
        return clipDimensionValues->size();
    }

    int Clip::getLevel() {
        return level;
    }

    std::string Clip::getIdVecString() const {

        stringstream s;
        s << "(";

        bool isFirst = true;
        for(int i = 0; i < clipDimensionValues->size(); ++i) {
            int val = clipDimensionValues->at(i);
            if(isFirst)
                isFirst = false;
            else
                s << ", ";

            s << val;
        }
        s << ")";

        return s.str();

    }

}
