#include <kukadu/utils/utils.hpp>
#include <kukadu/types/kukadutypes.hpp>
#include <kukadu/manipulation/playing/core.hpp>
#include <kukadu/storage/moduleusagesingleton.hpp>
#include <kukadu/manipulation/playing/controllers.hpp>

using namespace std;

namespace kukadu {

    HapticPlanner::HapticPlanner(std::string skillDatabase,
                                 std::vector<KUKADU_SHARED_PTR<kukadu::SensingController> > sensingControllers,
                                 std::vector<KUKADU_SHARED_PTR<kukadu::Controller> > preparatoryControllers,
                                 std::vector<KUKADU_SHARED_PTR<kukadu::Controller> > complexControllers,
                                 KUKADU_SHARED_PTR<kukadu::Controller> nothingController,
                                 KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, std::vector<KUKADU_SHARED_PTR<SensingController> > meanAndVarianceForSensingIds) : Reward(generator, false) {

        if(!preparatoryControllers.size())
            throw KukaduException("(HapticPlanner) no preparatory controllers defined");

        if(!sensingControllers.size())
            throw KukaduException("(HapticPlanner) no sensing controllers defined");

        if(meanAndVarianceForSensingIds.size() > 0)
            this->computeMeanAndVariance = true;
        else
            this->computeMeanAndVariance = false;

        this->meanAndVarianceForSensingIds = meanAndVarianceForSensingIds;
        this->generator = generator;

        preparePathString(skillDatabase);

        this->skillDatabase = skillDatabase;

        for(auto sensCont : sensingControllers) {
            if(sensCont->requiresGrasp()) {
                requiresGraspSensingControllers.insert(std::pair<std::string, KUKADU_SHARED_PTR<kukadu::SensingController> >(sensCont->getCaption(), sensCont));
                requiresGraspSensingControllersVec.push_back(sensCont);
            } else {
                nonRequiresGraspSensingControllers.insert(std::pair<std::string, KUKADU_SHARED_PTR<kukadu::SensingController> >(sensCont->getCaption(), sensCont));
                nonRequiresGraspSensingControllersVec.push_back(sensCont);
            }
        }

        // "do nothing" controller is available for all actions (no matter if they require grasped objects or not)
        allPrepControllers.insert(std::pair<std::string, KUKADU_SHARED_PTR<kukadu::Controller> >(nothingController->getCaption(), nothingController));
        preparationProducesGraspControllers.insert(std::pair<std::string, KUKADU_SHARED_PTR<kukadu::Controller> >(nothingController->getCaption(), nothingController));
        preparationProducesGraspControllersVector.push_back(nothingController);
        preparationProducesNonGraspControllers.insert(std::pair<std::string, KUKADU_SHARED_PTR<kukadu::Controller> >(nothingController->getCaption(), nothingController));
        preparationProducesNonGraspControllersVector.push_back(nothingController);
        for(auto prepCont : preparatoryControllers) {

            allPrepControllers.insert(std::pair<std::string, KUKADU_SHARED_PTR<kukadu::Controller> >(prepCont->getCaption(), prepCont));

            // this logics has problems --> e.g. in the case of the complex grasp controller (just add all controllers)
            preparationProducesGraspControllers.insert(std::pair<std::string, KUKADU_SHARED_PTR<kukadu::Controller> >(prepCont->getCaption(), prepCont));
            preparationProducesGraspControllersVector.push_back(prepCont);
            preparationProducesNonGraspControllers.insert(std::pair<std::string, KUKADU_SHARED_PTR<kukadu::Controller> >(prepCont->getCaption(), prepCont));
            preparationProducesNonGraspControllersVector.push_back(prepCont);

        }

        // also add complex controllers
        for(auto compCont : complexControllers)
            allPrepControllers.insert(std::pair<std::string, KUKADU_SHARED_PTR<kukadu::Controller> >(compCont->getCaption(), compCont));

        for(auto compCont : complexControllers) {

            KUKADU_SHARED_PTR<kukadu::ComplexController> castCompCont = KUKADU_DYNAMIC_POINTER_CAST<kukadu::ComplexController>(compCont);
            string contName = compCont->getCaption();
            replace(contName.begin(), contName.end(), ' ', '_');

            string complexPath = skillDatabase + contName;
            preparePathString(complexPath);
            replace(complexPath.begin(), complexPath.end(), ' ', '_');

            string hapticPath = complexPath + "haptics";
            preparePathString(hapticPath);

            if(!fileExists(complexPath))
                // initialize haptic planner in general (load controllers and create ps)
                createDirectory(complexPath);

            if(!fileExists(hapticPath))
                createDirectory(hapticPath);

            cout << "creating controller " << contName << endl;
            std::vector<KUKADU_SHARED_PTR<kukadu::SensingController> > sensingCopy;
            // sensing and complex controllers are assumed to require the same (grasped or not grasped)

            if(castCompCont->requiresGrasp())
                sensingCopy = copySensingControllers(castCompCont, requiresGraspSensingControllersVec, hapticPath);
            else
                sensingCopy = copySensingControllers(castCompCont, nonRequiresGraspSensingControllersVec, hapticPath);

            std::map<std::string, KUKADU_SHARED_PTR<kukadu::SensingController> > copiedMap;
            for(auto sense : sensingCopy)
                copiedMap[sense->getCaption()] = sense;

            if(!fileExists(complexPath + "composition")) {

                // this logics has problems --> e.g. in the case of the complex grasp controller (just add all controllers)
                castCompCont->setSensingControllers(sensingCopy);
                castCompCont->setPreparatoryControllers(preparationProducesGraspControllersVector);
                for(auto cont : complexControllers)
                    if(cont != castCompCont)
                        castCompCont->addPreparatoryController(cont);

                castCompCont->createSensingDatabase();
                castCompCont->initialize();
                castCompCont->store(complexPath);

            } else {
                castCompCont->load(complexPath, copiedMap, allPrepControllers);
            }

            registeredComplexControllers.insert(std::pair<std::string, KUKADU_SHARED_PTR<kukadu::Controller> >(contName, compCont));

        }

    }

    void HapticPlanner::pickAndPerformComplexSkill(bool updateFiles) {

        KUKADU_MODULE_START_USAGE();

        auto selectedId = pickComplexSkill();
        performComplexSkill(selectedId, updateFiles);

        KUKADU_MODULE_END_USAGE();

    }

    KUKADU_SHARED_PTR<kukadu::HapticControllerResult> HapticPlanner::performComplexSkill(std::string skillId, bool updateModels) {

        KUKADU_MODULE_START_USAGE();

        auto complSkill = KUKADU_DYNAMIC_POINTER_CAST<ComplexController>(registeredComplexControllers[skillId]);
        auto executeModeBefore = complSkill->getExecuteBasicBehaviourOnly();
        complSkill->setExecuteBasicBehaviourOnly(false);

        // for learning, it has to cleanup afterwards as well
        auto prevCleanup = complSkill->getCleanup();
        auto prevGenerateNewGroundTruth = complSkill->getGenerateNewGroundTruth();
        complSkill->setCleanup(true);
        complSkill->setGenerateNewGroundTruth(true);

        auto result = KUKADU_DYNAMIC_POINTER_CAST<HapticControllerResult>(complSkill->execute());

        complSkill->setCleanup(prevCleanup);
        complSkill->setGenerateNewGroundTruth(prevGenerateNewGroundTruth);

        if(computeMeanAndVariance) {
            auto meanAndVar = complSkill->computeEntropyMeanAndVariance(meanAndVarianceForSensingIds);
            std::map<std::string, std::vector<double> > entropies;
            std::map<std::string, std::pair<double, double> > meanAndVariances;
            for(auto& entrop : meanAndVar) {
                entropies[entrop.first] = get<2>(entrop.second);
                meanAndVariances[entrop.first] = {get<0>(entrop.second), get<1>(entrop.second)};
            }

            result->setEntropyMeanAndVariance(meanAndVariances);
            result->setEntropies(entropies);
        }

        if(updateModels)
            complSkill->updateFiles();

        complSkill->setExecuteBasicBehaviourOnly(executeModeBefore);

        KUKADU_MODULE_END_USAGE();
        return result;

    }

    void HapticPlanner::updateModels() {

        KUKADU_MODULE_START_USAGE();

        for(auto complSkill : registeredComplexControllers)
            KUKADU_DYNAMIC_POINTER_CAST<ComplexController>(complSkill.second)->updateFiles();

        KUKADU_MODULE_END_USAGE();

    }

    void HapticPlanner::setSimulationMode(bool simulationMode) {

        KUKADU_MODULE_START_USAGE();

        for(auto c : registeredComplexControllers)
            c.second->setSimulationMode(simulationMode);

        KUKADU_MODULE_END_USAGE();

    }

    std::string HapticPlanner::pickComplexSkill() {

        KUKADU_MODULE_START_USAGE();

        int selection = 0;
        map<int, string> keyMap;
        cout << "select a complex skill:" << endl << "======================================" << endl;
        int i = 0;
        for(auto comp : registeredComplexControllers) {
            cout << "(" << i << ") " << comp.second->getCaption() << endl;
            keyMap.insert(pair<int, string>(i, comp.second->getCaption()));
            ++i;
        }
        cout << endl << "selection: ";
        cin >> selection;

        KUKADU_MODULE_END_USAGE();

        return keyMap[selection];

    }

    KUKADU_SHARED_PTR<PerceptClip> HapticPlanner::generateNextPerceptClip(int immunity) {
        KUKADU_MODULE_START_USAGE();
        throw KukaduException("generateNextPerceptClip not implemented yet");
        KUKADU_MODULE_END_USAGE();
    }

    KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<ActionClip> > > HapticPlanner::generateActionClips() {
        KUKADU_MODULE_START_USAGE();
        throw KukaduException("generateActionClips not implemented yet");
        KUKADU_MODULE_END_USAGE();
    }

    KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<PerceptClip> > > HapticPlanner::generatePerceptClips() {
        KUKADU_MODULE_START_USAGE();
        throw KukaduException("generatePerceptClips not implemented yet");
        KUKADU_MODULE_END_USAGE();
    }

    double HapticPlanner::computeRewardInternal(KUKADU_SHARED_PTR<PerceptClip> providedPercept, KUKADU_SHARED_PTR<ActionClip> takenAction) {
        KUKADU_MODULE_START_USAGE();
        throw KukaduException("compouteRewardInternal not implemented yet");
        KUKADU_MODULE_END_USAGE();
    }

    int HapticPlanner::getDimensionality() {
        KUKADU_MODULE_START_USAGE();
        throw KukaduException("getDimensionlity not implemented yet");
        KUKADU_MODULE_END_USAGE();
    }

    std::vector<KUKADU_SHARED_PTR<kukadu::SensingController> > HapticPlanner::copySensingControllers(KUKADU_SHARED_PTR<kukadu::ComplexController> parentComplexController, std::vector<KUKADU_SHARED_PTR<kukadu::SensingController> > controllers,
                                                                                      std::string newBasePath) {

        KUKADU_MODULE_START_USAGE();

        vector<KUKADU_SHARED_PTR<kukadu::SensingController> > retVec;
        for(auto sens : controllers) {
            auto copiedSens = sens->clone();
            copiedSens->setParentComplexController(parentComplexController);
            copiedSens->setStateCount(parentComplexController->getStateCount(copiedSens->getCaption()));
            copiedSens->setDatabasePath(newBasePath + sens->getCaption() + "/");
            retVec.push_back(copiedSens);
        }

        KUKADU_MODULE_END_USAGE();

        return retVec;

    }

}
