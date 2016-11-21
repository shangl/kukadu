#ifndef KUKADU_PLAYINGCORE_H
#define KUKADU_PLAYINGCORE_H

#include <string>
#include <vector>
#include <kukadu/control/controller.hpp>
#include <kukadu/types/controllerresult.hpp>
#include <kukadu/manipulation/playing/controllers.hpp>
#include <kukadu/learning/projective_simulation/core.hpp>

namespace kukadu {

    class HapticPlanner : public kukadu::Reward, public KUKADU_ENABLE_SHARED_FROM_THIS<HapticPlanner> {

    private:

        bool computeMeanAndVariance;
        std::vector<KUKADU_SHARED_PTR<SensingController> > meanAndVarianceForSensingIds;

        std::string skillDatabase;

        std::map<std::string, KUKADU_SHARED_PTR<kukadu::Controller> > registeredComplexControllers;

        std::vector<KUKADU_SHARED_PTR<kukadu::SensingController> > requiresGraspSensingControllersVec;
        std::vector<KUKADU_SHARED_PTR<kukadu::SensingController> > nonRequiresGraspSensingControllersVec;

        std::map<std::string, KUKADU_SHARED_PTR<kukadu::SensingController> > requiresGraspSensingControllers;
        std::map<std::string, KUKADU_SHARED_PTR<kukadu::SensingController> > nonRequiresGraspSensingControllers;

        std::map<std::string, KUKADU_SHARED_PTR<kukadu::Controller> > allPrepControllers;
        std::map<std::string, KUKADU_SHARED_PTR<kukadu::Controller> > preparationProducesGraspControllers;
        std::map<std::string, KUKADU_SHARED_PTR<kukadu::Controller> > preparationProducesNonGraspControllers;

        std::vector<KUKADU_SHARED_PTR<kukadu::Controller> > preparationProducesGraspControllersVector;
        std::vector<KUKADU_SHARED_PTR<kukadu::Controller> > preparationProducesNonGraspControllersVector;

        KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator;

        void printNamedVector(std::vector<std::string> names);

        std::vector<KUKADU_SHARED_PTR<kukadu::SensingController> > copySensingControllers(KUKADU_SHARED_PTR<kukadu::ComplexController> parentComplexController,
                                                                                          std::vector<KUKADU_SHARED_PTR<kukadu::SensingController> > controllers,
                                                                                          std::string newBasePath);

    protected:

        virtual double computeRewardInternal(KUKADU_SHARED_PTR<PerceptClip> providedPercept, KUKADU_SHARED_PTR<ActionClip> takenAction);

    public:

        HapticPlanner(std::string skillDatabase,
                      std::vector<KUKADU_SHARED_PTR<kukadu::SensingController> > sensingControllers,
                      std::vector<KUKADU_SHARED_PTR<kukadu::Controller> > preparatoryControllers,
                      std::vector<KUKADU_SHARED_PTR<kukadu::Controller> > complexControllers,
                      KUKADU_SHARED_PTR<kukadu::Controller> nothingController,
                      KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator,
                      std::vector<KUKADU_SHARED_PTR<SensingController> > meanAndVarianceForSensingIds = {});

        virtual int getDimensionality();

        virtual KUKADU_SHARED_PTR<PerceptClip> generateNextPerceptClip(int immunity);
        virtual KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<ActionClip> > > generateActionClips();
        virtual KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<PerceptClip> > > generatePerceptClips();

        void pickAndPerformComplexSkill(bool updateModels = true);
        KUKADU_SHARED_PTR<kukadu::HapticControllerResult> performComplexSkill(std::string skillId, bool updateModels = true);

        void updateModels();

        std::string pickComplexSkill();

        void setSimulationMode(bool simulationMode);

    };

}

#endif // HAPTICPLANNER_H
