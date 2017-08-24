#ifndef KUKADU_GENERATED_SKILLS_BOOKGRASPING_H
#define KUKADU_GENERATED_SKILLS_BOOKGRASPING_H

#include <kukadu/robot.hpp>
#include <kukadu/control.hpp>
#include <kukadu/manipulation/skillfactory.hpp>
#include <kukadu/manipulation/playing/controllers.hpp>

namespace kukadu {
	class BookGrasping : public kukadu::ComplexController {

	private:

	protected:

	    virtual bool requiresGraspInternal();
	    virtual bool producesGraspInternal();

	    virtual void createSkillFromThisInternal(std::string skillName);

	public:

	    BookGrasping(kukadu::StorageSingleton& storage, std::vector< KUKADU_SHARED_PTR< kukadu::Hardware > > hardware);

	    virtual void executeComplexAction();
	    virtual void cleanupAfterAction();

	    virtual std::string getClassName();
	    int getStateCount(const std::string& sensingName);
	    void prepareNextState(KUKADU_SHARED_PTR<kukadu::SensingController> cont, int currentStateIdx);
	    virtual double getSimulatedReward(KUKADU_SHARED_PTR<kukadu::IntermediateEventClip> sensingClip, KUKADU_SHARED_PTR<kukadu::Clip> stateClip, KUKADU_SHARED_PTR<kukadu::ControllerActionClip> actionClip);
	    virtual KUKADU_SHARED_PTR<kukadu::Clip> computeGroundTruthTransition(KUKADU_SHARED_PTR<kukadu::Clip> sensingClip, KUKADU_SHARED_PTR<kukadu::Clip> stateClip, KUKADU_SHARED_PTR<kukadu::Clip> actionClip);

	};}

#endif
