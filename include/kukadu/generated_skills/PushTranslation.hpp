#ifndef KUKADU_GENERATED_SKILLS_PUSHTRANSLATION_H
#define KUKADU_GENERATED_SKILLS_PUSHTRANSLATION_H

#include <kukadu/robot.hpp>
#include <kukadu/control.hpp>
#include <kukadu/manipulation/skillfactory.hpp>
#include <kukadu/manipulation/playing/controllers.hpp>
#include <kukadu/vision/poseestimatorfactory.hpp>

namespace kukadu {
	class PushTranslation : public kukadu::Controller {

	private:

	protected:

		virtual void createSkillFromThisInternal(std::string skillName);

	public:

		PushTranslation(kukadu::StorageSingleton& storage, std::vector< KUKADU_SHARED_PTR< kukadu::Hardware > > hardware);

		bool requiresGraspInternal();

		bool producesGraspInternal();

		std::shared_ptr<kukadu::ControllerResult> executeInternal();

		std::string getClassName();

	};
}

#endif
