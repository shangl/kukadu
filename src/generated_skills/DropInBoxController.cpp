#include <kukadu/generated_skills/DropInBoxController.hpp>
#include <kukadu/manipulation/skillfactory.hpp>

namespace kukadu {
	DropInBoxController::DropInBoxController(kukadu::StorageSingleton& storage, std::vector< KUKADU_SHARED_PTR< kukadu::Hardware > > hardware)
 : Controller(storage, "DropInBoxController", hardware, 0.01) {

}

bool DropInBoxController::requiresGraspInternal() {
	return false;
}

bool DropInBoxController::producesGraspInternal() {
	return false;
}

std::shared_ptr<kukadu::ControllerResult> DropInBoxController::executeInternal() {
		auto sLeftQueue2110 = getUsedHardware()[0];
		sLeftQueue2110->install();
		sLeftQueue2110->start();

		auto skill211 = kukadu::SkillFactory::get().loadSkill("JointPtp", {sLeftQueue2110});

		std::dynamic_pointer_cast<kukadu::JointPtp>(skill211)->setJoints({-1.29, 1.26, 2.82, -1.07, -2.87, -0.85, 0.67});

		skill211->execute();

		auto sLeftQueue2120 = getUsedHardware()[1];
		sLeftQueue2120->install();
		sLeftQueue2120->start();

		auto skill212 = kukadu::SkillFactory::get().loadSkill("JointPtp", {sLeftQueue2120});

		std::dynamic_pointer_cast<kukadu::JointPtp>(skill212)->setJoints({0, -1.3, 0, -1.3, 0, -1.3, 0});

		skill212->execute();

	return nullptr;
}

std::string DropInBoxController::getClassName() {
	return "DropInBoxController";
}

void DropInBoxController::createSkillFromThisInternal(std::string skillName) {
	// nothing to do
}
}
