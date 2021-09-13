#include <kukadu/generated_skills/ShelfPlacementController.hpp>
#include <kukadu/manipulation/skillfactory.hpp>

namespace kukadu {
	ShelfPlacementController::ShelfPlacementController(kukadu::StorageSingleton& storage, std::vector< KUKADU_SHARED_PTR< kukadu::Hardware > > hardware)
 : Controller(storage, "ShelfPlacementController", hardware, 0.01) {

}

bool ShelfPlacementController::requiresGraspInternal() {
	return false;
}

bool ShelfPlacementController::producesGraspInternal() {
	return false;
}

std::shared_ptr<kukadu::ControllerResult> ShelfPlacementController::executeInternal() {
		auto sLeftQueue8820 = getUsedHardware()[0];
		sLeftQueue8820->install();
		sLeftQueue8820->start();

		auto skill882 = kukadu::SkillFactory::get().loadSkill("place_on_shelf_dmp", {sLeftQueue8820});

		std::dynamic_pointer_cast<kukadu::DMPExecutor>(skill882)->setExecutionMode(kukadu::TrajectoryExecutor::EXECUTE_ROBOT);

		skill882->execute();

		auto sLeftQueue8830 = getUsedHardware()[1];
		sLeftQueue8830->install();
		sLeftQueue8830->start();

		auto skill883 = kukadu::SkillFactory::get().loadSkill("JointPtp", {sLeftQueue8830});

		std::dynamic_pointer_cast<kukadu::JointPtp>(skill883)->setJoints({0, -0.8, 0.5, -0.5, 0, -0.8, 0.5});

		skill883->execute();

		auto sLeftQueue8840 = getUsedHardware()[0];
		sLeftQueue8840->install();
		sLeftQueue8840->start();

		auto skill884 = kukadu::SkillFactory::get().loadSkill("JointPtp", {sLeftQueue8840});

		std::dynamic_pointer_cast<kukadu::JointPtp>(skill884)->setJoints({-1.73, 1.11, 2.57, -1.88, -1.1, -1.73, 0.83});

		skill884->execute();

	return nullptr;
}

std::string ShelfPlacementController::getClassName() {
	return "ShelfPlacementController";
}

void ShelfPlacementController::createSkillFromThisInternal(std::string skillName) {
	// nothing to do
}
}
