#include <kukadu/generated_skills/PressButtonController.hpp>
#include <kukadu/generated_skills/ChangeStiffness.hpp>
#include <kukadu/manipulation/skillfactory.hpp>

namespace kukadu {
	PressButtonController::PressButtonController(kukadu::StorageSingleton& storage, std::vector< KUKADU_SHARED_PTR< kukadu::Hardware > > hardware)
 : Controller(storage, "PressButtonController", hardware, 0.01) {

}

bool PressButtonController::requiresGraspInternal() {
	return false;
}

bool PressButtonController::producesGraspInternal() {
	return false;
}

std::shared_ptr<kukadu::ControllerResult> PressButtonController::executeInternal() {
		auto sLeftQueue3210 = getUsedHardware()[0];
		sLeftQueue3210->install();
		sLeftQueue3210->start();

		auto skill321 = kukadu::SkillFactory::get().loadSkill("ChangeStiffness", {sLeftQueue3210});

		std::dynamic_pointer_cast<kukadu::ChangeStiffness>(skill321)->setStiffnessType(1);

		skill321->execute();

		auto sLeftQueue3220 = getUsedHardware()[0];
		sLeftQueue3220->install();
		sLeftQueue3220->start();

		auto skill322 = kukadu::SkillFactory::get().loadSkill("press_button_dmp", {sLeftQueue3220});

		std::dynamic_pointer_cast<kukadu::DMPExecutor>(skill322)->setExecutionMode(kukadu::TrajectoryExecutor::EXECUTE_ROBOT);
		std::dynamic_pointer_cast<kukadu::DMPExecutor>(skill322)->setAc(0);

		skill322->execute();

		auto sLeftQueue3230 = getUsedHardware()[0];
		sLeftQueue3230->install();
		sLeftQueue3230->start();

		auto skill323 = kukadu::SkillFactory::get().loadSkill("ChangeStiffness", {sLeftQueue3230});

		std::dynamic_pointer_cast<kukadu::ChangeStiffness>(skill323)->setStiffnessType(1);
		std::dynamic_pointer_cast<kukadu::ChangeStiffness>(skill323)->setStandardStiffnessDamping(-1);

		skill323->execute();

	return nullptr;
}

std::string PressButtonController::getClassName() {
	return "PressButtonController";
}

void PressButtonController::createSkillFromThisInternal(std::string skillName) {
	// nothing to do
}
}
