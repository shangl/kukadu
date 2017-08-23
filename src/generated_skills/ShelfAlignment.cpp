#include <kukadu/generated_skills/ShelfAlignment.hpp>
#include <kukadu/manipulation/skillfactory.hpp>

namespace kukadu {
	ShelfAlignment::ShelfAlignment(kukadu::StorageSingleton& storage, std::vector< KUKADU_SHARED_PTR< kukadu::Hardware > > hardware)
 : Controller(storage, "ShelfAlignment", hardware, 0.01) {

}

bool ShelfAlignment::requiresGraspInternal() {
	return false;
}

bool ShelfAlignment::producesGraspInternal() {
	return false;
}

std::shared_ptr<kukadu::ControllerResult> ShelfAlignment::executeInternal() {
		auto sLeftQueue3900 = getUsedHardware()[0];
		sLeftQueue3900->install();
		sLeftQueue3900->start();

		auto skill390 = kukadu::SkillFactory::get().loadSkill("JointPtp", {sLeftQueue3900});

		std::dynamic_pointer_cast<kukadu::JointPtp>(skill390)->setJoints({-1.73, 1.11, 2.57, -1.88, -1.1, -1.73, 0.83});

		skill390->execute();

		auto sLeftQueue3910 = getUsedHardware()[1];
		sLeftQueue3910->install();
		sLeftQueue3910->start();

		auto skill391 = kukadu::SkillFactory::get().loadSkill("JointPtp", {sLeftQueue3910});

		std::dynamic_pointer_cast<kukadu::JointPtp>(skill391)->setJoints({0, 0.35, 0.2, 0, 0, 0.35, 0.2});

		skill391->execute();

		auto sLeftQueue3920 = getUsedHardware()[0];
		sLeftQueue3920->install();
		sLeftQueue3920->start();

		auto skill392 = kukadu::SkillFactory::get().loadSkill("shelf_align_dmp", {sLeftQueue3920});

		std::dynamic_pointer_cast<kukadu::DMPExecutor>(skill392)->setExecutionMode(kukadu::TrajectoryExecutor::EXECUTE_ROBOT);

		skill392->execute();

		auto sLeftQueue3930 = getUsedHardware()[0];
		sLeftQueue3930->install();
		sLeftQueue3930->start();

		auto skill393 = kukadu::SkillFactory::get().loadSkill("JointPtp", {sLeftQueue3930});

		std::dynamic_pointer_cast<kukadu::JointPtp>(skill393)->setJoints({-1.45, 1.63, 2.92, -2.02, -1.43, -1, 0.83});

		skill393->execute();

	return nullptr;
}

std::string ShelfAlignment::getClassName() {
	return "ShelfAlignment";
}

void ShelfAlignment::createSkillFromThisInternal(std::string skillName) {
	// nothing to do
}
}
