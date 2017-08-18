#include <kukadu/generated_skills/PushPosition.hpp>
namespace kukadu {
	namespace skill
		{PushPosition::PushPosition(kukadu::StorageSingleton& storage, std::vector< KUKADU_SHARED_PTR< kukadu::Hardware > > hardware)
 : Controller(storage, "PushPosition", hardware, 0.01) {

}

bool PushPosition::requiresGraspInternal() {
	return false;
}

bool PushPosition::producesGraspInternal() {
	return false;
}

std::shared_ptr<kukadu::ControllerResult> PushPosition::executeInternal() {
	auto& hardwareFactory = kukadu::HardwareFactory::get();
		auto sLeftQueue3080 = getUsedHardware()[0];
		sLeftQueue3080->install();
		sLeftQueue3080->start();

		auto skill308 = kukadu::SkillFactory::get().loadSkill("JointPtp", {sLeftQueue3080});

		std::dynamic_pointer_cast<kukadu::JointPtp>(skill308)->setJoints({0, -1.57, 0, -1.57, 0, -1.57, 0});

		skill308->execute();

	return nullptr;
}

std::string PushPosition::getClassName() {
	return "PushPosition";
}

void PushPosition::createSkillFromThisInternal(std::string skillName) {
	// nothing to do
}
}
}
