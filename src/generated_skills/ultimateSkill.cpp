#import <kukadu/generated_skills/ultimateSkill.hpp>
namespace kukadu {
	namespace skill
		{ultimateSkill::ultimateSkill(kukadu::StorageSingleton& storage, std::vector< KUKADU_SHARED_PTR< kukadu::Hardware > > hardware)
 : Controller(storage, "ultimateSkill", hardware, 0.01) {

	this->hardware = hardware;
}

bool ultimateSkill::requiresGraspInternal() {
	return false;
}

bool ultimateSkill::producesGraspInternal() {
	return false;
}

std::shared_ptr<kukadu::ControllerResult> ultimateSkill::executeInternal() {
	auto hardwareFactory = kukadu::HardwareFactory::get();
		auto simLeftQueue250 = hardwareFactory.loadHardware("kukie_left_arm");
		simLeftQueue250->install();
		simLeftQueue250->start();

		auto skill25 = kukadu::SkillFactory::get().loadSkill("final_push_dmp", {simLeftQueue250});

		skill25->execute();

}

std::string ultimateSkill::getClassName() {
	return "ultimateSkill";
}

void ultimateSkill::createSkillFromThisInternal(std::string skillName) {
	// nothing to do
}
}
}
