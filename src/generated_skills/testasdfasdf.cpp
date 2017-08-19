#include <kukadu/generated_skills/testasdfasdf.hpp>
namespace kukadu {
	testasdfasdf::testasdfasdf(kukadu::StorageSingleton& storage, std::vector< KUKADU_SHARED_PTR< kukadu::Hardware > > hardware)
 : Controller(storage, "testasdfasdf", hardware, 0.01) {

}

bool testasdfasdf::requiresGraspInternal() {
	return false;
}

bool testasdfasdf::producesGraspInternal() {
	return false;
}

std::shared_ptr<kukadu::ControllerResult> testasdfasdf::executeInternal() {
	auto& hardwareFactory = kukadu::HardwareFactory::get();
		auto sLeftQueue410 = getUsedHardware()[0];
		sLeftQueue410->install();
		sLeftQueue410->start();

		auto skill41 = kukadu::SkillFactory::get().loadSkill("MoveHome", {sLeftQueue410});

		skill41->execute();

	return nullptr;
}

std::string testasdfasdf::getClassName() {
	return "testasdfasdf";
}

void testasdfasdf::createSkillFromThisInternal(std::string skillName) {
	// nothing to do
}
}
