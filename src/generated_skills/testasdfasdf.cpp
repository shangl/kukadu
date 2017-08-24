#include <kukadu/generated_skills/testasdfasdf.hpp>
#include <kukadu/generated_skills/KinestheticTeaching.hpp>
#include <kukadu/generated_skills/PressButtonController.hpp>
#include <kukadu/generated_skills/ShelfPlacementController.hpp>
#include <kukadu/generated_skills/WaitForReached.hpp>
#include <kukadu/generated_skills/ChangeStiffness.hpp>
#include <kukadu/generated_skills/PushHandPos.hpp>
#include <kukadu/generated_skills/ChangeModeToIMP.hpp>
#include <kukadu/generated_skills/ShelfAlignment.hpp>
#include <kukadu/generated_skills/SensingPoke.hpp>
#include <kukadu/generated_skills/SimpleJointPtp.hpp>
#include <kukadu/generated_skills/OpenHand.hpp>
#include <kukadu/generated_skills/BlockingPos.hpp>
#include <kukadu/generated_skills/CloseHand.hpp>
#include <kukadu/generated_skills/RightHandBlocking.hpp>
#include <kukadu/generated_skills/MoveHome.hpp>
#include <kukadu/generated_skills/PushTranslation.hpp>
#include <kukadu/generated_skills/DropInBoxController.hpp>
#include <kukadu/generated_skills/FinalPush.hpp>
#include <kukadu/generated_skills/BookGrasping.hpp>
#include <kukadu/generated_skills/PushForward.hpp>
#include <kukadu/generated_skills/SensingSlide.hpp>

 namespace kukadu {
	template<typename T > KUKADU_SHARED_PTR< T > retrieveHardwareWithId(std::vector<KUKADU_SHARED_PTR<kukadu::Hardware> > list, std::string instanceName) {
		for(auto& hw : list) {
			if(hw->getHardwareInstanceName() == instanceName)
				return KUKADU_DYNAMIC_POINTER_CAST< T >(hw);
		}
		throw kukadu::KukaduException("(utils) hardware to search is not in the list");
	}

	testasdfasdf::testasdfasdf(kukadu::StorageSingleton& storage, std::vector< KUKADU_SHARED_PTR< kukadu::Hardware > > hardware)
	 : SensingController(storage, kukadu::SkillFactory::get().getGenerator(), kukadu::SensingController::HAPTIC_MODE_CLASSIFIER,
	"testasdfasdf", {retrieveHardwareWithId <kukadu::ControlQueue> (hardware, "kukie_left_arm")},
    {retrieveHardwareWithId <kukadu::GenericHand>(hardware, "kukiehand_left")}, "/tmp/", 1.0) {
		this->hardware = hardware;
	}

	std::vector<KUKADU_SHARED_PTR< kukadu::Hardware> > testasdfasdf::getUsedHardware() {
		return hardware;
	}

	bool testasdfasdf::requiresGraspInternal() {
		return false;
	}

	bool testasdfasdf::producesGraspInternal() {
		return false;
	}

	void testasdfasdf::prepare() {
		auto sLeftQueue180 = getUsedHardware()[0];
	sLeftQueue180->install();
	sLeftQueue180->start();

	auto skill18 = kukadu::SkillFactory::get().loadSkill("MoveHome", {sLeftQueue180});

	skill18->execute();


	}
	void testasdfasdf::cleanUp() {

	auto sLeftQueue200 = getUsedHardware()[1];
	sLeftQueue200->install();
	sLeftQueue200->start();

	auto skill20 = kukadu::SkillFactory::get().loadSkill("OpenHand", {sLeftQueue200});

	skill20->execute();

	}
	void testasdfasdf::performCore() {

	auto sLeftQueue190 = getUsedHardware()[1];
	sLeftQueue190->install();
	sLeftQueue190->start();

	auto skill19 = kukadu::SkillFactory::get().loadSkill("OpenHand", {sLeftQueue190});

	skill19->execute();


	}

	std::string testasdfasdf::getClassName() {
		return "testasdfasdf";
	}

	KUKADU_SHARED_PTR<kukadu::SensingController> testasdfasdf::clone() {
		return std::make_shared<testasdfasdf>(getStorage(), getUsedHardware());
	}
}
