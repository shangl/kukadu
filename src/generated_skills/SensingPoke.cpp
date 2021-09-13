#include <kukadu/generated_skills/SensingPoke.hpp>
#include <kukadu/generated_skills/BlockingPos.hpp>
#include <kukadu/generated_skills/CloseHand.hpp>
#include <kukadu/generated_skills/PressButtonController.hpp>
#include <kukadu/generated_skills/ShelfAlignment.hpp>
#include <kukadu/generated_skills/FinalPush.hpp>
#include <kukadu/generated_skills/PushHandPos.hpp>
#include <kukadu/generated_skills/PushTranslation.hpp>
#include <kukadu/generated_skills/PushForward.hpp>
#include <kukadu/generated_skills/SensingSlide.hpp>
#include <kukadu/generated_skills/MoveHome.hpp>
#include <kukadu/generated_skills/DropInBoxController.hpp>
#include <kukadu/generated_skills/WaitForReached.hpp>
#include <kukadu/generated_skills/ChangeModeToIMP.hpp>
#include <kukadu/generated_skills/OpenHand.hpp>
#include <kukadu/generated_skills/ShelfPlacementController.hpp>
#include <kukadu/generated_skills/KinestheticTeaching.hpp>
#include <kukadu/generated_skills/RightHandBlocking.hpp>
#include <kukadu/generated_skills/ChangeStiffness.hpp>
#include <kukadu/generated_skills/SimpleJointPtp.hpp>
#include <kukadu/manipulation/skillfactory.hpp>

 namespace kukadu {

     template<typename T > KUKADU_SHARED_PTR< T > retrieveHardwareWithId(std::vector<KUKADU_SHARED_PTR<kukadu::Hardware> > list, std::string instanceName) {
         for(auto& hw : list) {
             if(hw->getHardwareInstanceName() == instanceName)
                 return KUKADU_DYNAMIC_POINTER_CAST< T >(hw);
         }
         for(auto& hw : list)
             std::cerr << hw->getHardwareInstanceName() << std::endl;
         std::cerr << "(utils) hardware " << instanceName << " not in the list" << std::endl;
         throw kukadu::KukaduException("(utils) hardware to search is not in the list");
     }

	SensingPoke::SensingPoke(kukadu::StorageSingleton& storage, std::vector< KUKADU_SHARED_PTR< kukadu::Hardware > > hardware)
	 : SensingController(storage, kukadu::SkillFactory::get().getGenerator(), kukadu::SensingController::HAPTIC_MODE_CLASSIFIER,
    "SensingPoke",
    {retrieveHardwareWithId <ControlQueue> (hardware, "kukie_left_arm")},
    {retrieveHardwareWithId <GenericHand>(hardware, "kukiehand_left")},
                         "/tmp/", 1.0,
                         hardware) {
		this->hardware = hardware;
	}

	std::vector<KUKADU_SHARED_PTR< kukadu::Hardware> > SensingPoke::getUsedHardware() {
		return hardware;
	}

	bool SensingPoke::requiresGraspInternal() {
		return false;
	}

	bool SensingPoke::producesGraspInternal() {
		return false;
	}

	void SensingPoke::prepare() {
		auto sLeftQueue360 = getUsedHardware()[0];
	sLeftQueue360->install();
	sLeftQueue360->start();

	auto skill36 = kukadu::SkillFactory::get().loadSkill("ChangeModeToIMP", {sLeftQueue360});

	skill36->execute();

	auto sLeftQueue370 = getUsedHardware()[1];
	sLeftQueue370->install();
	sLeftQueue370->start();

	auto skill37 = kukadu::SkillFactory::get().loadSkill("PushHandPos", {sLeftQueue370});

	skill37->execute();

	auto sLeftQueue380 = getUsedHardware()[0];
	sLeftQueue380->install();
	sLeftQueue380->start();

	auto skill38 = kukadu::SkillFactory::get().loadSkill("ChangeStiffness", {sLeftQueue380});

	std::dynamic_pointer_cast<kukadu::ChangeStiffness>(skill38)->setStiffnessType(0);
	std::dynamic_pointer_cast<kukadu::ChangeStiffness>(skill38)->setStandardStiffnessDamping(-1);

	skill38->execute();

	auto sLeftQueue390 = getUsedHardware()[2];
	sLeftQueue390->install();
	sLeftQueue390->start();

	auto skill39 = kukadu::SkillFactory::get().loadSkill("MoveHome", {sLeftQueue390});

	skill39->execute();

	auto sLeftQueue400 = getUsedHardware()[0];
	sLeftQueue400->install();
	sLeftQueue400->start();

	auto skill40 = kukadu::SkillFactory::get().loadSkill("BlockingPos", {sLeftQueue400});

	skill40->execute();

	auto sLeftQueue410 = getUsedHardware()[3];
	sLeftQueue410->install();
	sLeftQueue410->start();

	auto skill41 = kukadu::SkillFactory::get().loadSkill("RightHandBlocking", {sLeftQueue410});

	skill41->execute();


	}
	void SensingPoke::cleanUp() {

		auto sLeftQueue430 = getUsedHardware()[0];
		sLeftQueue430->install();
		sLeftQueue430->start();

		auto skill43 = kukadu::SkillFactory::get().loadSkill("BlockingPos", {sLeftQueue430});

		skill43->execute();

		auto sLeftQueue440 = getUsedHardware()[2];
		sLeftQueue440->install();
		sLeftQueue440->start();

		auto skill44 = kukadu::SkillFactory::get().loadSkill("MoveHome", {sLeftQueue440});

		skill44->execute();



	}
	void SensingPoke::performCore() {

		auto sLeftQueue420 = getUsedHardware()[2];
		sLeftQueue420->install();
		sLeftQueue420->start();
		auto sLeftQueue421 = getUsedHardware()[1];
		sLeftQueue421->install();
		sLeftQueue421->start();

		auto skill42 = kukadu::SkillFactory::get().loadSkill("FinalPush", {sLeftQueue420, sLeftQueue421});

		skill42->execute();

	}

	std::string SensingPoke::getClassName() {
		return "SensingPoke";
	}

	KUKADU_SHARED_PTR<kukadu::SensingController> SensingPoke::clone() {
		return std::make_shared<SensingPoke>(getStorage(), getUsedHardware());
	}
}
