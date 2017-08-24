#include <kukadu/generated_skills/SensingSlide.hpp>
#include <kukadu/generated_skills/BlockingPos.hpp>
#include <kukadu/generated_skills/CloseHand.hpp>
#include <kukadu/generated_skills/PressButtonController.hpp>
#include <kukadu/generated_skills/ShelfAlignment.hpp>
#include <kukadu/generated_skills/FinalPush.hpp>
#include <kukadu/generated_skills/PushHandPos.hpp>
#include <kukadu/generated_skills/PushTranslation.hpp>
#include <kukadu/generated_skills/PushForward.hpp>
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
        throw kukadu::KukaduException("(utils) hardware to search is not in the list");
    }

	SensingSlide::SensingSlide(kukadu::StorageSingleton& storage, std::vector< KUKADU_SHARED_PTR< kukadu::Hardware > > hardware)
	 : SensingController(storage, kukadu::SkillFactory::get().getGenerator(), kukadu::SensingController::HAPTIC_MODE_CLASSIFIER,
    "SensingSlide",
    {retrieveHardwareWithId <ControlQueue> (hardware, "kukie_left_arm")},
    {retrieveHardwareWithId <GenericHand>(hardware, "kukiehand_left")},
                         "/tmp/", 1.0) {
		this->hardware = hardware;
	}

	std::vector<KUKADU_SHARED_PTR< kukadu::Hardware> > SensingSlide::getUsedHardware() {
		return hardware;
	}

	bool SensingSlide::requiresGraspInternal() {
		return false;
	}

	bool SensingSlide::producesGraspInternal() {
		return false;
	}

	void SensingSlide::prepare() {
		auto sLeftQueue6000 = getUsedHardware()[0];
	sLeftQueue6000->install();
	sLeftQueue6000->start();

	auto skill600 = kukadu::SkillFactory::get().loadSkill("ChangeModeToIMP", {sLeftQueue6000});

	skill600->execute();

	auto sLeftQueue6010 = getUsedHardware()[1];
	sLeftQueue6010->install();
	sLeftQueue6010->start();

	auto skill601 = kukadu::SkillFactory::get().loadSkill("PushHandPos", {sLeftQueue6010});

	skill601->execute();

	auto sLeftQueue6020 = getUsedHardware()[2];
	sLeftQueue6020->install();
	sLeftQueue6020->start();

	auto skill602 = kukadu::SkillFactory::get().loadSkill("ChangeModeToIMP", {sLeftQueue6020});

	skill602->execute();

	auto sLeftQueue6030 = getUsedHardware()[2];
	sLeftQueue6030->install();
	sLeftQueue6030->start();

	auto skill603 = kukadu::SkillFactory::get().loadSkill("ChangeStiffness", {sLeftQueue6030});

	std::dynamic_pointer_cast<kukadu::ChangeStiffness>(skill603)->setStiffnessType(0);
	std::dynamic_pointer_cast<kukadu::ChangeStiffness>(skill603)->setStandardStiffnessDamping(-1);

	skill603->execute();

	auto sLeftQueue6040 = getUsedHardware()[0];
	sLeftQueue6040->install();
	sLeftQueue6040->start();

	auto skill604 = kukadu::SkillFactory::get().loadSkill("ChangeStiffness", {sLeftQueue6040});

	std::dynamic_pointer_cast<kukadu::ChangeStiffness>(skill604)->setStiffnessType(0);
	std::dynamic_pointer_cast<kukadu::ChangeStiffness>(skill604)->setStandardStiffnessDamping(-1);

	skill604->execute();

	auto sLeftQueue6050 = getUsedHardware()[2];
	sLeftQueue6050->install();
	sLeftQueue6050->start();

	auto skill605 = kukadu::SkillFactory::get().loadSkill("MoveHome", {sLeftQueue6050});

	skill605->execute();

	auto sLeftQueue6060 = getUsedHardware()[0];
	sLeftQueue6060->install();
	sLeftQueue6060->start();

	auto skill606 = kukadu::SkillFactory::get().loadSkill("BlockingPos", {sLeftQueue6060});

	skill606->execute();

	auto sLeftQueue6070 = getUsedHardware()[3];
	sLeftQueue6070->install();
	sLeftQueue6070->start();

	auto skill607 = kukadu::SkillFactory::get().loadSkill("RightHandBlocking", {sLeftQueue6070});

	skill607->execute();

	auto sLeftQueue6080 = getUsedHardware()[2];
	sLeftQueue6080->install();
	sLeftQueue6080->start();
	auto sLeftQueue6081 = getUsedHardware()[1];
	sLeftQueue6081->install();
	sLeftQueue6081->start();
	auto sLeftQueue6082 = getUsedHardware()[4];
	sLeftQueue6082->install();
	sLeftQueue6082->start();

	auto skill608 = kukadu::SkillFactory::get().loadSkill("PushTranslation", {sLeftQueue6080, sLeftQueue6081, sLeftQueue6082});

	skill608->execute();

	auto sLeftQueue6090 = getUsedHardware()[2];
	sLeftQueue6090->install();
	sLeftQueue6090->start();
	auto sLeftQueue6091 = getUsedHardware()[1];
	sLeftQueue6091->install();
	sLeftQueue6091->start();
	auto sLeftQueue6092 = getUsedHardware()[4];
	sLeftQueue6092->install();
	sLeftQueue6092->start();

	auto skill609 = kukadu::SkillFactory::get().loadSkill("PushTranslation", {sLeftQueue6090, sLeftQueue6091, sLeftQueue6092});

	skill609->execute();

	auto sLeftQueue6100 = getUsedHardware()[2];
	sLeftQueue6100->install();
	sLeftQueue6100->start();
	auto sLeftQueue6101 = getUsedHardware()[1];
	sLeftQueue6101->install();
	sLeftQueue6101->start();

	auto skill610 = kukadu::SkillFactory::get().loadSkill("FinalPush", {sLeftQueue6100, sLeftQueue6101});

	skill610->execute();

	auto sLeftQueue6110 = getUsedHardware()[2];
	sLeftQueue6110->install();
	sLeftQueue6110->start();

	auto skill611 = kukadu::SkillFactory::get().loadSkill("ChangeStiffness", {sLeftQueue6110});

	std::dynamic_pointer_cast<kukadu::ChangeStiffness>(skill611)->setStiffnessType(0);
	std::dynamic_pointer_cast<kukadu::ChangeStiffness>(skill611)->setStandardStiffnessDamping(0.3);

	skill611->execute();

	auto sLeftQueue6120 = getUsedHardware()[1];
	sLeftQueue6120->install();
	sLeftQueue6120->start();

	auto skill612 = kukadu::SkillFactory::get().loadSkill("JointPtp", {sLeftQueue6120});

	std::dynamic_pointer_cast<kukadu::JointPtp>(skill612)->setJoints({0, -1.5, -1, 0, -0.2, -1.5, -1});

	skill612->execute();

	auto sLeftQueue6130 = getUsedHardware()[2];
	sLeftQueue6130->install();
	sLeftQueue6130->start();

	auto skill613 = kukadu::SkillFactory::get().loadSkill("SimpleJointPtp", {sLeftQueue6130});

	std::dynamic_pointer_cast<kukadu::SimpleJointPtp>(skill613)->setJoints({-0.4, 1.7, 1.86, -0.65, 0.05, 1.19, -1.98});

	skill613->execute();

	auto sLeftQueue6140 = getUsedHardware()[1];
	sLeftQueue6140->install();
	sLeftQueue6140->start();

	auto skill614 = kukadu::SkillFactory::get().loadSkill("JointPtp", {sLeftQueue6140});

	std::dynamic_pointer_cast<kukadu::JointPtp>(skill614)->setJoints({0, -1.5, -1, 0.1, -0.1, -1.5, -1});

	skill614->execute();

	auto sLeftQueue6150 = getUsedHardware()[3];
	sLeftQueue6150->install();
	sLeftQueue6150->start();
	auto sLeftQueue6151 = getUsedHardware()[0];
	sLeftQueue6151->install();
	sLeftQueue6151->start();

	auto skill615 = kukadu::SkillFactory::get().loadSkill("PushForward", {sLeftQueue6150, sLeftQueue6151});

	std::dynamic_pointer_cast<kukadu::PushForward>(skill615)->setGoBackToBlockingPos(false);

	skill615->execute();

	auto sLeftQueue6160 = getUsedHardware()[1];
	sLeftQueue6160->install();
	sLeftQueue6160->start();

	auto skill616 = kukadu::SkillFactory::get().loadSkill("JointPtp", {sLeftQueue6160});

	std::dynamic_pointer_cast<kukadu::JointPtp>(skill616)->setJoints({0, -1.5, -1, 0, -0.2, -1.5, -1});

	skill616->execute();

	auto sLeftQueue6170 = getUsedHardware()[0];
	sLeftQueue6170->install();
	sLeftQueue6170->start();

	auto skill617 = kukadu::SkillFactory::get().loadSkill("ChangeStiffness", {sLeftQueue6170});

	std::dynamic_pointer_cast<kukadu::ChangeStiffness>(skill617)->setStiffnessType(1);

	skill617->execute();


	}
	void SensingSlide::cleanUp() {

	auto sLeftQueue6180 = getUsedHardware()[1];
	sLeftQueue6180->install();
	sLeftQueue6180->start();

	auto skill618 = kukadu::SkillFactory::get().loadSkill("JointPtp", {sLeftQueue6180});

	std::dynamic_pointer_cast<kukadu::JointPtp>(skill618)->setJoints({0, -1.5, -1, 0.4, 1.2, -1.5, -1});

	skill618->execute();


	}
	void SensingSlide::performCore() {

	auto sLeftQueue6190 = getUsedHardware()[0];
	sLeftQueue6190->install();
	sLeftQueue6190->start();

	auto skill619 = kukadu::SkillFactory::get().loadSkill("ChangeStiffness", {sLeftQueue6190});

	std::dynamic_pointer_cast<kukadu::ChangeStiffness>(skill619)->setStiffnessType(0);
	std::dynamic_pointer_cast<kukadu::ChangeStiffness>(skill619)->setStandardStiffnessDamping(-1);

	skill619->execute();

	auto sLeftQueue6200 = getUsedHardware()[0];
	sLeftQueue6200->install();
	sLeftQueue6200->start();

	auto skill620 = kukadu::SkillFactory::get().loadSkill("BlockingPos", {sLeftQueue6200});

	skill620->execute();

	auto sLeftQueue6210 = getUsedHardware()[2];
	sLeftQueue6210->install();
	sLeftQueue6210->start();

	auto skill621 = kukadu::SkillFactory::get().loadSkill("MoveHome", {sLeftQueue6210});

	skill621->execute();

	auto sLeftQueue6220 = getUsedHardware()[1];
	sLeftQueue6220->install();
	sLeftQueue6220->start();

	auto skill622 = kukadu::SkillFactory::get().loadSkill("PushHandPos", {sLeftQueue6220});

	skill622->execute();

	auto sLeftQueue6230 = getUsedHardware()[3];
	sLeftQueue6230->install();
	sLeftQueue6230->start();
	auto sLeftQueue6231 = getUsedHardware()[0];
	sLeftQueue6231->install();
	sLeftQueue6231->start();

	auto skill623 = kukadu::SkillFactory::get().loadSkill("PushForward", {sLeftQueue6230, sLeftQueue6231});

	std::dynamic_pointer_cast<kukadu::PushForward>(skill623)->setGoBackToBlockingPos(true);

	skill623->execute();

	}

	std::string SensingSlide::getClassName() {
		return "SensingSlide";
	}

	KUKADU_SHARED_PTR<kukadu::SensingController> SensingSlide::clone() {
		return std::make_shared<SensingSlide>(getStorage(), getUsedHardware());
	}
}
