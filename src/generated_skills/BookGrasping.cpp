#include <kukadu/generated_skills/BookGrasping.hpp>
#include <kukadu/generated_skills/BlockingPos.hpp>
#include <kukadu/generated_skills/ChangeModeToIMP.hpp>
#include <kukadu/generated_skills/ChangeStiffness.hpp>
#include <kukadu/generated_skills/CloseHand.hpp>
#include <kukadu/generated_skills/DropInBoxController.hpp>
#include <kukadu/generated_skills/FinalPush.hpp>
#include <kukadu/generated_skills/MoveHome.hpp>
#include <kukadu/generated_skills/OpenHand.hpp>
#include <kukadu/generated_skills/PushForward.hpp>
#include <kukadu/generated_skills/PushHandPos.hpp>
#include <kukadu/generated_skills/PushTranslation.hpp>
#include <kukadu/generated_skills/RightHandBlocking.hpp>
#include <kukadu/generated_skills/SimpleJointPtp.hpp>
#include <kukadu/generated_skills/WaitForReached.hpp>
#include <kukadu/manipulation/skillfactory.hpp>

namespace kukadu {
	BookGrasping::BookGrasping(kukadu::StorageSingleton& storage, std::vector< KUKADU_SHARED_PTR< kukadu::Hardware > > hardware)
	    : ComplexController(storage, "BookGrasping", hardware, kukadu::resolvePath("$KUKADU_HOME/skills/BookGrasping"), kukadu::SkillFactory::get().getGenerator(), kukadu::SkillFactory::get().loadSkill("nothing", {})) {
}

	void BookGrasping::cleanupAfterAction() {

	}

	bool BookGrasping::requiresGraspInternal() {
	    return false;
	}

	bool BookGrasping::producesGraspInternal() {
	    return false;
	}

	std::string BookGrasping::getClassName() {
	    return "BookGrasping";
	}

	void BookGrasping::createSkillFromThisInternal(std::string skillName) {
	}

	int BookGrasping::getStateCount(const std::string& sensingName) {

	}

	double BookGrasping::getSimulatedReward(KUKADU_SHARED_PTR<kukadu::IntermediateEventClip> sensingClip,
	KUKADU_SHARED_PTR<kukadu::Clip> stateClip,
			KUKADU_SHARED_PTR<kukadu::ControllerActionClip> actionClip) {

			throw kukadu::KukaduException("(BookGrasping) simulated reward not supported for generated skills");
	}

	KUKADU_SHARED_PTR<kukadu::Clip> BookGrasping::computeGroundTruthTransition(KUKADU_SHARED_PTR<kukadu::Clip> sensingClip, KUKADU_SHARED_PTR<kukadu::Clip> stateClip, KUKADU_SHARED_PTR<kukadu::Clip> actionClip) {

		throw kukadu::KukaduException("(BookGrasping) ground truth not available for generated skills");

	}

	void BookGrasping::prepareNextState(KUKADU_SHARED_PTR<kukadu::SensingController> cont, int currentStateIdx) {
	    // nothing to do
	}

	void BookGrasping::executeComplexAction() {
		auto sLeftQueue8320 = getUsedHardware()[0];
	sLeftQueue8320->install();
	sLeftQueue8320->start();

	auto skill832 = kukadu::SkillFactory::get().loadSkill("MoveHome", {sLeftQueue8320});

	skill832->execute();

	auto sLeftQueue8330 = getUsedHardware()[1];
	sLeftQueue8330->install();
	sLeftQueue8330->start();

	auto skill833 = kukadu::SkillFactory::get().loadSkill("ChangeModeToIMP", {sLeftQueue8330});

	skill833->execute();

	auto sLeftQueue8340 = getUsedHardware()[1];
	sLeftQueue8340->install();
	sLeftQueue8340->start();

	auto skill834 = kukadu::SkillFactory::get().loadSkill("ChangeStiffness", {sLeftQueue8340});

	std::dynamic_pointer_cast<kukadu::ChangeStiffness>(skill834)->setStiffnessType(0);
	std::dynamic_pointer_cast<kukadu::ChangeStiffness>(skill834)->setStandardStiffnessDamping(-1);

	skill834->execute();

	auto sLeftQueue8350 = getUsedHardware()[0];
	sLeftQueue8350->install();
	sLeftQueue8350->start();

	auto skill835 = kukadu::SkillFactory::get().loadSkill("ChangeModeToIMP", {sLeftQueue8350});

	skill835->execute();

	auto sLeftQueue8360 = getUsedHardware()[2];
	sLeftQueue8360->install();
	sLeftQueue8360->start();
	auto sLeftQueue8361 = getUsedHardware()[1];
	sLeftQueue8361->install();
	sLeftQueue8361->start();

	auto skill836 = kukadu::SkillFactory::get().loadSkill("PushForward", {sLeftQueue8360, sLeftQueue8361});

	skill836->execute();

	auto sLeftQueue8370 = getUsedHardware()[1];
	sLeftQueue8370->install();
	sLeftQueue8370->start();

	auto skill837 = kukadu::SkillFactory::get().loadSkill("BlockingPos", {sLeftQueue8370});

	skill837->execute();

	auto sLeftQueue8380 = getUsedHardware()[0];
	sLeftQueue8380->install();
	sLeftQueue8380->start();
	auto sLeftQueue8381 = getUsedHardware()[3];
	sLeftQueue8381->install();
	sLeftQueue8381->start();
	auto sLeftQueue8382 = getUsedHardware()[4];
	sLeftQueue8382->install();
	sLeftQueue8382->start();

	auto skill838 = kukadu::SkillFactory::get().loadSkill("PushTranslation", {sLeftQueue8380, sLeftQueue8381, sLeftQueue8382});

	skill838->execute();

	auto sLeftQueue8390 = getUsedHardware()[0];
	sLeftQueue8390->install();
	sLeftQueue8390->start();
	auto sLeftQueue8391 = getUsedHardware()[3];
	sLeftQueue8391->install();
	sLeftQueue8391->start();

	auto skill839 = kukadu::SkillFactory::get().loadSkill("FinalPush", {sLeftQueue8390, sLeftQueue8391});

	skill839->execute();

	auto sLeftQueue8400 = getUsedHardware()[0];
	sLeftQueue8400->install();
	sLeftQueue8400->start();

	auto skill840 = kukadu::SkillFactory::get().loadSkill("MoveHome", {sLeftQueue8400});

	skill840->execute();

	auto sLeftQueue8410 = getUsedHardware()[3];
	sLeftQueue8410->install();
	sLeftQueue8410->start();

	auto skill841 = kukadu::SkillFactory::get().loadSkill("JointPtp", {sLeftQueue8410});

	std::dynamic_pointer_cast<kukadu::JointPtp>(skill841)->setJoints({0, -0.2, 0, -0.7, -1.2, -0.2, 0});

	skill841->execute();

	auto sLeftQueue8420 = getUsedHardware()[2];
	sLeftQueue8420->install();
	sLeftQueue8420->start();

	auto skill842 = kukadu::SkillFactory::get().loadSkill("RightHandBlocking", {sLeftQueue8420});

	skill842->execute();

	auto sLeftQueue8430 = getUsedHardware()[0];
	sLeftQueue8430->install();
	sLeftQueue8430->start();

	auto skill843 = kukadu::SkillFactory::get().loadSkill("SimpleJointPtp", {sLeftQueue8430});

	std::dynamic_pointer_cast<kukadu::SimpleJointPtp>(skill843)->setJoints({-0.35, 1.66, 1.96, -0.54, -0.13, 1.4, 1.21});

	skill843->execute();

	auto sLeftQueue8440 = getUsedHardware()[2];
	sLeftQueue8440->install();
	sLeftQueue8440->start();
	auto sLeftQueue8441 = getUsedHardware()[1];
	sLeftQueue8441->install();
	sLeftQueue8441->start();

	auto skill844 = kukadu::SkillFactory::get().loadSkill("PushForward", {sLeftQueue8440, sLeftQueue8441});

	skill844->execute();

	auto sLeftQueue8450 = getUsedHardware()[1];
	sLeftQueue8450->install();
	sLeftQueue8450->start();

	auto skill845 = kukadu::SkillFactory::get().loadSkill("ChangeStiffness", {sLeftQueue8450});

	std::dynamic_pointer_cast<kukadu::ChangeStiffness>(skill845)->setStiffnessType(1);

	skill845->execute();

	auto sLeftQueue8460 = getUsedHardware()[0];
	sLeftQueue8460->install();
	sLeftQueue8460->start();

	auto skill846 = kukadu::SkillFactory::get().loadSkill("ChangeStiffness", {sLeftQueue8460});

	std::dynamic_pointer_cast<kukadu::ChangeStiffness>(skill846)->setStiffnessType(1);

	skill846->execute();

	auto sLeftQueue8470 = getUsedHardware()[0];
	sLeftQueue8470->install();
	sLeftQueue8470->start();

	auto skill847 = kukadu::SkillFactory::get().loadSkill("SimpleJointPtp", {sLeftQueue8470});

	std::dynamic_pointer_cast<kukadu::SimpleJointPtp>(skill847)->setJoints({-0.35, 1.66, 1.96, -0.54, -0.13, 1.4, 1.21});

	skill847->execute();

	auto sLeftQueue8480 = getUsedHardware()[3];
	sLeftQueue8480->install();
	sLeftQueue8480->start();

	auto skill848 = kukadu::SkillFactory::get().loadSkill("JointPtp", {sLeftQueue8480});

	std::dynamic_pointer_cast<kukadu::JointPtp>(skill848)->setJoints({0, -0.2, 0, -0.7, -1.2, -0.2, 0});

	skill848->execute();

	auto sLeftQueue8490 = getUsedHardware()[3];
	sLeftQueue8490->install();
	sLeftQueue8490->start();

	auto skill849 = kukadu::SkillFactory::get().loadSkill("WaitForReached", {sLeftQueue8490});

	std::dynamic_pointer_cast<kukadu::WaitForReached>(skill849)->setWait(true);

	skill849->execute();

	auto sLeftQueue8500 = getUsedHardware()[3];
	sLeftQueue8500->install();
	sLeftQueue8500->start();

	auto skill850 = kukadu::SkillFactory::get().loadSkill("JointPtp", {sLeftQueue8500});

	std::dynamic_pointer_cast<kukadu::JointPtp>(skill850)->setJoints({0, -0.2, 0, -0.7, -1.2, -0.33, 1.29});

	skill850->execute();

	auto sLeftQueue8510 = getUsedHardware()[3];
	sLeftQueue8510->install();
	sLeftQueue8510->start();

	auto skill851 = kukadu::SkillFactory::get().loadSkill("JointPtp", {sLeftQueue8510});

	std::dynamic_pointer_cast<kukadu::JointPtp>(skill851)->setJoints({0, -0.93, 0.82, -0.7, -1.2, -0.33, 1.29});

	skill851->execute();

	auto sLeftQueue8520 = getUsedHardware()[3];
	sLeftQueue8520->install();
	sLeftQueue8520->start();

	auto skill852 = kukadu::SkillFactory::get().loadSkill("JointPtp", {sLeftQueue8520});

	std::dynamic_pointer_cast<kukadu::JointPtp>(skill852)->setJoints({0, -0.83, 0, -0.7, -1.2, -0.33, 1.29});

	skill852->execute();

	auto sLeftQueue8530 = getUsedHardware()[3];
	sLeftQueue8530->install();
	sLeftQueue8530->start();

	auto skill853 = kukadu::SkillFactory::get().loadSkill("JointPtp", {sLeftQueue8530});

	std::dynamic_pointer_cast<kukadu::JointPtp>(skill853)->setJoints({0, 0, 0, -0.7, -1.2, -0.33, 1.29});

	skill853->execute();

	auto sLeftQueue8540 = getUsedHardware()[3];
	sLeftQueue8540->install();
	sLeftQueue8540->start();

	auto skill854 = kukadu::SkillFactory::get().loadSkill("JointPtp", {sLeftQueue8540});

	std::dynamic_pointer_cast<kukadu::JointPtp>(skill854)->setJoints({0, 0.3, 1.6, -0.7, -1.2, -0.33, 1.29});

	skill854->execute();

	auto sLeftQueue8550 = getUsedHardware()[3];
	sLeftQueue8550->install();
	sLeftQueue8550->start();

	auto skill855 = kukadu::SkillFactory::get().loadSkill("JointPtp", {sLeftQueue8550});

	std::dynamic_pointer_cast<kukadu::JointPtp>(skill855)->setJoints({0, 0.3, 1.6, -0.7, -1.2, -0.93, 0.82});

	skill855->execute();

	auto sLeftQueue8560 = getUsedHardware()[3];
	sLeftQueue8560->install();
	sLeftQueue8560->start();

	auto skill856 = kukadu::SkillFactory::get().loadSkill("JointPtp", {sLeftQueue8560});

	std::dynamic_pointer_cast<kukadu::JointPtp>(skill856)->setJoints({0, 0.3, 1.6, -0.7, -1.2, -0.83, 0});

	skill856->execute();

	auto sLeftQueue8570 = getUsedHardware()[3];
	sLeftQueue8570->install();
	sLeftQueue8570->start();

	auto skill857 = kukadu::SkillFactory::get().loadSkill("JointPtp", {sLeftQueue8570});

	std::dynamic_pointer_cast<kukadu::JointPtp>(skill857)->setJoints({0, 0.3, 1.6, -0.7, -1.2, 0, 0});

	skill857->execute();

	auto sLeftQueue8580 = getUsedHardware()[3];
	sLeftQueue8580->install();
	sLeftQueue8580->start();

	auto skill858 = kukadu::SkillFactory::get().loadSkill("JointPtp", {sLeftQueue8580});

	std::dynamic_pointer_cast<kukadu::JointPtp>(skill858)->setJoints({0, 0.3, 1.6, -0.7, -1.2, 0, 0});

	skill858->execute();

	auto sLeftQueue8590 = getUsedHardware()[3];
	sLeftQueue8590->install();
	sLeftQueue8590->start();

	auto skill859 = kukadu::SkillFactory::get().loadSkill("WaitForReached", {sLeftQueue8590});

	std::dynamic_pointer_cast<kukadu::WaitForReached>(skill859)->setWait(true);

	skill859->execute();

	auto sLeftQueue8600 = getUsedHardware()[3];
	sLeftQueue8600->install();
	sLeftQueue8600->start();

	auto skill860 = kukadu::SkillFactory::get().loadSkill("JointPtp", {sLeftQueue8600});

	std::dynamic_pointer_cast<kukadu::JointPtp>(skill860)->setJoints({0, 0.3, 1.6, -0.7, -1.2, 0.3, 1.6});

	skill860->execute();

	auto sLeftQueue8610 = getUsedHardware()[3];
	sLeftQueue8610->install();
	sLeftQueue8610->start();

	auto skill861 = kukadu::SkillFactory::get().loadSkill("JointPtp", {sLeftQueue8610});

	std::dynamic_pointer_cast<kukadu::JointPtp>(skill861)->setJoints({0, 0.3, 1.6, -0.2, 0, 0.3, 1.6});

	skill861->execute();

	auto sLeftQueue8620 = getUsedHardware()[0];
	sLeftQueue8620->install();
	sLeftQueue8620->start();

	auto skill862 = kukadu::SkillFactory::get().loadSkill("SimpleJointPtp", {sLeftQueue8620});

	std::dynamic_pointer_cast<kukadu::SimpleJointPtp>(skill862)->setJoints({-0.92, 1.91, 1.97, -0.95, -0.11, 1.2, 1.17});

	skill862->execute();

	auto sLeftQueue8630 = getUsedHardware()[3];
	sLeftQueue8630->install();
	sLeftQueue8630->start();

	auto skill863 = kukadu::SkillFactory::get().loadSkill("WaitForReached", {sLeftQueue8630});

	std::dynamic_pointer_cast<kukadu::WaitForReached>(skill863)->setWait(true);

	skill863->execute();

	}
}
