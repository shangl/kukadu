#include <kukadu/generated_skills/BookGrasping.hpp>
#include <kukadu/generated_skills/BlockingPos.hpp>
#include <kukadu/generated_skills/ChangeModeToIMP.hpp>
#include <kukadu/generated_skills/ChangeStiffness.hpp>
#include <kukadu/generated_skills/CloseHand.hpp>
#include <kukadu/generated_skills/DropInBoxController.hpp>
#include <kukadu/generated_skills/FinalPush.hpp>
#include <kukadu/generated_skills/KinestheticTeaching.hpp>
#include <kukadu/generated_skills/MoveHome.hpp>
#include <kukadu/generated_skills/OpenHand.hpp>
#include <kukadu/generated_skills/PressButtonController.hpp>
#include <kukadu/generated_skills/PushForward.hpp>
#include <kukadu/generated_skills/PushHandPos.hpp>
#include <kukadu/generated_skills/PushTranslation.hpp>
#include <kukadu/generated_skills/RightHandBlocking.hpp>
#include <kukadu/generated_skills/SensingPoke.hpp>
#include <kukadu/generated_skills/SensingSlide.hpp>
#include <kukadu/generated_skills/ShelfAlignment.hpp>
#include <kukadu/generated_skills/ShelfPlacementController.hpp>
#include <kukadu/generated_skills/SimpleJointPtp.hpp>
#include <kukadu/generated_skills/WaitForReached.hpp>

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
        return 4;
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
		auto sLeftQueue2240 = getUsedHardware()[0];
	sLeftQueue2240->install();
	sLeftQueue2240->start();

	auto skill224 = kukadu::SkillFactory::get().loadSkill("MoveHome", {sLeftQueue2240});

	skill224->execute();

	auto sLeftQueue2250 = getUsedHardware()[1];
	sLeftQueue2250->install();
	sLeftQueue2250->start();

	auto skill225 = kukadu::SkillFactory::get().loadSkill("ChangeModeToIMP", {sLeftQueue2250});

	skill225->execute();

	auto sLeftQueue2260 = getUsedHardware()[1];
	sLeftQueue2260->install();
	sLeftQueue2260->start();

	auto skill226 = kukadu::SkillFactory::get().loadSkill("ChangeStiffness", {sLeftQueue2260});

	std::dynamic_pointer_cast<kukadu::ChangeStiffness>(skill226)->setStiffnessType(0);
	std::dynamic_pointer_cast<kukadu::ChangeStiffness>(skill226)->setStandardStiffnessDamping(-1);

	skill226->execute();

	auto sLeftQueue2270 = getUsedHardware()[0];
	sLeftQueue2270->install();
	sLeftQueue2270->start();

	auto skill227 = kukadu::SkillFactory::get().loadSkill("ChangeModeToIMP", {sLeftQueue2270});

	skill227->execute();

	auto sLeftQueue2280 = getUsedHardware()[2];
	sLeftQueue2280->install();
	sLeftQueue2280->start();
	auto sLeftQueue2281 = getUsedHardware()[1];
	sLeftQueue2281->install();
	sLeftQueue2281->start();

	auto skill228 = kukadu::SkillFactory::get().loadSkill("PushForward", {sLeftQueue2280, sLeftQueue2281});

	skill228->execute();

	auto sLeftQueue2290 = getUsedHardware()[1];
	sLeftQueue2290->install();
	sLeftQueue2290->start();

	auto skill229 = kukadu::SkillFactory::get().loadSkill("BlockingPos", {sLeftQueue2290});

	skill229->execute();

    auto sLeftQueue2300 = getUsedHardware()[0];
    sLeftQueue2300->install();
    sLeftQueue2300->start();
    auto sLeftQueue2301 = getUsedHardware()[1];
    sLeftQueue2301->install();
    sLeftQueue2301->start();
    auto sLeftQueue2302 = getUsedHardware()[2];
    sLeftQueue2302->install();
    sLeftQueue2302->start();
    auto sLeftQueue2303 = getUsedHardware()[3];
    sLeftQueue2303->install();
    sLeftQueue2303->start();
    auto sLeftQueue2304 = getUsedHardware()[4];
    sLeftQueue2304->install();
    sLeftQueue2304->start();

    auto skill230 = kukadu::SkillFactory::get().loadSkill("PushTranslation", {sLeftQueue2300, sLeftQueue2301, sLeftQueue2302, sLeftQueue2303, sLeftQueue2304});

	skill230->execute();

	auto sLeftQueue2310 = getUsedHardware()[0];
	sLeftQueue2310->install();
	sLeftQueue2310->start();
	auto sLeftQueue2311 = getUsedHardware()[3];
	sLeftQueue2311->install();
	sLeftQueue2311->start();

	auto skill231 = kukadu::SkillFactory::get().loadSkill("FinalPush", {sLeftQueue2310, sLeftQueue2311});

	skill231->execute();

	auto sLeftQueue2320 = getUsedHardware()[0];
	sLeftQueue2320->install();
	sLeftQueue2320->start();

	auto skill232 = kukadu::SkillFactory::get().loadSkill("MoveHome", {sLeftQueue2320});

	skill232->execute();

	auto sLeftQueue2330 = getUsedHardware()[3];
	sLeftQueue2330->install();
	sLeftQueue2330->start();

	auto skill233 = kukadu::SkillFactory::get().loadSkill("JointPtp", {sLeftQueue2330});

	std::dynamic_pointer_cast<kukadu::JointPtp>(skill233)->setJoints({0, -0.2, 0, -0.7, -1.2, -0.2, 0});

	skill233->execute();

	auto sLeftQueue2340 = getUsedHardware()[2];
	sLeftQueue2340->install();
	sLeftQueue2340->start();

	auto skill234 = kukadu::SkillFactory::get().loadSkill("RightHandBlocking", {sLeftQueue2340});

	skill234->execute();

	auto sLeftQueue2350 = getUsedHardware()[0];
	sLeftQueue2350->install();
	sLeftQueue2350->start();

	auto skill235 = kukadu::SkillFactory::get().loadSkill("SimpleJointPtp", {sLeftQueue2350});

	std::dynamic_pointer_cast<kukadu::SimpleJointPtp>(skill235)->setJoints({-0.35, 1.66, 1.96, -0.54, -0.13, 1.4, 1.21});

	skill235->execute();

	auto sLeftQueue2360 = getUsedHardware()[2];
	sLeftQueue2360->install();
	sLeftQueue2360->start();
	auto sLeftQueue2361 = getUsedHardware()[1];
	sLeftQueue2361->install();
	sLeftQueue2361->start();

	auto skill236 = kukadu::SkillFactory::get().loadSkill("PushForward", {sLeftQueue2360, sLeftQueue2361});

	skill236->execute();

	auto sLeftQueue2370 = getUsedHardware()[1];
	sLeftQueue2370->install();
	sLeftQueue2370->start();

	auto skill237 = kukadu::SkillFactory::get().loadSkill("ChangeStiffness", {sLeftQueue2370});

	std::dynamic_pointer_cast<kukadu::ChangeStiffness>(skill237)->setStiffnessType(1);

	skill237->execute();

	auto sLeftQueue2380 = getUsedHardware()[0];
	sLeftQueue2380->install();
	sLeftQueue2380->start();

	auto skill238 = kukadu::SkillFactory::get().loadSkill("ChangeStiffness", {sLeftQueue2380});

	std::dynamic_pointer_cast<kukadu::ChangeStiffness>(skill238)->setStiffnessType(1);

	skill238->execute();

	auto sLeftQueue2390 = getUsedHardware()[0];
	sLeftQueue2390->install();
	sLeftQueue2390->start();

	auto skill239 = kukadu::SkillFactory::get().loadSkill("SimpleJointPtp", {sLeftQueue2390});

	std::dynamic_pointer_cast<kukadu::SimpleJointPtp>(skill239)->setJoints({-0.35, 1.66, 1.96, -0.54, -0.13, 1.4, 1.21});

	skill239->execute();

	auto sLeftQueue2400 = getUsedHardware()[3];
	sLeftQueue2400->install();
	sLeftQueue2400->start();

	auto skill240 = kukadu::SkillFactory::get().loadSkill("JointPtp", {sLeftQueue2400});

	std::dynamic_pointer_cast<kukadu::JointPtp>(skill240)->setJoints({0, -0.2, 0, -0.7, -1.2, -0.2, 0});

	skill240->execute();

	auto sLeftQueue2410 = getUsedHardware()[3];
	sLeftQueue2410->install();
	sLeftQueue2410->start();

	auto skill241 = kukadu::SkillFactory::get().loadSkill("WaitForReached", {sLeftQueue2410});

	std::dynamic_pointer_cast<kukadu::WaitForReached>(skill241)->setWait(true);

	skill241->execute();

	auto sLeftQueue2420 = getUsedHardware()[3];
	sLeftQueue2420->install();
	sLeftQueue2420->start();

	auto skill242 = kukadu::SkillFactory::get().loadSkill("JointPtp", {sLeftQueue2420});

	std::dynamic_pointer_cast<kukadu::JointPtp>(skill242)->setJoints({0, -0.2, 0, -0.7, -1.2, -0.33, 1.29});

	skill242->execute();

	auto sLeftQueue2430 = getUsedHardware()[3];
	sLeftQueue2430->install();
	sLeftQueue2430->start();

	auto skill243 = kukadu::SkillFactory::get().loadSkill("JointPtp", {sLeftQueue2430});

	std::dynamic_pointer_cast<kukadu::JointPtp>(skill243)->setJoints({0, -0.93, 0.82, -0.7, -1.2, -0.33, 1.29});

	skill243->execute();

	auto sLeftQueue2440 = getUsedHardware()[3];
	sLeftQueue2440->install();
	sLeftQueue2440->start();

	auto skill244 = kukadu::SkillFactory::get().loadSkill("JointPtp", {sLeftQueue2440});

	std::dynamic_pointer_cast<kukadu::JointPtp>(skill244)->setJoints({0, -0.83, 0, -0.7, -1.2, -0.33, 1.29});

	skill244->execute();

	auto sLeftQueue2450 = getUsedHardware()[3];
	sLeftQueue2450->install();
	sLeftQueue2450->start();

	auto skill245 = kukadu::SkillFactory::get().loadSkill("JointPtp", {sLeftQueue2450});

	std::dynamic_pointer_cast<kukadu::JointPtp>(skill245)->setJoints({0, 0, 0, -0.7, -1.2, -0.33, 1.29});

	skill245->execute();

	auto sLeftQueue2460 = getUsedHardware()[3];
	sLeftQueue2460->install();
	sLeftQueue2460->start();

	auto skill246 = kukadu::SkillFactory::get().loadSkill("JointPtp", {sLeftQueue2460});

	std::dynamic_pointer_cast<kukadu::JointPtp>(skill246)->setJoints({0, 0.3, 1.6, -0.7, -1.2, -0.33, 1.29});

	skill246->execute();

	auto sLeftQueue2470 = getUsedHardware()[3];
	sLeftQueue2470->install();
	sLeftQueue2470->start();

	auto skill247 = kukadu::SkillFactory::get().loadSkill("JointPtp", {sLeftQueue2470});

	std::dynamic_pointer_cast<kukadu::JointPtp>(skill247)->setJoints({0, 0.3, 1.6, -0.7, -1.2, -0.93, 0.82});

	skill247->execute();

	auto sLeftQueue2480 = getUsedHardware()[3];
	sLeftQueue2480->install();
	sLeftQueue2480->start();

	auto skill248 = kukadu::SkillFactory::get().loadSkill("JointPtp", {sLeftQueue2480});

	std::dynamic_pointer_cast<kukadu::JointPtp>(skill248)->setJoints({0, 0.3, 1.6, -0.7, -1.2, -0.83, 0});

	skill248->execute();

	auto sLeftQueue2490 = getUsedHardware()[3];
	sLeftQueue2490->install();
	sLeftQueue2490->start();

	auto skill249 = kukadu::SkillFactory::get().loadSkill("JointPtp", {sLeftQueue2490});

	std::dynamic_pointer_cast<kukadu::JointPtp>(skill249)->setJoints({0, 0.3, 1.6, -0.7, -1.2, 0, 0});

	skill249->execute();

	auto sLeftQueue2500 = getUsedHardware()[3];
	sLeftQueue2500->install();
	sLeftQueue2500->start();

	auto skill250 = kukadu::SkillFactory::get().loadSkill("JointPtp", {sLeftQueue2500});

	std::dynamic_pointer_cast<kukadu::JointPtp>(skill250)->setJoints({0, 0.3, 1.6, -0.7, -1.2, 0, 0});

	skill250->execute();

	auto sLeftQueue2510 = getUsedHardware()[3];
	sLeftQueue2510->install();
	sLeftQueue2510->start();

	auto skill251 = kukadu::SkillFactory::get().loadSkill("WaitForReached", {sLeftQueue2510});

	std::dynamic_pointer_cast<kukadu::WaitForReached>(skill251)->setWait(true);

	skill251->execute();

	auto sLeftQueue2520 = getUsedHardware()[3];
	sLeftQueue2520->install();
	sLeftQueue2520->start();

	auto skill252 = kukadu::SkillFactory::get().loadSkill("JointPtp", {sLeftQueue2520});

	std::dynamic_pointer_cast<kukadu::JointPtp>(skill252)->setJoints({0, 0.3, 1.6, -0.7, -1.2, 0.3, 1.6});

	skill252->execute();

	auto sLeftQueue2530 = getUsedHardware()[3];
	sLeftQueue2530->install();
	sLeftQueue2530->start();

	auto skill253 = kukadu::SkillFactory::get().loadSkill("JointPtp", {sLeftQueue2530});

	std::dynamic_pointer_cast<kukadu::JointPtp>(skill253)->setJoints({0, 0.3, 1.6, -0.2, 0, 0.3, 1.6});

	skill253->execute();

	auto sLeftQueue2540 = getUsedHardware()[0];
	sLeftQueue2540->install();
	sLeftQueue2540->start();

	auto skill254 = kukadu::SkillFactory::get().loadSkill("SimpleJointPtp", {sLeftQueue2540});

	std::dynamic_pointer_cast<kukadu::SimpleJointPtp>(skill254)->setJoints({-0.92, 1.91, 1.97, -0.95, -0.11, 1.2, 1.17});

	skill254->execute();

	auto sLeftQueue2550 = getUsedHardware()[3];
	sLeftQueue2550->install();
	sLeftQueue2550->start();

	auto skill255 = kukadu::SkillFactory::get().loadSkill("WaitForReached", {sLeftQueue2550});

	std::dynamic_pointer_cast<kukadu::WaitForReached>(skill255)->setWait(true);

	skill255->execute();

	}
}
