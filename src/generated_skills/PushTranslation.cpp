#include <kukadu/generated_skills/PushTranslation.hpp>
#include <kukadu/manipulation/skillfactory.hpp>

namespace kukadu {
	PushTranslation::PushTranslation(kukadu::StorageSingleton& storage, std::vector< KUKADU_SHARED_PTR< kukadu::Hardware > > hardware)
 : Controller(storage, "PushTranslation", hardware, 0.01) {

	}

	bool PushTranslation::requiresGraspInternal() {
		return false;
	}

	bool PushTranslation::producesGraspInternal() {
		return false;
	}

	std::shared_ptr<kukadu::ControllerResult> PushTranslation::executeInternal() {
		int pushForward = 0;
	if (pushForward) {
		auto sLeftQueue720 = getUsedHardware()[0];
		sLeftQueue720->install();
		sLeftQueue720->start();
		auto sLeftQueue721 = getUsedHardware()[1];
		sLeftQueue721->install();
		sLeftQueue721->start();

		auto skill72 = kukadu::SkillFactory::get().loadSkill("PushForward", {sLeftQueue720, sLeftQueue721});

		skill72->execute();

	}
	auto sLeftQueue730 = getUsedHardware()[2];
	sLeftQueue730->install();
	sLeftQueue730->start();

	auto skill73 = kukadu::SkillFactory::get().loadSkill("MoveHome", {sLeftQueue730});

	skill73->execute();

	auto sLeftQueue740 = getUsedHardware()[3];
	sLeftQueue740->install();
	sLeftQueue740->start();

	auto skill74 = kukadu::SkillFactory::get().loadSkill("PushHandPos", {sLeftQueue740});

	skill74->execute();

	auto sLeftQueue750 = getUsedHardware()[4];
	sLeftQueue750->install();
	sLeftQueue750->start();

	auto skill75 = kukadu::SkillFactory::get().loadSkill("localize_object", {sLeftQueue750});

	std::dynamic_pointer_cast<kukadu::LocalizeObject>(skill75)->setObjectToLoad("something");

	skill75->execute();

	auto pos = kukadu::PoseEstimatorFactory::get().getPoseFor("something").pose;
	pos.position.z = -0.03;
	auto startPoseWithOffset = kukadu::PoseEstimatorFactory::get().getPoseFor("something").pose;
	auto startPoseWoHeightOffset = kukadu::PoseEstimatorFactory::get().getPoseFor("something").pose;
	auto startPose = kukadu::PoseEstimatorFactory::get().getPoseFor("something").pose;
	auto dim = kukadu::PoseEstimatorFactory::get().getDimensionsFor("something");
	double length = (dim[0] + 0.02);
	double width = (dim[1] + 0.02);
	double height = dim[2];
	double targetx = 0.21;
	double targety = 0.675;
	int finishedPushing = 0;
	double horizontalDistance = 0;
	double verticalDistance = 0;
	int bookVertical = dim[0] > dim[1];
	int booleanHelper = 0;
	double distance = 0;
	double jumpingSteps = 0.015;
	double i = 0;
	double bookoffset[3] = {0};
	while (finishedPushing != 1) {
		horizontalDistance = (pos.position.y - targety);
		verticalDistance = (pos.position.x - targetx);
        booleanHelper = (std::sqrt(std::pow(horizontalDistance,2))) < 0.02;
		if (booleanHelper) {
			finishedPushing = 1;
		}
		pos.position.y = (pos.position.y + 0.07);
		horizontalDistance = (pos.position.y - targety);
		booleanHelper = horizontalDistance > 0.02;
		if (booleanHelper) {
			horizontalDistance = (horizontalDistance - 0.03);
		}
		booleanHelper = finishedPushing != 1;
		if (booleanHelper) {
			startPoseWithOffset.orientation.w = 0.5;
			startPoseWithOffset.orientation.x = 0.5;
			startPoseWithOffset.orientation.y = 0.5;
			startPoseWithOffset.orientation.z = -0.5;
			bookoffset[0] = 0;
			bookoffset[2] = 0.519;
			if (bookVertical) {
				bookoffset[1] = ((width / 2) + (0.12 + 0.03));
			} else {
				bookoffset[1] = ((length / 2) + (0.12 + 0.03));
			}
			startPoseWithOffset.position.x = (pos.position.x + bookoffset[0]);
			startPoseWithOffset.position.y = (pos.position.y + bookoffset[1]);
			startPoseWithOffset.position.z = (pos.position.z + bookoffset[2]);
			startPoseWoHeightOffset.orientation.w = 0.5;
			startPoseWoHeightOffset.orientation.x = 0.5;
			startPoseWoHeightOffset.orientation.y = 0.5;
			startPoseWoHeightOffset.orientation.z = -0.5;
			bookoffset[0] = 0;
			bookoffset[2] = 0.3;
			if (bookVertical) {
				bookoffset[1] = ((width / 2) + (0.12 + 0.03));
			} else {
				bookoffset[1] = ((length / 2) + (0.12 + 0.03));
			}
			startPoseWoHeightOffset.position.x = (pos.position.x + bookoffset[0]);
			startPoseWoHeightOffset.position.y = (pos.position.y + bookoffset[1]);
			startPoseWoHeightOffset.position.z = (pos.position.z + bookoffset[2]);
			startPose.orientation.w = 0.5;
			startPose.orientation.x = 0.5;
			startPose.orientation.y = 0.5;
			startPose.orientation.z = -0.5;
			bookoffset[0] = 0;
			bookoffset[2] = 0.3;
			if (bookVertical) {
				bookoffset[1] = ((width / 2) + (0.12 + 0.03));
			} else {
				bookoffset[1] = ((length / 2) + (0.12 + 0));
			}
			startPose.position.x = (pos.position.x + bookoffset[0]);
			startPose.position.y = (pos.position.y + bookoffset[1]);
			startPose.position.z = (pos.position.z + bookoffset[2]);
			auto sLeftQueue760 = getUsedHardware()[2];
			sLeftQueue760->install();
			sLeftQueue760->start();

            std::cout << startPoseWithOffset.position.x << " " << startPoseWithOffset.position.y << " " << startPoseWithOffset.position.z << std::endl;

			auto skill76 = kukadu::SkillFactory::get().loadSkill("CartesianPtp", {sLeftQueue760});

			std::dynamic_pointer_cast<kukadu::CartesianPtp>(skill76)->setCartesians(startPoseWithOffset);

			skill76->execute();

			auto sLeftQueue770 = getUsedHardware()[2];
			sLeftQueue770->install();
			sLeftQueue770->start();

			auto skill77 = kukadu::SkillFactory::get().loadSkill("CartesianPtp", {sLeftQueue770});

			std::dynamic_pointer_cast<kukadu::CartesianPtp>(skill77)->setCartesians(startPoseWoHeightOffset);

			skill77->execute();

            distance = (std::sqrt(std::pow(horizontalDistance,2)));
			while (i < distance) {
				auto sLeftQueue780 = getUsedHardware()[2];
				sLeftQueue780->install();
				sLeftQueue780->start();

				auto skill78 = kukadu::SkillFactory::get().loadSkill("CartesianPtp", {sLeftQueue780});

				std::dynamic_pointer_cast<kukadu::CartesianPtp>(skill78)->setCartesians(startPose);

				skill78->execute();

				startPose.position.y = (startPose.position.y - jumpingSteps);
                booleanHelper = jumpingSteps < (std::sqrt(std::pow((distance - i),2)));
				if (booleanHelper) {
					i = (i + jumpingSteps);
				} else {
                    i = (i + (std::sqrt(std::pow((distance - i),2))));
				}
			}
			//keep this CartesianPtp for last Push
			auto sLeftQueue790 = getUsedHardware()[2];
			sLeftQueue790->install();
			sLeftQueue790->start();

			auto skill79 = kukadu::SkillFactory::get().loadSkill("CartesianPtp", {sLeftQueue790});

			std::dynamic_pointer_cast<kukadu::CartesianPtp>(skill79)->setCartesians(startPose);

			skill79->execute();

			pos.position.y = targety;
			startPose.position.z = (startPose.position.z + 0.1);
			auto sLeftQueue800 = getUsedHardware()[2];
			sLeftQueue800->install();
			sLeftQueue800->start();

			auto skill80 = kukadu::SkillFactory::get().loadSkill("CartesianPtp", {sLeftQueue800});

			std::dynamic_pointer_cast<kukadu::CartesianPtp>(skill80)->setCartesians(startPose);

			skill80->execute();

		}
	}

		return nullptr;
	}


	std::string PushTranslation::getClassName() {
		return "PushTranslation";
	}

	void PushTranslation::createSkillFromThisInternal(std::string skillName) {
		// nothing to do
	}
}
