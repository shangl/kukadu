#include <kukadu/generated_skills/FinalPush.hpp>
#include <kukadu/generated_skills/ChangeStiffness.hpp>
#include <kukadu/manipulation/skillfactory.hpp>

namespace kukadu {
    FinalPush::FinalPush(kukadu::StorageSingleton &storage, std::vector<KUKADU_SHARED_PTR<kukadu::Hardware> > hardware)
            : Controller(storage, "FinalPush", hardware, 0.01) {

    }

    bool FinalPush::requiresGraspInternal() {
        return false;
    }

    bool FinalPush::producesGraspInternal() {
        return false;
    }

    std::shared_ptr<kukadu::ControllerResult> FinalPush::executeInternal() {
        auto sLeftQueue21360 = getUsedHardware()[0];
        sLeftQueue21360->install();
        sLeftQueue21360->start();

        auto skill2136 = kukadu::SkillFactory::get().loadSkill("ChangeModeToIMP", {sLeftQueue21360});

        skill2136->execute();

        auto sLeftQueue21370 = getUsedHardware()[0];
        sLeftQueue21370->install();
        sLeftQueue21370->start();

        auto skill2137 = kukadu::SkillFactory::get().loadSkill("ChangeStiffness", {sLeftQueue21370});

        std::dynamic_pointer_cast<kukadu::ChangeStiffness>(skill2137)->setStiffnessType(0);

        skill2137->execute();

        auto sLeftQueue21380 = getUsedHardware()[1];
        sLeftQueue21380->install();
        sLeftQueue21380->start();

        auto skill2138 = kukadu::SkillFactory::get().loadSkill("PushHandPos", {sLeftQueue21380});

        skill2138->execute();

        auto sLeftQueue21390 = getUsedHardware()[0];
        sLeftQueue21390->install();
        sLeftQueue21390->start();

        auto skill2139 = kukadu::SkillFactory::get().loadSkill("JointPtp", {sLeftQueue21390});

        std::dynamic_pointer_cast<kukadu::JointPtp>(skill2139)->setJoints(
                {-0.9692263007164001, 1.113829493522644, 1.1473214626312256, -1.444376826286316, -0.28663957118988037,
                 -0.8957559466362, -0.2651996612548828});

        skill2139->execute();

        auto sLeftQueue21480 = getUsedHardware()[0];
        sLeftQueue21480->install();
        sLeftQueue21480->start();

        auto skill2148 = kukadu::SkillFactory::get().loadSkill("JointPtp", {sLeftQueue21480});

        std::dynamic_pointer_cast<kukadu::JointPtp>(skill2148)->setJoints(
                {-0.0390551, 1.53972, 1.52831, -0.262562, -0.576725, -0.567193, -0.683141});

        skill2148->execute();

        auto sLeftQueue21490 = getUsedHardware()[0];
        sLeftQueue21490->install();
        sLeftQueue21490->start();

        auto skill2149 = kukadu::SkillFactory::get().loadSkill("final_push_dmp", {sLeftQueue21490});

        std::dynamic_pointer_cast<kukadu::DMPExecutor>(skill2149)->setExecutionMode(
                kukadu::TrajectoryExecutor::EXECUTE_ROBOT);
        std::dynamic_pointer_cast<kukadu::DMPExecutor>(skill2149)->setDoRollBackOnMaxForceEvent(false);
        std::dynamic_pointer_cast<kukadu::DMPExecutor>(skill2149)->setMaxXForce(19);
        std::dynamic_pointer_cast<kukadu::DMPExecutor>(skill2149)->setMaxYForce(19);
        std::dynamic_pointer_cast<kukadu::DMPExecutor>(skill2149)->setMaxZForce(19);

        skill2149->execute();

        auto sLeftQueue21500 = getUsedHardware()[0];
        sLeftQueue21500->install();
        sLeftQueue21500->start();

        auto skill2150 = kukadu::SkillFactory::get().loadSkill("JointPtp", {sLeftQueue21500});

        std::dynamic_pointer_cast<kukadu::JointPtp>(skill2150)->setJoints(
                {-0.9692263007164001, 1.113829493522644, 1.1473214626312256, -1.444376826286316, -0.28663957118988037,
                 -0.8957559466362, -0.2651996612548828});

        skill2150->execute();

        auto sLeftQueue21510 = getUsedHardware()[0];
        sLeftQueue21510->install();
        sLeftQueue21510->start();

        auto skill2151 = kukadu::SkillFactory::get().loadSkill("MoveHome", {sLeftQueue21510});

        skill2151->execute();

        return nullptr;
    }

    std::string FinalPush::getClassName() {
        return "FinalPush";
    }

    void FinalPush::createSkillFromThisInternal(std::string skillName) {
        // nothing to do
    }
}
