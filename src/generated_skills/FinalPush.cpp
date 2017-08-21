#include <kukadu/generated_skills/FinalPush.hpp>

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

        auto sLeftQueue21400 = getUsedHardware()[0];
        sLeftQueue21400->install();
        sLeftQueue21400->start();

        auto skill2140 = kukadu::SkillFactory::get().loadSkill("JointPtp", {sLeftQueue21400});

        std::dynamic_pointer_cast<kukadu::JointPtp>(skill2140)->setJoints(
                {-0.109378, 1.55693, 1.50002, -0.224965, -0.553483, -0.614582, -0.658589});

        skill2140->execute();

        auto sLeftQueue21410 = getUsedHardware()[0];
        sLeftQueue21410->install();
        sLeftQueue21410->start();

        auto skill2141 = kukadu::SkillFactory::get().loadSkill("JointPtp", {sLeftQueue21410});

        std::dynamic_pointer_cast<kukadu::JointPtp>(skill2141)->setJoints(
                {-0.1002, 1.54825, 1.51948, -0.22662, -0.543338, -0.614317, -0.658062});

        skill2141->execute();

        auto sLeftQueue21420 = getUsedHardware()[0];
        sLeftQueue21420->install();
        sLeftQueue21420->start();

        auto skill2142 = kukadu::SkillFactory::get().loadSkill("JointPtp", {sLeftQueue21420});

        std::dynamic_pointer_cast<kukadu::JointPtp>(skill2142)->setJoints(
                {-0.0793886, 1.54343, 1.5273, -0.222725, -0.54333, -0.607497, -0.669542});

        skill2142->execute();

        auto sLeftQueue21430 = getUsedHardware()[0];
        sLeftQueue21430->install();
        sLeftQueue21430->start();

        auto skill2143 = kukadu::SkillFactory::get().loadSkill("JointPtp", {sLeftQueue21430});

        std::dynamic_pointer_cast<kukadu::JointPtp>(skill2143)->setJoints(
                {-0.0529747, 1.53886, 1.52831, -0.217783, -0.524916, -0.607438, -0.692989});

        skill2143->execute();

        auto sLeftQueue21440 = getUsedHardware()[0];
        sLeftQueue21440->install();
        sLeftQueue21440->start();

        auto skill2144 = kukadu::SkillFactory::get().loadSkill("JointPtp", {sLeftQueue21440});

        std::dynamic_pointer_cast<kukadu::JointPtp>(skill2144)->setJoints(
                {-0.0422586, 1.53685, 1.52831, -0.21765, -0.515485, -0.606345, -0.705268});

        skill2144->execute();

        auto sLeftQueue21450 = getUsedHardware()[0];
        sLeftQueue21450->install();
        sLeftQueue21450->start();

        auto skill2145 = kukadu::SkillFactory::get().loadSkill("JointPtp", {sLeftQueue21450});

        std::dynamic_pointer_cast<kukadu::JointPtp>(skill2145)->setJoints(
                {-0.0237716, 1.53687, 1.52831, -0.21757, -0.515368, -0.604097, -0.705288});

        skill2145->execute();

        auto sLeftQueue21460 = getUsedHardware()[0];
        sLeftQueue21460->install();
        sLeftQueue21460->start();

        auto skill2146 = kukadu::SkillFactory::get().loadSkill("JointPtp", {sLeftQueue21460});

        std::dynamic_pointer_cast<kukadu::JointPtp>(skill2146)->setJoints(
                {-0.0156862, 1.53686, 1.52831, -0.217573, -0.515372, -0.59964, -0.708731});

        skill2146->execute();

        auto sLeftQueue21470 = getUsedHardware()[0];
        sLeftQueue21470->install();
        sLeftQueue21470->start();

        auto skill2147 = kukadu::SkillFactory::get().loadSkill("JointPtp", {sLeftQueue21470});

        std::dynamic_pointer_cast<kukadu::JointPtp>(skill2147)->setJoints(
                {-0.0156172, 1.53727, 1.52831, -0.219744, -0.511537, -0.566932, -0.708706});

        skill2147->execute();

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
        std::dynamic_pointer_cast<kukadu::DMPExecutor>(skill2149)->setMaxXForce(14);
        std::dynamic_pointer_cast<kukadu::DMPExecutor>(skill2149)->setMaxYForce(14);
        std::dynamic_pointer_cast<kukadu::DMPExecutor>(skill2149)->setMaxZForce(14);

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
