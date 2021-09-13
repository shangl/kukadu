#include <kukadu/generated_skills/PushForward.hpp>
#include <kukadu/manipulation/skillfactory.hpp>

namespace kukadu {
        PushForward::PushForward(kukadu::StorageSingleton &storage,
                                 std::vector<KUKADU_SHARED_PTR<kukadu::Hardware> > hardware)
                : Controller(storage, "PushForward", hardware, 0.01) {
            this->goBackToBlockingPos = false;
        }

        bool PushForward::requiresGraspInternal() {
            return false;
        }

        bool PushForward::producesGraspInternal() {
            return false;
        }

        void PushForward::setGoBackToBlockingPos(bool goBackToBlockingPos) {
            this->goBackToBlockingPos = goBackToBlockingPos;
        }

        std::shared_ptr<kukadu::ControllerResult> PushForward::executeInternal() {
            auto &hardwareFactory = kukadu::HardwareFactory::get();
            auto sLeftQueue2010 = getUsedHardware()[0];
            sLeftQueue2010->install();
            sLeftQueue2010->start();

            auto skill201 = kukadu::SkillFactory::get().loadSkill("RightHandBlocking", {sLeftQueue2010});

            skill201->execute();

            auto sLeftQueue2020 = getUsedHardware()[1];
            sLeftQueue2020->install();
            sLeftQueue2020->start();

            auto skill202 = kukadu::SkillFactory::get().loadSkill("BlockingPos", {sLeftQueue2020});

            skill202->execute();

            auto sLeftQueue2030 = getUsedHardware()[1];
            sLeftQueue2030->install();
            sLeftQueue2030->start();

            auto skill203 = kukadu::SkillFactory::get().loadSkill("push_forward_dmp", {sLeftQueue2030});

            std::dynamic_pointer_cast<kukadu::DMPExecutor>(skill203)->setExecutionMode(
                    kukadu::TrajectoryExecutor::EXECUTE_ROBOT);
            std::dynamic_pointer_cast<kukadu::DMPExecutor>(skill203)->setDoRollBackOnMaxForceEvent(false);
            std::dynamic_pointer_cast<kukadu::DMPExecutor>(skill203)->setExecutionDuration(3.5);
            std::dynamic_pointer_cast<kukadu::DMPExecutor>(skill203)->setMaxAbsForce(15);

            skill203->execute();

            if (goBackToBlockingPos) {
                auto sLeftQueue2040 = getUsedHardware()[1];
                sLeftQueue2040->install();
                sLeftQueue2040->start();

                auto skill13 = kukadu::SkillFactory::get().loadSkill("BlockingPos", {sLeftQueue2040});
                skill13->execute();
            }

            return nullptr;
        }

        std::string PushForward::getClassName() {
            return "PushForward";
        }

        void PushForward::createSkillFromThisInternal(std::string skillName) {
            // nothing to do
        }
}
