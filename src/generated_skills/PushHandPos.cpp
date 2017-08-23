#include <kukadu/generated_skills/PushHandPos.hpp>
#include <kukadu/manipulation/skillfactory.hpp>

namespace kukadu {
        PushHandPos::PushHandPos(kukadu::StorageSingleton &storage,
                                 std::vector<KUKADU_SHARED_PTR<kukadu::Hardware> > hardware)
                : Controller(storage, "PushHandPos", hardware, 0.01) {

        }

        bool PushHandPos::requiresGraspInternal() {
            return false;
        }

        bool PushHandPos::producesGraspInternal() {
            return false;
        }

        std::shared_ptr<kukadu::ControllerResult> PushHandPos::executeInternal() {
            auto &hardwareFactory = kukadu::HardwareFactory::get();
            auto sLeftQueue1160 = getUsedHardware()[0];
            sLeftQueue1160->install();
            sLeftQueue1160->start();

            auto skill116 = kukadu::SkillFactory::get().loadSkill("JointPtp", {sLeftQueue1160});

            std::dynamic_pointer_cast<kukadu::JointPtp>(skill116)->setJoints({0, -1.57, 0, -1.57, 0, -1.57, 0});

            skill116->execute();

            return nullptr;
        }

        std::string PushHandPos::getClassName() {
            return "PushHandPos";
        }

        void PushHandPos::createSkillFromThisInternal(std::string skillName) {
            // nothing to do
        }
}
