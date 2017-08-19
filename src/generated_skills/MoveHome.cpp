#include <kukadu/generated_skills/MoveHome.hpp>

namespace kukadu {
        MoveHome::MoveHome(kukadu::StorageSingleton &storage,
                           std::vector<KUKADU_SHARED_PTR<kukadu::Hardware> > hardware)
                : Controller(storage, "MoveHome", hardware, 0.01) {

        }

        bool MoveHome::requiresGraspInternal() {
            return false;
        }

        bool MoveHome::producesGraspInternal() {
            return false;
        }

        std::shared_ptr<kukadu::ControllerResult> MoveHome::executeInternal() {
            auto &hardwareFactory = kukadu::HardwareFactory::get();
            auto sLeftQueue650 = getUsedHardware()[0];
            sLeftQueue650->install();
            sLeftQueue650->start();

            auto skill65 = kukadu::SkillFactory::get().loadSkill("JointPtp", {sLeftQueue650});

            std::dynamic_pointer_cast<kukadu::JointPtp>(skill65)->setJoints(
                    {-1.5, 1.56, 2.33, -1.74, -1.85, 1.27, 0.71});

            skill65->execute();

            return nullptr;
        }

        std::string MoveHome::getClassName() {
            return "MoveHome";
        }

        void MoveHome::createSkillFromThisInternal(std::string skillName) {
            // nothing to do
        }
}
