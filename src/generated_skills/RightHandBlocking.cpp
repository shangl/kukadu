#include <kukadu/generated_skills/RightHandBlocking.hpp>
#include <kukadu/manipulation/skillfactory.hpp>

namespace kukadu {
        RightHandBlocking::RightHandBlocking(kukadu::StorageSingleton &storage,
                                             std::vector<KUKADU_SHARED_PTR<kukadu::Hardware> > hardware)
                : Controller(storage, "RightHandBlocking", hardware, 0.01) {

        }

        bool RightHandBlocking::requiresGraspInternal() {
            return false;
        }

        bool RightHandBlocking::producesGraspInternal() {
            return false;
        }

        std::shared_ptr<kukadu::ControllerResult> RightHandBlocking::executeInternal() {
            auto &hardwareFactory = kukadu::HardwareFactory::get();
            auto sLeftQueue650 = getUsedHardware()[0];
            sLeftQueue650->install();
            sLeftQueue650->start();

            auto skill65 = kukadu::SkillFactory::get().loadSkill("JointPtp", {sLeftQueue650});

            std::dynamic_pointer_cast<kukadu::JointPtp>(skill65)->setJoints({0, 0.13, -0.65, 1.06, 0.46, 0.18, -0.88});

            skill65->execute();

            return nullptr;
        }

        std::string RightHandBlocking::getClassName() {
            return "RightHandBlocking";
        }

        void RightHandBlocking::createSkillFromThisInternal(std::string skillName) {
            // nothing to do
        }
}
