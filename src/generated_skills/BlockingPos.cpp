#include <kukadu/generated_skills/BlockingPos.hpp>
#include <kukadu/generated_skills/SimpleJointPtp.hpp>
#include <kukadu/manipulation/skillfactory.hpp>

namespace kukadu {
        BlockingPos::BlockingPos(kukadu::StorageSingleton &storage,
                                 std::vector<KUKADU_SHARED_PTR<kukadu::Hardware> > hardware)
                : Controller(storage, "BlockingPos", hardware, 0.01) {

        }

        bool BlockingPos::requiresGraspInternal() {
            return false;
        }

        bool BlockingPos::producesGraspInternal() {
            return false;
        }

        std::shared_ptr<kukadu::ControllerResult> BlockingPos::executeInternal() {
            auto &hardwareFactory = kukadu::HardwareFactory::get();
            auto sLeftQueue1150 = getUsedHardware()[0];
            sLeftQueue1150->install();
            sLeftQueue1150->start();

            auto skill115 = kukadu::SkillFactory::get().loadSkill("SimpleJointPtp", {sLeftQueue1150});

            std::dynamic_pointer_cast<kukadu::SimpleJointPtp>(skill115)->setJoints(
                    {-2.4132, 1.62996, -2.22251, 2.01567, 2.18936, -1.65823, -0.956807});

            skill115->execute();

            return nullptr;
        }

        std::string BlockingPos::getClassName() {
            return "BlockingPos";
        }

        void BlockingPos::createSkillFromThisInternal(std::string skillName) {
            // nothing to do
        }
}
