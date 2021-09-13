#include <kukadu/generated_skills/ChangeModeToIMP.hpp>
#include <kukadu/generated_skills/ChangeStiffness.hpp>
#include <kukadu/manipulation/skillfactory.hpp>

namespace kukadu {
        ChangeModeToIMP::ChangeModeToIMP(kukadu::StorageSingleton &storage,
                                         std::vector<KUKADU_SHARED_PTR<kukadu::Hardware> > hardware)
                : Controller(storage, "ChangeModeToIMP", hardware, 0.01) {

        }

        bool ChangeModeToIMP::requiresGraspInternal() {
            return false;
        }

        bool ChangeModeToIMP::producesGraspInternal() {
            return false;
        }

        std::shared_ptr<kukadu::ControllerResult> ChangeModeToIMP::executeInternal() {
            auto queue = KUKADU_DYNAMIC_POINTER_CAST<ControlQueue>(getUsedHardware()[0]);
            queue->install();
            queue->start();

            if (queue->getCurrentMode() != KukieControlQueue::KUKA_JNT_IMP_MODE) {
                queue->stopCurrentMode();
                ChangeStiffness setStandardStiffness(getStorage(), {queue});
                setStandardStiffness.setStiffnessType(0);
                setStandardStiffness.execute();
                queue->switchMode(KukieControlQueue::KUKA_JNT_IMP_MODE);
            }

            return nullptr;
        }

        std::string ChangeModeToIMP::getClassName() {
            return "ChangeModeToIMP";
        }

        void ChangeModeToIMP::createSkillFromThisInternal(std::string skillName) {
            // nothing to do
        }
}
