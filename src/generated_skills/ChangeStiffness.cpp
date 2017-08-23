#include <kukadu/generated_skills/ChangeStiffness.hpp>
#include <kukadu/manipulation/skillfactory.hpp>

namespace kukadu {
        ChangeStiffness::ChangeStiffness(kukadu::StorageSingleton &storage,
                                         std::vector<KUKADU_SHARED_PTR<kukadu::Hardware> > hardware)
                : Controller(storage, "ChangeStiffness", hardware, 0.01) {
            stiffnessType = 0;
            damping = -1;
        }

        bool ChangeStiffness::requiresGraspInternal() {
            return false;
        }

        bool ChangeStiffness::producesGraspInternal() {
            return false;
        }

        void ChangeStiffness::setStiffnessType(int stiffnessType) {
            this->stiffnessType = stiffnessType;
        }

        void ChangeStiffness::setStandardStiffnessDamping(double standardStiffnessDamping){
            damping = standardStiffnessDamping;
        }

        std::shared_ptr<kukadu::ControllerResult> ChangeStiffness::executeInternal() {
            auto queue = KUKADU_DYNAMIC_POINTER_CAST<ControlQueue>(getUsedHardware()[0]);
            queue->install();
            queue->start();

            switch (stiffnessType) {
                case -1:
                    queue->setStiffness(50, 1, KukieControlQueue::KUKA_STD_CPDAMPING, 99, 150, 1500);
                    break;
                case 1:
                    queue->setStiffness(500, 200, KukieControlQueue::KUKA_STD_CPDAMPING, 99, 150, 1500);
                    break;
                case 0:
                default:
                    queue->setStiffness(KukieControlQueue::KUKA_STD_XYZ_STIFF, KukieControlQueue::KUKA_STD_ABC_STIFF,
                                        damping, 99, 150, 1500);
                    break;
            }

            return nullptr;
        }

        std::string ChangeStiffness::getClassName() {
            return "ChangeStiffness";
        }

        void ChangeStiffness::createSkillFromThisInternal(std::string skillName) {
            // nothing to do
        }
}
