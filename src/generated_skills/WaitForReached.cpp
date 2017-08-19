#include <kukadu/generated_skills/WaitForReached.hpp>

namespace kukadu {
    namespace skill {
        WaitForReached::WaitForReached(kukadu::StorageSingleton &storage,
                                       std::vector<KUKADU_SHARED_PTR<kukadu::Hardware> > hardware)
                : Controller(storage, "WaitForReached", hardware, 0.01) {

        }

        bool WaitForReached::requiresGraspInternal() {
            return false;
        }

        bool WaitForReached::producesGraspInternal() {
            return false;
        }

        void WaitForReached::setWait(bool wait) {
            this->wait = wait;
        }

        std::shared_ptr<kukadu::ControllerResult> WaitForReached::executeInternal() {
            auto hand = KUKADU_DYNAMIC_POINTER_CAST<KukieHand>(getUsedHardware()[0]);
            hand->install();
            hand->start();

            hand->setWaitForReached(this->wait);

            return nullptr;
        }

        std::string WaitForReached::getClassName() {
            return "WaitForReached";
        }

        void WaitForReached::createSkillFromThisInternal(std::string skillName) {
            // nothing to do
        }
    }
}
