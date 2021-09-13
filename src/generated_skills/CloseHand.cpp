#include <kukadu/generated_skills/CloseHand.hpp>
#include <kukadu/manipulation/skillfactory.hpp>

namespace kukadu {
        CloseHand::CloseHand(kukadu::StorageSingleton &storage,
                             std::vector<KUKADU_SHARED_PTR<kukadu::Hardware> > hardware)
                : Controller(storage, "CloseHand", hardware, 0.01) {

        }

        bool CloseHand::requiresGraspInternal() {
            return false;
        }

        bool CloseHand::producesGraspInternal() {
            return false;
        }

        std::shared_ptr<kukadu::ControllerResult> CloseHand::executeInternal() {
            auto &hardwareFactory = kukadu::HardwareFactory::get();
            auto hand = hardwareFactory.loadHardware("kukiehand_left");
            hand->install();
            hand->start();

            KUKADU_DYNAMIC_POINTER_CAST<kukadu::GenericHand>(hand)->setGrasp(eGID_SPHERICAL);
            KUKADU_DYNAMIC_POINTER_CAST<kukadu::GenericHand>(hand)->closeHand(1.0, 0.8);

            return nullptr;
        }

        std::string CloseHand::getClassName() {
            return "CloseHand";
        }

        void CloseHand::createSkillFromThisInternal(std::string skillName) {
            // nothing to do
        }
}
