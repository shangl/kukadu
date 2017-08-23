#include <kukadu/generated_skills/OpenHand.hpp>
#include <kukadu/manipulation/skillfactory.hpp>

namespace kukadu {
        OpenHand::OpenHand(kukadu::StorageSingleton &storage,
                           std::vector<KUKADU_SHARED_PTR<kukadu::Hardware> > hardware)
                : Controller(storage, "OpenHand", hardware, 0.01) {

        }

        bool OpenHand::requiresGraspInternal() {
            return false;
        }

        bool OpenHand::producesGraspInternal() {
            return false;
        }

        std::shared_ptr<kukadu::ControllerResult> OpenHand::executeInternal() {
            auto &hardwareFactory = kukadu::HardwareFactory::get();
            auto hand = hardwareFactory.loadHardware("kukiehand_left");
            hand->install();
            hand->start();

            KUKADU_DYNAMIC_POINTER_CAST<kukadu::GenericHand>(hand)->setGrasp(eGID_SPHERICAL);
            KUKADU_DYNAMIC_POINTER_CAST<kukadu::GenericHand>(hand)->closeHand(0.0, 0.8);

            return nullptr;
        }

        std::string OpenHand::getClassName() {
            return "OpenHand";
        }

        void OpenHand::createSkillFromThisInternal(std::string skillName) {
            // nothing to do
        }
}
