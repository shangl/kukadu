#include <kukadu/generated_skills/SimpleJointPtp.hpp>
#include <kukadu/planning/simple.hpp>
#include <kukadu/manipulation/skillfactory.hpp>

using namespace arma;

namespace kukadu {
    SimpleJointPtp::SimpleJointPtp(kukadu::StorageSingleton &storage,
                                   std::vector<KUKADU_SHARED_PTR<kukadu::Hardware> > hardware)
            : Controller(storage, "SimpleJointPtp", hardware, 0.01) {

        maxForce = 30.0;
        auto queue = KUKADU_DYNAMIC_POINTER_CAST<KukieControlQueue>(hardware.front());
        queue->install();
        queue->start();

        joints.clear();
        int degOfFreedom = queue->getDegreesOfFreedom();
        for(int i = 0; i < degOfFreedom; ++i)
            joints.push_back(0.0);

    }

    bool SimpleJointPtp::requiresGraspInternal() {
        return false;
    }

    bool SimpleJointPtp::producesGraspInternal() {
        return false;
    }

    std::shared_ptr<kukadu::ControllerResult> SimpleJointPtp::executeInternal() {

        auto sLeftQueue13000 = KUKADU_DYNAMIC_POINTER_CAST<KukieControlQueue>(getUsedHardware()[0]);
        auto kin = KUKADU_DYNAMIC_POINTER_CAST<KukieControlQueue>(sLeftQueue13000)->getKinematics();

        kukadu::SimplePlanner sp(sLeftQueue13000, kin);
        sLeftQueue13000->startRollBackMode(1.5);

        std::vector <vec> desiredJoints;
        desiredJoints.push_back(sLeftQueue13000->getCurrentJoints().joints);
        desiredJoints.push_back(stdToArmadilloVec(this->joints));
        std::vector <vec> jointPlan = sp.planJointTrajectory(desiredJoints);

        for (vec joint : jointPlan) {

            if (sLeftQueue13000->getAbsoluteCartForce() > this->maxForce) {
                cerr << "(SimpleJointPtp) exceeded max force with force " << sLeftQueue13000->getAbsoluteCartForce() << " > " << this->maxForce << "; doing rollback" << endl;
                //sLeftQueue13000->rollBack(1.0);
                //break;
            } else {
                sLeftQueue13000->move(joint);
                sLeftQueue13000->synchronizeToQueue(1);
            }

        }

        sLeftQueue13000->stopJointRollBackMode();

        return nullptr;
    }

    std::string SimpleJointPtp::getClassName() {
        return "SimpleJointPtp";
    }

    void SimpleJointPtp::setJoints(std::vector<double> joints) {
        this->joints = joints;
    }

    void SimpleJointPtp::setMaxForce(double maxForce) {
        this->maxForce = maxForce;
    }

    void SimpleJointPtp::createSkillFromThisInternal(std::string skillName) {
        // nothing to do
    }
}
