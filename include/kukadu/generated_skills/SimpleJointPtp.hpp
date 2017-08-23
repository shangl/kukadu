#ifndef KUKADU_GENERATED_SKILLS_SIMPLEJOINTPTP_H
#define KUKADU_GENERATED_SKILLS_SIMPLEJOINTPTP_H

#include <kukadu/robot.hpp>
#include <kukadu/control.hpp>

namespace kukadu {
    class SimpleJointPtp : public kukadu::Controller {

    private:
        double maxForce;
        std::vector<double> joints;

    protected:

        virtual void createSkillFromThisInternal(std::string skillName);

    public:

        SimpleJointPtp(kukadu::StorageSingleton &storage, std::vector<KUKADU_SHARED_PTR<kukadu::Hardware> > hardware);

        bool requiresGraspInternal();

        bool producesGraspInternal();

        std::shared_ptr<kukadu::ControllerResult> executeInternal();

        std::string getClassName();

        void setJoints(std::vector<double> joints);

        void setMaxForce(double maxForce);
    };
}

#endif
