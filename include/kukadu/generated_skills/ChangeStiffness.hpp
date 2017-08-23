#ifndef KUKADU_GENERATED_SKILLS_CHANGESTIFFNESS_H
#define KUKADU_GENERATED_SKILLS_CHANGESTIFFNESS_H

#include <kukadu/robot.hpp>
#include <kukadu/control.hpp>

namespace kukadu {
        class ChangeStiffness : public kukadu::Controller {

        private:
            int stiffnessType;
            double damping;

        protected:

            virtual void createSkillFromThisInternal(std::string skillName);

        public:

            ChangeStiffness(kukadu::StorageSingleton &storage,
                            std::vector<KUKADU_SHARED_PTR<kukadu::Hardware> > hardware);

            bool requiresGraspInternal();

            bool producesGraspInternal();

            std::shared_ptr<kukadu::ControllerResult> executeInternal();

            std::string getClassName();

            void setStiffnessType(int StiffnessType);

            void setStandardStiffnessDamping(double StandardStiffnessDamping);
        };
}

#endif
