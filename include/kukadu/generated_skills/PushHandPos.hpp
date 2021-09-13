#ifndef KUKADU_GENERATED_SKILLS_PUSHHANDPOS_H
#define KUKADU_GENERATED_SKILLS_PUSHHANDPOS_H

#include <kukadu/robot.hpp>
#include <kukadu/control.hpp>

namespace kukadu {
        class PushHandPos : public kukadu::Controller {

        private:

        protected:

            virtual void createSkillFromThisInternal(std::string skillName);

        public:

            PushHandPos(kukadu::StorageSingleton &storage, std::vector<KUKADU_SHARED_PTR<kukadu::Hardware> > hardware);

            bool requiresGraspInternal();

            bool producesGraspInternal();

            std::shared_ptr<kukadu::ControllerResult> executeInternal();

            std::string getClassName();

        };
}

#endif
