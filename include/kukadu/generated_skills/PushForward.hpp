#ifndef KUKADU_GENERATED_SKILLS_PUSHFORWARD_H
#define KUKADU_GENERATED_SKILLS_PUSHFORWARD_H

#include <kukadu/robot.hpp>
#include <kukadu/control.hpp>

namespace kukadu {
        class PushForward : public kukadu::Controller {

        private:
            bool goBackToBlockingPos;

        protected:

            virtual void createSkillFromThisInternal(std::string skillName);

        public:

            PushForward(kukadu::StorageSingleton &storage, std::vector<KUKADU_SHARED_PTR<kukadu::Hardware> > hardware);

            bool requiresGraspInternal();

            bool producesGraspInternal();

            std::shared_ptr<kukadu::ControllerResult> executeInternal();

            std::string getClassName();

            void setGoBackToBlockingPos(bool goBackToBlockingPos);
        };
}

#endif
