#ifndef KUKADU_GENERATED_SKILLS_WAITFORREACHED_H
#define KUKADU_GENERATED_SKILLS_WAITFORREACHED_H

#include <kukadu/robot.hpp>
#include <kukadu/control.hpp>

namespace kukadu {
        class WaitForReached : public kukadu::Controller {

        private:
            bool wait;

        protected:

            virtual void createSkillFromThisInternal(std::string skillName);

        public:

            WaitForReached(kukadu::StorageSingleton &storage,
                           std::vector<KUKADU_SHARED_PTR<kukadu::Hardware> > hardware);

            bool requiresGraspInternal();

            bool producesGraspInternal();

            std::shared_ptr<kukadu::ControllerResult> executeInternal();

            std::string getClassName();

            void setWait(bool wait);
        };
}

#endif
