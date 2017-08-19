#ifndef KUKADU_GENERATED_SKILLS_BLOCKINGPOS_H
#define KUKADU_GENERATED_SKILLS_BLOCKINGPOS_H

#include <stdlib.h>
#include <kukadu/kukadu.hpp>

namespace kukadu {
        class BlockingPos : public kukadu::Controller {

        private:

        protected:

            virtual void createSkillFromThisInternal(std::string skillName);

        public:

            BlockingPos(kukadu::StorageSingleton &storage, std::vector<KUKADU_SHARED_PTR<kukadu::Hardware> > hardware);

            bool requiresGraspInternal();

            bool producesGraspInternal();

            std::shared_ptr<kukadu::ControllerResult> executeInternal();

            std::string getClassName();

        };
}

#endif
