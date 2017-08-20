#ifndef KUKADU_GENERATED_SKILLS_PUSHTRANSLATION_H
#define KUKADU_GENERATED_SKILLS_PUSHTRANSLATION_H

#include <stdlib.h>
#include <kukadu/kukadu.hpp>

namespace kukadu {
    class PushTranslation : public kukadu::Controller {

    private:
		bool pushForward;

    protected:

        virtual void createSkillFromThisInternal(std::string skillName);

    public:

        PushTranslation(kukadu::StorageSingleton &storage, std::vector<KUKADU_SHARED_PTR<kukadu::Hardware> > hardware);

        bool requiresGraspInternal();

        bool producesGraspInternal();

        std::shared_ptr<kukadu::ControllerResult> executeInternal();

        std::string getClassName();

        void setPushForward(bool pushForward);
    };
}

#endif
