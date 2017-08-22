#ifndef KUKADU_SKILLS_KINESTHETICTEACHING_H
#define KUKADU_SKILLS_KINESTHETICTEACHING_H

#include <kukadu/control/dmp.hpp>
#include <kukadu/control/controller.hpp>

namespace kukadu {

    class KinestheticTeaching : public Controller {

    private:

        long long int startTime;
        long long int endTime;

        KUKADU_SHARED_PTR<Dmp> teachingDmp;
        KUKADU_SHARED_PTR<ControlQueue> teachingHardware;

        bool teachingRunning;

        KUKADU_SHARED_PTR<kukadu_thread> teachingThread;

    protected:

        virtual void createSkillFromThisInternal(std::string skillName);

    public:

        KinestheticTeaching(StorageSingleton &storage, KUKADU_SHARED_PTR<ControlQueue> hardware);

        bool requiresGraspInternal();

        bool producesGraspInternal();

        KUKADU_SHARED_PTR<ControllerResult> executeInternal();

        void bringToStartPos();

        void showDmp();

        void endTeachingAndTrainDmp();

        void testTrainedDmp();

        void installDmp(std::string dmpName);

        std::string getClassName();

    };


}

#endif
