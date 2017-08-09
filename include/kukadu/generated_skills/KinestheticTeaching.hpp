#ifndef KUKADU_SKILLS_KINESTHETICTEACHING_H
#define KUKADU_SKILLS_KINESTHETICTEACHING_H

#include <kukadu/control/dmp.hpp>
#include <kukadu/control/controller.hpp>

namespace kukadu {
	
	namespace skill {

		class KinestheticTeaching : public Controller {

		private:

			KUKADU_SHARED_PTR<Dmp> teachingDmp;
			KUKADU_SHARED_PTR<ControlQueue> teachingHardware;

		protected:

			virtual void createSkillFromThisInternal(std::string skillName);

		public:

			KinestheticTeaching (StorageSingleton& storage, KUKADU_SHARED_PTR<ControlQueue> hardware);

			bool requiresGraspInternal();
			bool producesGraspInternal();

			KUKADU_SHARED_PTR<ControllerResult> executeInternal();

			void bringToStartPos();
			std::pair<long long int, long long int> showDmp();
			void endTeachingAndTrainDmp(long long int startTime, long long int endTime);
			void testTrainedDmp();
			void installDmp(std::string dmpName);

			std::string getClassName();

		};
    
	}
    
}

#endif
