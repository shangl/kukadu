#include <stdlib.h>
#include <kukadu/kukadu.hpp>
#include <boost/program_options.hpp>
namespace kukadu {
	namespace skill
		{class ultimateSkill : public kukadu::Controller {

private:

	std::vector< KUKADU_SHARED_PTR< kukadu::Hardware > > hardware;

protected:

	virtual void createSkillFromThisInternal(std::string skillName);

public:

	ultimateSkill(kukadu::StorageSingleton& storage, std::vector< KUKADU_SHARED_PTR< kukadu::Hardware > > hardware);

	bool requiresGraspInternal();

	bool producesGraspInternal();

	std::shared_ptr<kukadu::ControllerResult> executeInternal();

	std::string getClassName();

};
}
}