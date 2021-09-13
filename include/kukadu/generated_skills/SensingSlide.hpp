#ifndef KUKADU_GENERATED_SKILLS_SENSINGSLIDE_H
#define KUKADU_GENERATED_SKILLS_SENSINGSLIDE_H

#include <kukadu/robot.hpp>
#include <kukadu/control.hpp>
#include <kukadu/manipulation/playing/controllers.hpp>

namespace kukadu {
	class SensingSlide : public kukadu::SensingController {

	private:
	    std::vector< KUKADU_SHARED_PTR< kukadu::Hardware > > hardware;
	    virtual std::vector<KUKADU_SHARED_PTR< kukadu::Hardware> > getUsedHardware();

	protected:

	    virtual bool requiresGraspInternal();
	    virtual bool producesGraspInternal();

	public:

	    SensingSlide(kukadu::StorageSingleton& storage, std::vector< KUKADU_SHARED_PTR< kukadu::Hardware > > hardware);
	    virtual void prepare();
	    virtual void cleanUp();
	    virtual void performCore();

	    virtual std::string getClassName();
	    virtual KUKADU_SHARED_PTR<kukadu::SensingController> clone();
	};
}

#endif
