#ifndef KUKADU_CONCATCONTROLLER_H
#define KUKADU_CONCATCONTROLLER_H

#include <vector>
#include <string>
#include <kukadu/control/controller.hpp>

namespace kukadu {

class ConcatController : public Controller {

private:

    std::vector<KUKADU_SHARED_PTR<kukadu::Controller> > controllers;

public:

    ConcatController(std::vector<KUKADU_SHARED_PTR<kukadu::Controller> > controllers);

    virtual bool requiresGrasp();
    virtual bool producesGrasp();
    virtual bool getSimulationMode();

    virtual KUKADU_SHARED_PTR<ControllerResult> performAction();

    static std::string generateLabelFromControllers(std::vector<KUKADU_SHARED_PTR<kukadu::Controller> >& controllers);

};

}

#endif
