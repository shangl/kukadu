#include <kukadu/control/controller.hpp>
#include <kukadu/storage/moduleusagesingleton.hpp>

using namespace std;

namespace kukadu {

    Controller::Controller(std::string caption, double simulationFailingProbability) {

        std::replace(caption.begin(), caption.end(), ' ', '_');

        isShutUp = true;

        std::replace(caption.begin(), caption.end(), ' ', '_');

        this->caption = caption;
        this->simulation = false;

        this->simulationFailingProbability = simulationFailingProbability;

    }

    void Controller::initialize() {

    }

    void Controller::setSimulationModeInChain(bool simulationMode) {

    }

    std::string Controller::getCaption() {
        return caption;
    }

    void Controller::setSimulationMode(bool simulationMode) {
        KUKADU_MODULE_START_USAGE();
        simulation = simulationMode;
        setSimulationModeInChain(simulationMode);
        KUKADU_MODULE_END_USAGE();
    }

    bool Controller::getSimulationMode() {
        return simulation;
    }

    void Controller::shutUp() {
        KUKADU_MODULE_START_USAGE();
        isShutUp = true;
        KUKADU_MODULE_END_USAGE();
    }

    void Controller::startTalking() {
        KUKADU_MODULE_START_USAGE();
        isShutUp = false;
        KUKADU_MODULE_END_USAGE();
    }

    double Controller::getSimFailingProb() {
        return simulationFailingProbability;
    }



}
