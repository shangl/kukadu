#include <kukadu/manipulation/controller.hpp>

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
        simulation = simulationMode;
        setSimulationModeInChain(simulationMode);
    }

    bool Controller::getSimulationMode() {
        return simulation;
    }

    void Controller::shutUp() {
        isShutUp = true;
    }

    void Controller::startTalking() {
        isShutUp = false;
    }

    double Controller::getSimFailingProb() {
        return simulationFailingProbability;
    }



}
