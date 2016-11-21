#ifndef KUKADU_SENSINGCONTROLLER_H
#define KUKADU_SENSINGCONTROLLER_H

#include <string>
#include <vector>
#include <kukadu/types/kukadutypes.hpp>
#include <kukadu/storage/sensorstorage.hpp>
#include <kukadu/types/controllerresult.hpp>
#include <kukadu/manipulation/controller.hpp>
#include <kukadu/robot/arm/kukiecontrolqueue.hpp>

namespace kukadu {

    class SensingController : public Controller {

    private:

        bool databaseAlreadySet;
        bool classifierParamsSet;

        int hapticMode;
        int stateCount;
        int currentIterationNum;
        int simulationGroundTruth;
        int simulatedClassificationPrecision;

        KUKADU_DISCRETE_DISTRIBUTION<int> classifierDist;

        double bestParamC;
        double bestParamD;
        double bestParamParam1;
        double bestParamParam2;

        std::string tmpPath;
        std::string databasePath;
        std::string classifierPath;
        std::string classifierFile;
        std::string classifierFunction;

        KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator;

        std::vector<KUKADU_SHARED_PTR<GenericHand> > hands;
        std::vector<KUKADU_SHARED_PTR<ControlQueue> > queues;

        std::vector<double> callClassifier(std::string trainedPath, std::string passedFilePath, bool classify,
                                           double bestParamC, double bestParamD, double bestParamParam1, double bestParamParam2);

        void writeLabelFile(std::string baseFolderPath, std::vector<std::pair<int, std::string> > collectedSamples);

        void gatherData(std::string completePath);
        void gatherData(std::string dataBasePath, std::string dataName);

    protected:

        KUKADU_SHARED_PTR<kukadu_mersenne_twister> getGenerator();
        std::vector<KUKADU_SHARED_PTR<ControlQueue> > getQueues();
        std::vector<KUKADU_SHARED_PTR<GenericHand> > getHands();
        std::string getTmpPath();
        std::string getClassifierPath();
        std::string getClassifierFile();
        std::string getClassifierFunction();
        int getHapticMode();
        int getSimClassificationPrecision();

    public:

        SensingController(KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, int hapticMode, std::string caption, std::vector<KUKADU_SHARED_PTR<ControlQueue> > queues, std::vector<KUKADU_SHARED_PTR<GenericHand> > hands,
                          std::string tmpPath, std::string classifierPath, std::string classifierFile, std::string classifierFunction,
                          int simClassificationPrecision);

        void setSimulationGroundTruth(int idx);
        void setSimulationClassificationPrecision(int percent);
        void setCLassifierParams(double bestParamC, double bestParamD, double bestParamParam1, double bestParamParam2);

        virtual void prepare() = 0;
        virtual void cleanUp() = 0;
        virtual void performCore() = 0;
        virtual void prepareNextState() = 0;

        virtual KUKADU_SHARED_PTR<kukadu::SensingController> clone() = 0;

        virtual int performClassification();
        int createRandomGroundTruthIdx();
        int getSimulationGroundTruthIdx();

        int getStateCount();
        void setStateCount(const int& stateCount);

        double createDataBase();

        void setDatabasePath(std::string databasePath);

        std::string getDatabasePath();
        std::string getFirstRobotFileName();

        std::vector<double> callClassifier();

        KUKADU_SHARED_PTR<ControllerResult> performAction();

        static const int HAPTIC_MODE_TERMINAL = 0;
        static const int HAPTIC_MODE_CLASSIFIER = 1;

    };

}

#endif
