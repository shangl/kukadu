#include <Python.h>
#include <boost/filesystem.hpp>
#include <kukadu/manipulation/sensingcontroller.hpp>

using namespace std;
namespace pf = boost::filesystem;

namespace kukadu {

    SensingController::SensingController(KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, int hapticMode, string caption, std::vector<KUKADU_SHARED_PTR<ControlQueue> > queues, vector<KUKADU_SHARED_PTR<GenericHand> > hands, std::string tmpPath, std::string classifierPath, std::string classifierFile, std::string classifierFunction, int simClassificationPrecision) : Controller(caption, 1) {

        currentIterationNum = 0;
        classifierParamsSet = false;
        simulationGroundTruth = 0;
        simulatedClassificationPrecision = simClassificationPrecision;

        this->generator = generator;

        this->hands = hands;
        this->queues = queues;
        this->tmpPath = tmpPath;
        this->hapticMode = hapticMode;
        this->classifierFile = classifierFile;
        this->classifierPath = classifierPath;
        this->classifierFunction = classifierFunction;

        this->stateCount = 4;

        databaseAlreadySet = false;

        bestParamC = 0.0;
        bestParamD = 0.0;
        bestParamParam1 = 0.0;
        bestParamParam2 = 0.0;

        Py_Initialize();

    }

    KUKADU_SHARED_PTR<kukadu_mersenne_twister> SensingController::getGenerator() {
        return generator;
    }

    std::vector<KUKADU_SHARED_PTR<ControlQueue> > SensingController::getQueues() {
        return queues;
    }

    std::vector<KUKADU_SHARED_PTR<GenericHand> > SensingController::getHands() {
        return hands;
    }

    std::string SensingController::getTmpPath() {
        return tmpPath;
    }

    std::string SensingController::getClassifierPath() {
        return classifierPath;
    }

    std::string SensingController::getClassifierFile() {
        return classifierFile;
    }

    std::string SensingController::getClassifierFunction() {
        return classifierFunction;
    }

    int SensingController::getHapticMode() {
        return hapticMode;
    }

    int SensingController::getSimClassificationPrecision() {
        return simulatedClassificationPrecision;
    }

    void SensingController::gatherData(std::string dataBasePath, std::string dataName) {

        if(!fileExists(dataBasePath))
            createDirectory(dataBasePath);

        gatherData(dataBasePath + dataName);

    }

    std::string SensingController::getDatabasePath() {
        return databasePath;
    }

    void SensingController::setDatabasePath(std::string databasePath) {
        this->databasePath = databasePath;
        databaseAlreadySet = true;
        createDataBase();
    }

    void SensingController::gatherData(std::string completePath) {

        vector<KUKADU_SHARED_PTR<ControlQueue> > castedQueues;
        for(int i = 0; i < queues.size(); ++i) {
            KUKADU_SHARED_PTR<ControlQueue> queue = queues.at(i);
            castedQueues.push_back(queue);
        }

        SensorStorage store(castedQueues, hands, 100);

        prepare();

        KUKADU_SHARED_PTR<kukadu_thread> storageThread = store.startDataStorage(completePath);

        performCore();

        store.stopDataStorage();
        storageThread->join();

    }

    int SensingController::getStateCount() {
        return stateCount;
    }

    void SensingController::setStateCount(const int& stateCount) {
        this->stateCount = stateCount;
    }

    std::string SensingController::getFirstRobotFileName() {
        return queues.at(0)->getRobotFileName();
    }

    std::vector<double> SensingController::callClassifier() {

        if(!databaseAlreadySet)
            throw KukaduException("(SensingController::callClassifier) database not defined yet");

        return callClassifier(databasePath, tmpPath + "hapticTest/" + queues.at(0)->getRobotFileName() + "_0", true, bestParamC, bestParamD, bestParamParam1, bestParamParam2);
    }

    int SensingController::performClassification() {

        if(!databaseAlreadySet)
            throw KukaduException("(SensingController::performClassification) database not defined yet");

        int classifierRes = -1;

        KUKADU_SHARED_PTR<kukadu_thread> cleanupThread;
        if(!getSimulationMode()) {

            int executeIt = 0;
            int temporaryHapticMode = hapticMode;
            cout << "(SensingController) selected sensing action is \"" << getCaption() << "\"; want to execute it? (0 = no / 1 = yes)" << endl;
            cin >> executeIt;

            if(executeIt == 1) {

                if(!classifierParamsSet) {
                    string errorMsg = "(SensingController) classifier parameters not yet set" ;
                    cerr << errorMsg << endl;
                    throw KukaduException(errorMsg.c_str());
                }

                pf::remove_all(tmpPath + "hapticTest");

                gatherData(tmpPath, "hapticTest");

                // start clean up in a separate thread
                cleanupThread = KUKADU_SHARED_PTR<kukadu_thread>(new kukadu_thread(&SensingController::cleanUp, this));

                stringstream s;
                s << tmpPath << "hapticTest_" << queues.at(0)->getRobotFileName() << "_0_" << currentIterationNum;
                copyFile(tmpPath + "hapticTest/" + queues.at(0)->getRobotFileName() + "_0", s.str());

            } else {
                if(!isShutUp) {
                    cout << "(SensinController) you decided not to perform the action" << endl;
                    cout << "(SensinController) switching temporarily to haptic mode HAPTIC_MODE_TERMINAL; continue" << endl;
                }
                temporaryHapticMode = SensingController::HAPTIC_MODE_TERMINAL;
            }

            if(temporaryHapticMode == SensingController::HAPTIC_MODE_TERMINAL) {
                cout << "(SensingController) what was the haptic result? [0, " << (getStateCount() - 1) << "]" << endl;
                cin >> classifierRes;
            } else if(temporaryHapticMode == SensingController::HAPTIC_MODE_CLASSIFIER) {
                vector<double> res = callClassifier(databasePath, tmpPath + "hapticTest/" + queues.at(0)->getRobotFileName() + "_0", true, bestParamC, bestParamD, bestParamParam1, bestParamParam2);
                int maxIdx = 0;
                double maxElement = res.at(0);
                for(int i = 1; i < getStateCount(); ++i) {
                    if(res.at(i) > maxElement) {
                        maxElement = res.at(i);
                        maxIdx = i;
                    }
                }
                classifierRes = maxIdx;
            } else {
                throw KukaduException("haptic mode not known");
            }

            if(!isShutUp)
                cout << "(SensinController) press enter to continue" << endl;
            getchar();

            pf::remove_all(tmpPath + "hapticTest");
            ++currentIterationNum;


        } else {

            // this is here for simulating a non-perfect classifier
            vector<double> precisionProbVec;
            precisionProbVec.push_back((double) simulatedClassificationPrecision);
            precisionProbVec.push_back((double) (100 - simulatedClassificationPrecision));
            KUKADU_DISCRETE_DISTRIBUTION<int> precisionProb(precisionProbVec.begin(), precisionProbVec.end());

            int correctClass = precisionProb(*generator);

            // simulate correct classification
            if(!correctClass)
                classifierRes = simulationGroundTruth;
            else {
                // classify it wrongly (random)
                classifierRes = simulationGroundTruth;
                while(classifierRes == simulationGroundTruth)
                    classifierRes = createRandomGroundTruthIdx();
            }

        }

        if(cleanupThread)
            cleanupThread->join();

        return classifierRes;

    }

    int SensingController::getSimulationGroundTruthIdx() {
        return simulationGroundTruth;
    }

    int SensingController::createRandomGroundTruthIdx() {
        vector<int> randValues;
        for(int i = 0; i < getStateCount(); ++i)
            randValues.push_back(1);
        classifierDist = KUKADU_DISCRETE_DISTRIBUTION<int>(randValues.begin(),randValues.end());
        return classifierDist(*generator);
    }

    void SensingController::setSimulationClassificationPrecision(int percent) {
        simulatedClassificationPrecision = percent;
    }

    void SensingController::setSimulationGroundTruth(int idx) {
        simulationGroundTruth = idx;
    }

    double SensingController::createDataBase() {

        if(!databaseAlreadySet)
            throw KukaduException("(SensingController::createDataBase) database not defined yet");

        int numClasses = 0;
        string path = getDatabasePath();
        vector<pair<int, string> > collectedSamples;
        if(!isShutUp)
            cout << "(SensingController) data is stored to " << path << endl;

        if(!fileExists(path)) {

            if(!isShutUp)
                cout << "(SensingController) folder doesn't exist - create" << endl;
            createDirectory(path);

            // create the database
            numClasses = getStateCount();
            if(!isShutUp)
                cout << "(SensingController) " << getCaption() << " offers " << numClasses << " classes" << endl;

            ofstream labelFile;
            labelFile.open((path + "labels").c_str(), std::ios_base::app);

            for(int currClass = 0; currClass < numClasses; ++currClass) {

                if(currClass != 0)
                    this->prepareNextState();

                int cont = 1;
                for(int sampleNum = 0; cont == 1; ++sampleNum) {

                    cout << "(SensingController) press key to collect sample number " << sampleNum << " for class " << currClass << " with sensing controller " << this->getCaption() << endl;
                    getchar();

                    stringstream s;
                    s << "class_" << currClass << "_sample_" << sampleNum;
                    string relativePath = s.str();
                    string relativeClassifyPath = relativePath + "/" + getFirstRobotFileName() + "_0";
                    string nextSamplePath = path + relativePath;
                    gatherData(nextSamplePath);
                    cleanUp();

                    collectedSamples.push_back(pair<int, string>(currClass, relativeClassifyPath));
                    labelFile << relativeClassifyPath << " " << currClass << endl;

                    cout << "(SensingController) want to collect another sample for class " << currClass << "? (0 = no / 1 = yes): ";
                    cin >> cont;

                }

            }

            labelFile.close();

        } else {
            if(!isShutUp)
                cout << "(SensingController) database for controller " << getCaption() << " exists - no collection required" << endl;
        }

        // if no classifier file exists
        if(!fileExists(path + "classRes")) {

            // determine confidence value on database
            vector<double> classRes = callClassifier(path, "", false, 0.0, 0.0, 0.0, 0.0);
            for(double res : classRes)
                cout << res << endl;

            double confidence = classRes.at(classRes.size() - 1);

            /*
            double bestParamC = classRes.at(classRes.size() - 4);
            double bestParamD = classRes.at(classRes.size() - 3);
            double bestParamPar1 = classRes.at(classRes.size() - 2);
            double bestParamPar2 = classRes.at(classRes.size() - 1);
            */

            double bestParamC = 0.0;
            double bestParamD = 0.0;
            double bestParamPar1 = 0.0;
            double bestParamPar2 = 0.0;

            ofstream ofile;
            ofile.open((path + "classRes").c_str());
            ofile << confidence << "\t" << bestParamC << "\t" << bestParamD << "\t" << bestParamPar1 << "\t" << bestParamPar2 << endl;
            ofile.close();

        }

        ifstream infile;
        infile.open((path + "classRes").c_str());
        double confidence = 0.0;
        double bestParamC = 0.0;
        double bestParamD = 0.0;
        double bestParamPar1 = 0.0;
        double bestParamPar2 = 0.0;
        infile >> confidence >> bestParamC >> bestParamD >> bestParamPar1 >> bestParamPar2;

        setCLassifierParams(bestParamC, bestParamD, bestParamParam1, bestParamParam2);

        if(!isShutUp)
            cout << "(SensingController) determined a confidence of " << confidence << endl;

        return confidence;

    }

    KUKADU_SHARED_PTR<ControllerResult> SensingController::performAction() {

        prepare();
        performCore();
        cleanUp();

        return KUKADU_SHARED_PTR<ControllerResult>();

    }

    void SensingController::setCLassifierParams(double bestParamC, double bestParamD, double bestParamParam1, double bestParamParam2) {
        classifierParamsSet = true;
        this->bestParamC = bestParamC;
        this->bestParamD = bestParamD;
        this->bestParamParam1 = bestParamParam1;
        this->bestParamParam2 = bestParamParam2;
    }

    std::vector<double> SensingController::callClassifier(std::string trainedPath, std::string passedFilePath, bool classify, double bestParamC, double bestParamD, double bestParamParam1, double bestParamParam2) {

        vector<double> retVals;
        string mName = classifierFile;
        string fName = classifierFunction;
        string argumentVal = trainedPath;

        PyObject *pName, *pModule, *pFunc;
        PyObject *pArgs, *pValue;

        PyRun_SimpleString("import sys");
        PyRun_SimpleString(string(string("sys.path.append('") + classifierPath + string("')")).c_str());
        PyRun_SimpleString("import trajlab_main");

        pName = PyUnicode_FromString(mName.c_str());
        pModule = PyImport_Import(pName);
        Py_DECREF(pName);

        if (pModule != NULL) {

            pFunc = PyObject_GetAttrString(pModule, fName.c_str());

            if (pFunc && PyCallable_Check(pFunc)) {

                //pArgs = PyTuple_New(7);
                pArgs = PyTuple_New(3);
                pValue = PyUnicode_FromString(argumentVal.c_str());

                if (!pValue) {

                    Py_DECREF(pArgs);
                    Py_DECREF(pModule);
                    fprintf(stderr, "Cannot convert argument\n");
                    return retVals;

                }

                PyTuple_SetItem(pArgs, 0, pValue);

                pValue = PyUnicode_FromString(passedFilePath.c_str());

                if (!pValue) {

                    Py_DECREF(pArgs);
                    Py_DECREF(pModule);
                    fprintf(stderr, "Cannot convert argument\n");

                }

                PyTuple_SetItem(pArgs, 1, pValue);

                pValue = PyFloat_FromDouble((classify) ? 1.0 : -1.0);

                if (!pValue) {

                    Py_DECREF(pArgs);
                    Py_DECREF(pModule);
                    fprintf(stderr, "Cannot convert argument\n");

                }

                PyTuple_SetItem(pArgs, 2, pValue);

                pValue = PyFloat_FromDouble(bestParamC);

                if (!pValue) {

                    Py_DECREF(pArgs);
                    Py_DECREF(pModule);
                    fprintf(stderr, "Cannot convert argument\n");

                }

                pValue = PyObject_CallObject(pFunc, pArgs);
                Py_DECREF(pArgs);

                if (pValue != NULL) {

                    int count = (int) PyList_Size(pValue);
                    for(int i = 0; i < count; ++i) {
                        PyObject* ptemp = PyList_GetItem(pValue, i);
                        retVals.push_back(PyFloat_AsDouble(ptemp));
                    }

                    // retVal = PyLong_AsLong(pValue);
                    Py_DECREF(pValue);

                } else {

                    Py_DECREF(pFunc);
                    Py_DECREF(pModule);
                    PyErr_Print();
                    fprintf(stderr,"Call failed\n");

                }

            } else {

                if (PyErr_Occurred())
                    PyErr_Print();
                cerr << "Cannot find function " << fName << endl;

            }

            Py_XDECREF(pFunc);
            Py_DECREF(pModule);

        }
        else {

            PyErr_Print();
            cerr << "Failed to load " << mName << endl;

        }

        return retVals;

    }

    void SensingController::writeLabelFile(std::string baseFolderPath, std::vector<std::pair<int, std::string> > collectedSamples) {

        ofstream outFile;
        outFile.open((baseFolderPath + "labels").c_str());

        for(int i = 0; i < collectedSamples.size(); ++i) {
            pair<int, string> sample = collectedSamples.at(i);
            outFile << sample.second << " " << sample.first << endl;
        }

    }

}
