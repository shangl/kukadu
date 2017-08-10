#include <sstream>
#include <kukadu/types/sensordata.hpp>
#include <kukadu/storage/sensorstorage.hpp>
#include <kukadu/storage/moduleusagesingleton.hpp>
#include <kukadu/storage/sensorstoragesingleton.hpp>

using namespace std;
using namespace arma;

namespace kukadu {

    SensorStorageSingleton::SensorStorageSingleton() {

    }

    SensorStorageSingleton& SensorStorageSingleton::get() {

        static SensorStorageSingleton instance;
        return instance;

    }

    KUKADU_SHARED_PTR<Hardware> SensorStorageSingleton::getRegisteredHardware(std::string name) {

        if(registeredHardware.find(name) != registeredHardware.end())
            return registeredHardware[name];

        throw KukaduException("(SensorStorageSingleton) requested hardware not registered yet");

    }

    void SensorStorageSingleton::registerHardware(KUKADU_SHARED_PTR<Hardware> hardware) {

        if(hardware) {

            instanceMutex.lock();

            auto hardwareName = hardware->getHardwareInstanceName();
            bool hardwareAlreadyRegistered = registeredHardware.find(hardwareName) != registeredHardware.end();
            if(hardwareAlreadyRegistered && registeredHardware[hardware->getHardwareInstanceName()] != hardware) {
                instanceMutex.unlock();
                throw KukaduException("(SensorStorageSingleton) different hardware with the same instance name is already registered");
            }

            if(!hardwareAlreadyRegistered) {
                registeredInstanceNames.push_back(hardwareName);
                registeredHardware[hardwareName] = hardware;
                startedThreads[hardwareName] = {false, nullptr};
                startedCount[hardwareName] = 0;
            }

            instanceMutex.unlock();

        }

    }

    void SensorStorageSingleton::registerHardware(std::vector<KUKADU_SHARED_PTR<Hardware> > hardware) {
        for(auto& hw : hardware)
            registerHardware(hw);
    }

    void SensorStorageSingleton::initiateStorageAllRegistered() {
        initiateStorage(registeredInstanceNames);
    }

    std::vector<bool> SensorStorageSingleton::initiateStorage(std::vector<KUKADU_SHARED_PTR<Hardware> > hardware) {

        vector<string> instanceNames;
        for(auto& hw : hardware)
            if(hw)
                instanceNames.push_back(hw->getHardwareInstanceName());
        auto retVal =  initiateStorage(instanceNames);

        return retVal;
    }

    std::vector<bool> SensorStorageSingleton::initiateStorage(std::vector<std::string> instanceNames) {

        vector<bool> initiatedFlags;
        for(auto& requiredInstances : instanceNames)
            initiatedFlags.push_back(initiateStorage(requiredInstances));

        return initiatedFlags;

    }

    bool SensorStorageSingleton::initiateStorage(std::string instanceName) {

        instanceMutex.lock();

        bool hardwareAlreadyRegistered = registeredHardware.find(instanceName) != registeredHardware.end();
        if(hardwareAlreadyRegistered) {

            auto& hardware = registeredHardware[instanceName];
            auto& actualThreadPair = startedThreads[instanceName];
            auto& actualInitiationCount = startedCount[instanceName];
            auto& actualThread = actualThreadPair.second;

            ++actualInitiationCount;

            if(!actualThread || (actualThread && !actualThreadPair.first)) {
                actualThreadPair.first = true;

                actualThread = make_shared<kukadu_thread>([this, hardware]() {
                    ros::Rate r(hardware->getPreferredPollingFrequency());
                    auto& threadPair = this->startedThreads[hardware->getHardwareInstanceName()];
                    auto& runningFlag = threadPair.first;
                    while(runningFlag) {
                        if(hardware->isStarted())
                            hardware->storeCurrentSensorDataToDatabase();
                        r.sleep();
                    }
                    runningFlag = false;
                });

            }

            instanceMutex.unlock();
            return !actualThreadPair.first;

        } else {
            instanceMutex.unlock();
            throw KukaduException("(SensorStorageSingleton) storing of sensor data cannot be initiated (instance name is not registered yet)");
        }

        instanceMutex.unlock();
        return true;

    }

    void SensorStorageSingleton::stopStorageAll() {

        instanceMutex.lock();

        // first go through all threads and send kill signal
        for(auto& threadPair : startedThreads)
            threadPair.second.first = false;

        for(auto& c : startedCount)
            c.second = 0;

        // then wait until all of them are actually done
        for(auto& threadPair : startedThreads)
            if(!startedCount[threadPair.first])
                if(threadPair.second.second && threadPair.second.second->joinable())
                    threadPair.second.second->join();

        instanceMutex.unlock();

    }

    void SensorStorageSingleton::stopStorage(std::vector<KUKADU_SHARED_PTR<Hardware> > hardware) {

        vector<string> hwNames;
        for(auto& hw : hardware)
            if(hw)
                hwNames.push_back(hw->getHardwareInstanceName());
        stopStorage(hwNames);

    }

    void SensorStorageSingleton::stopStorage(std::vector<std::string> instanceNames) {

        instanceMutex.lock();

        // first go through all threads and send kill signal
        for(auto& threadPair : startedThreads)
            if(std::find(instanceNames.begin(), instanceNames.end(), threadPair.first) != instanceNames.end()) {
                startedCount[threadPair.first] = (startedCount[threadPair.first] > 0) ? startedCount[threadPair.first] - 1 : 0;
                // only destroy if no other instance is expecting data to be stored
                if(!startedCount[threadPair.first])
                    threadPair.second.first = false;
            }

        // then wait until all of them are actually done
        for(auto& threadPair : startedThreads)
            if(std::find(instanceNames.begin(), instanceNames.end(), threadPair.first) != instanceNames.end())
                if(!startedCount[threadPair.first])
                    if(threadPair.second.second && threadPair.second.second->joinable())
                        threadPair.second.second->join();

        instanceMutex.unlock();

    }

    void SensorStorage::initSensorStorage(std::vector<KUKADU_SHARED_PTR<ControlQueue> > queues, std::vector<KUKADU_SHARED_PTR<GenericHand> > hands, double pollingFrequency) {

        this->hands = hands;
        this->queues = queues;
        this->pollingFrequency = pollingFrequency;

        stopped = false;
        storeCartAbsFrc = false;
        storeTime = storeJntPos = storeCartPos = storeJntFrc = storeCartFrcTrq = storeHndJntPos = storeHndTctle = true;

    }

    SensorStorage::SensorStorage(StorageSingleton& storage, std::vector<KUKADU_SHARED_PTR<ControlQueue> > queues, std::vector<KUKADU_SHARED_PTR<GenericHand> > hands, double pollingFrequency)
        : dbStorage(storage) {

        initSensorStorage(queues, hands, pollingFrequency);

    }

    std::pair<long long int, long long int> SensorStorage::transferArmDataToDb(StorageSingleton& dbStorage, KUKADU_SHARED_PTR<ControlQueue> queue, std::string file) {

        long long int currentTime = getCurrentTime();
        auto robotId = queue->getRobotId();
        auto jointIds = queue->getJointIds();
        vec fakeFrc;
        auto storeFrc = false;

        auto fileData = readStorage(queue, file);

        fileData->removeDuplicateTimes();

        auto timeMilliSec = fileData->getTimeInMilliSeconds();
        vec normTimeSec = fileData->getNormalizedTimeInSeconds();

        mat positions = fileData->getJointPos();
        mat velocities = computeDiscreteDerivatives(normTimeSec, positions);
        mat accelerations = computeDiscreteDerivatives(normTimeSec, velocities);

        for(int i = 0; i < positions.n_rows; ++i) {

            vec currPos = positions.row(i).t();
            vec currVel = velocities.row(i).t();
            vec currAcc = accelerations.row(i).t();

            storeJointInfoToDatabase(dbStorage, robotId, currentTime + timeMilliSec.at(i), jointIds, currPos, currVel, currAcc, storeFrc, fakeFrc);

        }

        return {currentTime + timeMilliSec.front(), currentTime + timeMilliSec.back()};

    }

    arma::vec SensorStorage::loadSampleTimesInRangeFromDb(StorageSingleton& storage, KUKADU_SHARED_PTR<ControlQueue> queue, long long int startTime, long long int endTime) {

        stringstream s;
        s << "select distinct(time_stamp) from joint_mes where robot_id = " << queue->getRobotId() << " and " <<
             " time_stamp >= " << startTime << " and time_stamp <= " << endTime <<
             " order by time_stamp asc";

        vector<long long int> timeStamps;
        auto timeRes = storage.executeQuery(s.str());
        while(timeRes->next())
            timeStamps.push_back(timeRes->getInt64("time_stamp"));

        timeStamps.resize(timeStamps.size() - 1);
        return convertAndRemoveOffset(timeStamps);

    }

    arma::mat SensorStorage::loadJointsFromDb(StorageSingleton& storage, KUKADU_SHARED_PTR<ControlQueue> queue, long long int startTime, long long int endTime) {

        auto robotId = queue->getRobotId();
        auto jointIds = queue->getJointIds();

        stringstream s;
        s << "select joint_id, time_stamp, position from joint_mes where robot_id = " << robotId << " and " <<
             "time_stamp >= " << startTime << " and time_stamp <= " << endTime << " and joint_id in (";
        for(int i = 0; i < jointIds.size(); ++i) {
            if(i)
                s << ", ";
            s << jointIds.at(i);
        }
        s << ") order by time_stamp asc, joint_id asc";

        auto jointRes = storage.executeQuery(s.str());

        vector<bool> degOfFreedomRetrieved;
        for(int i = 0; i < jointIds.size(); ++i)
            degOfFreedomRetrieved.push_back(false);

        long long int prevTimeStamp = -1;
        bool firstRun = true;

        mat loadedJoints;
        vector<long long int> retTimes;

        while(jointRes->next()) {

            vec currJoint(jointIds.size());
            auto currJointId = jointRes->getInt("joint_id");
            long long int currTime = jointRes->getInt64("time_stamp");
            double currPos = jointRes->getDouble("position");

            if(firstRun) {
                prevTimeStamp = currTime;
                firstRun = false;
            }

            // if new timestamp --> store and continue
            if(prevTimeStamp != currTime) {

                // check if all deg of freedom were retrieved
                for(int i = 0; i < degOfFreedomRetrieved.size(); ++i) {
                    auto ret = degOfFreedomRetrieved.at(i);

                    if(ret)
                        ret = false;
                    else
                        throw KukaduException("(SensorStorage) not all requested joints are stored in the database");
                }

                retTimes.push_back(prevTimeStamp);
                loadedJoints = join_cols(loadedJoints, currJoint.t());

                prevTimeStamp = currTime;

            }

            auto idIt = find(jointIds.begin(), jointIds.end(), currJointId);
            if(idIt != jointIds.end()) {

                auto idIdx = (int) (idIt - jointIds.begin());
                degOfFreedomRetrieved.at(idIdx) = true;
                currJoint(idIdx) = currPos;

            } else
                throw KukaduException("(SensorStorage) retrieved something that was not requested");

        }

        return loadedJoints;

    }

    void SensorStorage::setExportMode(int mode) {

        KUKADU_MODULE_START_USAGE();

        storeCartAbsFrc = storeTime = storeJntPos = storeCartPos = storeJntFrc = storeCartFrcTrq = storeHndJntPos = storeHndTctle = false;

        //if(mode & STORE_TIME)
        // in the current version, the time must always be stored
        storeTime = true;

        if(mode & STORE_RBT_JNT_POS)
            storeJntPos = true;

        if(mode & STORE_RBT_CART_POS)
            storeCartPos = true;

        if(mode & STORE_RBT_JNT_FTRQ)
            storeJntFrc = true;

        if(mode & STORE_RBT_CART_FTRQ)
            storeCartFrcTrq = true;

        if(mode & STORE_HND_JNT_POS)
            storeHndJntPos = true;

        if(mode & STORE_HND_TCTLE)
            storeHndTctle = true;

        if(mode & STORE_CART_ABS_FRC)
            storeCartAbsFrc = true;

        KUKADU_MODULE_END_USAGE();

    }

    long long int SensorStorage::startDataStorage(std::string folderName) {

        KUKADU_MODULE_START_USAGE();

        long long int startTime = 0;
        if(queues.size())
            startTime = queues.front()->getCurrentTime();
        else if(hands.size())
            startTime = hands.front()->getCurrentTime();

        if(queues.size() || hands.size()) {

            bool createFolderWorked = false;
            if(folderName != "") {

                if(createFolderWorked = createDirectory(folderName)) {

                    queueStreams.clear();
                    for(int i = 0; i < queues.size(); ++i) {
                        stringstream s;
                        s << i;
                        KUKADU_SHARED_PTR<std::ofstream> queueFile = KUKADU_SHARED_PTR<std::ofstream>(new ofstream());
                        queueFile->open((folderName + string("/") + queues.at(i)->getRobotFileName() + string("_") + s.str()).c_str());
                        queueStreams.push_back(queueFile);
                    }

                    handStreams.clear();
                    for(int i = 0; i < hands.size(); ++i) {
                        stringstream s;
                        s << i;
                        KUKADU_SHARED_PTR<std::ofstream> queueFile = KUKADU_SHARED_PTR<std::ofstream>(new ofstream());
                        queueFile->open((folderName + string("/") + hands.at(i)->getHandName() + string("_") + s.str()).c_str());
                        handStreams.push_back(queueFile);
                    }

                }

            }

            if(folderName == "" || createFolderWorked)
                thr = KUKADU_SHARED_PTR<kukadu_thread>(new kukadu_thread(&SensorStorage::store, this));

        } else
            throw KukaduException("(SensorStorage) no queues and hands to store");

        KUKADU_MODULE_END_USAGE();

        return startTime;

    }

    long long int SensorStorage::stopDataStorage() {

        KUKADU_MODULE_START_USAGE();

        long long int endTime = 0;
        if(queues.size())
            endTime = queues.front()->getCurrentTime();
        else if(hands.size())
            endTime = hands.front()->getCurrentTime();

        stopped = true;
        if(thr && thr->joinable())
            thr->join();

        for(int i = 0; i < queueStreams.size(); ++i)
            queueStreams.at(i)->close();
        queueStreams.clear();

        for(int i = 0; i < handStreams.size(); ++i)
            handStreams.at(i)->close();
        handStreams.clear();

        KUKADU_MODULE_END_USAGE();

        return endTime;

    }

    void SensorStorage::writeVectorInLine(KUKADU_SHARED_PTR<ofstream> stream, arma::vec writeVec) {
        for(int i = 0; i < writeVec.n_elem; ++i)
            *stream << writeVec(i) << "\t";
    }

    void SensorStorage::writeMatrixInLine(KUKADU_SHARED_PTR<ofstream> stream, arma::mat writeMat) {
        for(int i = 0; i < writeMat.n_rows; ++i)
            for(int j = 0; j < writeMat.n_cols; ++j)
                *stream << writeMat(i, j) << "\t";
        *stream << "|\t";
    }

    void SensorStorage::writeLabels(KUKADU_SHARED_PTR<std::ofstream> stream, std::vector<std::string> labels) {

        for(int i = 0; i < labels.size(); ++i)
            *stream << labels.at(i) << "\t";
        *stream << endl;

    }

    void SensorStorage::writeMatrixMetaInfo(KUKADU_SHARED_PTR<std::ofstream> stream, int matrixNum, int xDim, int yDim) {

        *stream << "matrix " << matrixNum << ": " << xDim << "x" << yDim << endl;

    }

    void SensorStorage::store() {

        KUKADU_MODULE_START_USAGE();
        storeData(true, std::vector<KUKADU_SHARED_PTR<SensorData> >(), queueStreams);
        KUKADU_MODULE_END_USAGE();

    }

    void SensorStorage::storeData(bool storeHeader, KUKADU_SHARED_PTR<SensorData> data, std::string file) {

        KUKADU_MODULE_START_USAGE();
        storeData(storeHeader, std::vector<KUKADU_SHARED_PTR<SensorData> >{data}, {file});
        KUKADU_MODULE_END_USAGE();

    }

    void SensorStorage::storeData(bool storeHeader, std::vector<KUKADU_SHARED_PTR<SensorData> > data, std::vector<std::string> files) {

        KUKADU_MODULE_START_USAGE();

        std::vector<KUKADU_SHARED_PTR<ofstream> > queueStreams;

        // if the data shall not be stored in a file, but in the database --> pass an empty stream vector
        if(files.front() == "")
            files.clear();

        for(int i = 0; i < files.size(); ++i) {
            string file = files.at(i);
            KUKADU_SHARED_PTR<ofstream> currentStream = KUKADU_SHARED_PTR<ofstream>(new ofstream());
            currentStream->open(file.c_str());
            queueStreams.push_back(currentStream);
        }
        storeData(storeHeader, data, queueStreams);

        KUKADU_MODULE_END_USAGE();

    }

    void SensorStorage::storeData(bool storeHeader, std::vector<KUKADU_SHARED_PTR<SensorData> > data, std::vector<KUKADU_SHARED_PTR<std::ofstream> > queueStreams) {

        KUKADU_MODULE_START_USAGE();

        bool storeToFile = (queueStreams.size()) ? true : false;

        if(storeToFile && data.size() != queueStreams.size() && data.size())
            throw KukaduException("(SensorStorage) number of files does not match the number of data blocks to store");

        stopped = false;
        double waitTime = 1.0 / pollingFrequency;
        ros::Rate rate(pollingFrequency);

        bool firstTime = true;
        int iterationSize = (data.size()) ? data.size() : queues.size();

        long long int time = 0;
        long long int currentTime = 0;

        vector<int> robotIdVec;
        vector<vector<int> > jointIdsVec;
        vector<vector<string> > jointNamesVec;
        for(int i = 0; i < iterationSize; ++i) {
            jointIdsVec.push_back(queues.at(i)->getJointIds(queues.at(i)->getJointNames()));
            jointNamesVec.push_back(queues.at(i)->getJointNames());
            robotIdVec.push_back(queues.at(i)->getRobotId());
        }

        vector<bool> firstTimes(iterationSize);
        for(int i = 0; i < firstTimes.size(); ++i)
            firstTimes.at(i) = true;

        vector<mes_result> prevJoints(iterationSize);
        vector<mes_result> prevPrevJoints(iterationSize);

        for(int dataPointIdx = 0; !stopped && (!data.size() || dataPointIdx < data.at(0)->getNormalizedTimeInSeconds().n_elem); ++dataPointIdx) {

            for(int i = 0; i < iterationSize; ++i) {

                KUKADU_SHARED_PTR<ControlQueue> currentQueue = queues.at(i);
                KUKADU_SHARED_PTR<ofstream> currentOfStream;
                if(storeToFile)
                    currentOfStream = queueStreams.at(i);

                mes_result joints;
                mes_result cartPos;
                mes_result jntFrcTrq;
                mes_result cartFrcTrq;
                mes_result cartAbsFrcTrq;

                geometry_msgs::Pose cartPose;

                string referenceFrame = currentQueue->getCartesianReferenceFrame();
                string linkName = currentQueue->getCartesianLinkName();

                double absCartFrc = 0.0;

                // if not collect data live
                if(data.size()) {

                    time = data.at(i)->getTimeInMilliSeconds().at(dataPointIdx);

                    if(storeJntPos) {
                        joints.time = time;
                        joints.joints = data.at(i)->getJointPosRow(dataPointIdx);

                    }

                    if(storeCartPos) {
                        cartPos.time = time;
                        cartPos.joints = data.at(i)->getCartPosRow(dataPointIdx);
                        cartPose = data.at(i)->getCartPose(dataPointIdx);
                    }

                    if(storeJntFrc) {
                        cartPos.time = time;
                        jntFrcTrq.joints = data.at(i)->getJointForcesRow(dataPointIdx);
                    }

                    if(storeCartFrcTrq) {
                        cartPos.time = time;
                        cartFrcTrq.joints = data.at(i)->getCartFrcTrqsRow(dataPointIdx);
                    }

                    if(storeCartAbsFrc) {
                        cartPos.time = time;
                        cartAbsFrcTrq.joints = data.at(i)->getCartFrcTrqsRow(dataPointIdx);
                        absCartFrc = 0.0;
                        for(int i = 0; i < 3; ++i) {
                            absCartFrc += pow(cartAbsFrcTrq.joints(i), 2);
                        }
                        absCartFrc = sqrt(absCartFrc);
                    }

                } else {

                    if(storeJntPos)
                        joints = currentQueue->getCurrentJoints();

                    if(storeCartPos) {
                        cartPos = currentQueue->getCurrentCartesianPos();
                        cartPose = currentQueue->getCurrentCartesianPose();
                    }

                    if(storeJntFrc)
                        jntFrcTrq = currentQueue->getCurrentJntFrc();

                    if(storeCartFrcTrq)
                        cartFrcTrq = currentQueue->getCurrentCartesianFrcTrq();

                    if(storeCartAbsFrc)
                        absCartFrc = currentQueue->getAbsoluteCartForce();

                    time = joints.time;

                }

                if(storeJntPos) {
                    prevPrevJoints.at(i) = prevJoints.at(i);
                    prevJoints.at(i) = joints;
                    if(firstTimes.at(i)) {
                        prevPrevJoints.at(i) = prevJoints.at(i) = joints;
                        firstTimes.at(i) = false;
                    }
                }

                if(firstTime && storeHeader) {

                    vector<string> labels;

                    if(storeTime)
                        labels.push_back("time");

                    if(storeJntPos) {

                        auto& jointNames = jointNamesVec.at(i);
                        for(int j = 0; j < jointNames.size(); ++j)
                            labels.push_back(string("joint_") + jointNames.at(j));

                    }

                    if(storeCartPos) {

                        labels.push_back("cart_pos_x");
                        labels.push_back("cart_pos_y");
                        labels.push_back("cart_pos_z");
                        labels.push_back("cart_quat_x");
                        labels.push_back("cart_quat_y");
                        labels.push_back("cart_quat_z");
                        labels.push_back("cart_quat_w");

                    }

                    if(storeJntFrc) {

                        auto& jointNames = jointNamesVec.at(i);
                        for(int j = 0; j < jointNames.size(); ++j)
                            labels.push_back(string("force_joint_") + jointNames.at(j));

                    }

                    if(storeCartFrcTrq) {

                        labels.push_back("cart_force_x");
                        labels.push_back("cart_force_y");
                        labels.push_back("cart_force_z");
                        labels.push_back("cart_trq_x");
                        labels.push_back("cart_trq_y");
                        labels.push_back("cart_trq_z");

                    }

                    if(storeCartAbsFrc) {

                        labels.push_back("cart_abs_force");

                    }

                    if(storeToFile)
                        writeLabels(currentOfStream, labels);

                }

                if(currentTime != time) {

                    if(storeToFile) {

                        if(storeTime)
                            *currentOfStream << time << "\t";

                        if(storeJntPos)
                            writeVectorInLine(currentOfStream, joints.joints);

                        if(storeCartPos)
                            writeVectorInLine(currentOfStream, cartPos.joints);

                        if(storeJntFrc)
                            writeVectorInLine(currentOfStream, jntFrcTrq.joints);

                        if(storeCartFrcTrq)
                            writeVectorInLine(currentOfStream, cartFrcTrq.joints);

                        if(storeCartAbsFrc) {
                            vec absForce(1);
                            absForce(0) = absCartFrc;
                            writeVectorInLine(currentOfStream, absForce);
                        }

                        *currentOfStream << endl;

                    } else {

                        auto& robotId = robotIdVec.at(i);
                        if(storeJntPos || storeJntFrc) {

                            vec vel;
                            vec acc;
                            // if the previous times are the same, it is the first time - acceleration are 0
                            if(prevPrevJoints.at(i).time == prevJoints.at(i).time)
                                acc = zeros(joints.joints.n_elem);
                            else {
                                auto timeDiffInSec = (double) (prevJoints.at(i).time - prevPrevJoints.at(i).time) / 1000.0;
                                acc = (prevPrevJoints.at(i).joints - prevJoints.at(i).joints) / timeDiffInSec;
                            }

                            // same reasoning vor velocity
                            if(prevJoints.at(i).time == joints.time)
                                vel = zeros(joints.joints.n_elem);
                            else {
                                auto timeDiffInSec = (double) (joints.time - prevJoints.at(i).time) / 1000.0;
                                vel = (joints.joints - prevJoints.at(i).joints) / timeDiffInSec;
                            }

                            // store the data in the database
                            auto& jointIds = jointIdsVec.at(i);
                            storeJointInfoToDatabase(dbStorage, robotId, time, jointIds, joints.joints, vel, acc, true, jntFrcTrq.joints);

                        }

                        if(storeCartPos || storeCartFrcTrq || storeCartAbsFrc)
                            storeCartInformation(dbStorage, robotId, time, referenceFrame, linkName, cartPose, cartFrcTrq.joints, absCartFrc, storeCartPos, storeCartFrcTrq, storeCartAbsFrc);

                    }

                    currentTime = time;

                }

            }

            for(int i = 0; i < hands.size(); ++i) {

                KUKADU_SHARED_PTR<GenericHand> currentHand = hands.at(i);
                KUKADU_SHARED_PTR<ofstream> currentOfStream = handStreams.at(i);
                std::vector<arma::mat> currentSensing = currentHand->getTactileSensing();

                if(firstTime && storeHeader) {

                    vector<string> labels;

                    labels.push_back("time");

                    for(int k = 0, running = 0; k < currentSensing.size(); ++k) {
                        writeMatrixMetaInfo(currentOfStream, k, currentSensing.at(k).n_rows, currentSensing.at(k).n_cols);
                        for(int j = 0; j < currentSensing.at(k).n_cols * currentSensing.at(k).n_rows; ++j, ++running) {
                            stringstream s;
                            s << running;
                            labels.push_back(s.str());
                        }
                    }

                    writeLabels(currentOfStream, labels);

                }

                *currentOfStream << currentTime << "\t";
                for(int j = 0; j < currentSensing.size(); ++j)
                    writeMatrixInLine(currentOfStream, currentSensing.at(j));

                *currentOfStream << endl;

                if(queues.size() == 0)
                    currentTime += waitTime;

            }

            firstTime = false;

            if(!data.size())
                rate.sleep();

        }

        KUKADU_MODULE_END_USAGE();

    }

    void SensorStorage::storeCartInformation(StorageSingleton& dbStorage, const int& robotId, const long long int& timeStamp, const std::string& referenceFrame, const std::string& linkName, geometry_msgs::Pose& cartesianPose, const arma::vec& frcTrq, const double& absFrc, const bool& storePos, const bool& storeFrc, const bool& storeAbsFrc) {

        KUKADU_MODULE_START_USAGE();

        if(storePos || storeFrc || storeAbsFrc) {

            stringstream s;
            s << "robot_id = " << robotId;
            auto robotWhere = s.str();

            auto nextId = dbStorage.getNextIdInTable("cart_mes", "cart_mes_id");
            auto referenceId = dbStorage.getCachedLabelId("reference_frames", "frame_id", "frame_name", referenceFrame, robotWhere);
            auto linkId = dbStorage.getCachedLabelId("links", "link_id", "link_name", linkName, robotWhere);
            auto rpy = quatToRpy(cartesianPose.orientation);

            vector<string> stmts;

            s.str("");
            s << "insert into cart_mes(cart_mes_id, time_stamp, robot_id, reference_frame_id, link_id) values(" << nextId << ", " << timeStamp << ", " << robotId << ", " << referenceId << ", " << linkId << ")";
            stmts.push_back(s.str());

            if(storePos) {
                s.str("");
                s << "insert into cart_mes_pos(cart_mes_id, cart_pos_x, cart_pos_y, cart_pos_z, cart_rot_x, cart_rot_y, cart_rot_z) values (" <<
                     nextId << ", " << cartesianPose.position.x << ", " << cartesianPose.position.y << ", " << cartesianPose.position.z << ", " <<
                     rpy(0) << ", " << rpy(1) << ", " << rpy(2) << ")";
                stmts.push_back(s.str());
            }

            if(storeFrc || storeAbsFrc) {

                s.str("");
                s << "insert into cart_mes_frc(cart_mes_id, cart_frc_x, cart_frc_y, cart_frc_z, cart_trq_x, cart_trq_y, cart_trq_z, cart_abs_frc) values (" <<
                     nextId << ", ";
                if(storeFrc)
                    s << frcTrq(0) << ", " << frcTrq(1) << ", " << frcTrq(2)<< ", " << frcTrq(3)<< ", " << frcTrq(4)<< ", " << frcTrq(5) << ", ";
                else
                    s << "null, " << "null, " << "null, " << "null, " << "null, " << "null" << ", ";
                if(storeAbsFrc)
                    s << absFrc;
                else
                    s << "null";
                s << ")";
                stmts.push_back(s.str());

            }

            dbStorage.executeStatements(stmts);

        }

        KUKADU_MODULE_END_USAGE();

    }

    void SensorStorage::storeJointInfoToDatabase(StorageSingleton& dbStorage, const int& robotId, const long long int& timeStamp, std::vector<int>& jointIds, arma::vec& jointPositions,
                                                 arma::vec& jointVelocities, arma::vec& jointAccelerations, bool storeForces, arma::vec& jointForces) {

        KUKADU_MODULE_START_USAGE();

        bool usePos = false;
        if(jointPositions.n_elem > 0)
            usePos = true;

        // if none is selected, don't store anything (not even timestamp)
        if(!storeForces && !usePos)
            return;

        vector<string> statements(jointIds.size());
        for(int i = 0; i < jointIds.size(); ++i) {

            stringstream s;
            s << "insert into joint_mes(robot_id, joint_id, time_stamp, position, velocity, acceleration, frc)" <<
                 " values(" << robotId << ", " << jointIds.at(i) << ", " << timeStamp << ", ";
            if(usePos)
                s << jointPositions(i) << ", " <<
                 jointVelocities(i) << ", " << jointAccelerations(i) << ", ";
            else
                s << "NULL, NULL, NULL, NULL, ";

            if(storeForces && i < jointForces.n_elem)
                s << jointForces(i) << ");";
            else
                s << "NULL);";

            statements.at(i) = s.str();

        }

        dbStorage.executeStatements(statements);

        KUKADU_MODULE_END_USAGE();

    }

    KUKADU_SHARED_PTR<SensorData> SensorStorage::readStorage(KUKADU_SHARED_PTR<ControlQueue> queue, std::string file) {

        KUKADU_MODULE_START_USAGE();

        vector<string> jointNames = queue->getJointNames();

        vector<string> jointPosLabels;
        vector<string> jointFrcLabels;
        vector<string> cartPosLabels;
        vector<string> cartFrcTrqLabels;
        vector<string> cartAbsForceLabels;

        for(int i = 0; i < jointNames.size(); ++i)
            jointPosLabels.push_back("joint_" + jointNames.at(i));

        for(int i = 0; i < jointNames.size(); ++i)
            jointFrcLabels.push_back("force_joint_" + jointNames.at(i));

        cartPosLabels.push_back("cart_pos_x");
        cartPosLabels.push_back("cart_pos_y");
        cartPosLabels.push_back("cart_pos_z");
        cartPosLabels.push_back("cart_quat_x");
        cartPosLabels.push_back("cart_quat_y");
        cartPosLabels.push_back("cart_quat_z");
        cartPosLabels.push_back("cart_quat_w");

        cartFrcTrqLabels.push_back("cart_force_x");
        cartFrcTrqLabels.push_back("cart_force_y");
        cartFrcTrqLabels.push_back("cart_force_z");
        cartFrcTrqLabels.push_back("cart_trq_x");
        cartFrcTrqLabels.push_back("cart_trq_y");
        cartFrcTrqLabels.push_back("cart_trq_z");

        cartAbsForceLabels.push_back("cart_abs_force");

        string line = "";
        string nextToken = "";
        ifstream inFile;
        inFile.open(file.c_str(), ios::in | ios::app | ios::binary);

        getline(inFile, line);
        KukaduTokenizer tok(line);
        bool foundDimension = false;
        int jointsPosStartIdx = -1;
        int jointsForceStartIdx = -1;
        int cartsPosStartIdx = -1;
        int cartsForceStartIdx = -1;
        int cartsAbsForceStartIdx = -1;
        int timeIdx = -1;

        // localizing single measurements in header
        while((nextToken = tok.next()) != "") {

            foundDimension = false;

            if(!nextToken.compare("time")) {

                foundDimension = true;
                timeIdx = tok.getTokenIdx();

                if(timeIdx != 0)
                    throw KukaduException("(SensorStorage) time must be the first column");

            } else {

                // if first joint token found, all of them have to be in consecutive order
                if(!nextToken.compare("joint_" + jointNames.at(0))) {

                    foundDimension = true;
                    jointsPosStartIdx = tok.getTokenIdx() - 1;

                    for(int i = 0; i < jointNames.size(); ++i) {
                        if(!nextToken.compare(jointPosLabels.at(i))) {

                            nextToken = tok.next();

                        } else {

                            throw KukaduException("(SensorStorage) joint names order not ok");

                        }

                    }

                    tok.putBackLast();

                }

            }

            // if none of the previous cases, go on...
            if(!foundDimension) {

                if(!nextToken.compare("cart_pos_x")) {
                    cartsPosStartIdx = tok.getTokenIdx() - 1;
                    foundDimension = true;

                    if(tok.next().compare("cart_pos_y") || tok.next().compare("cart_pos_z") ||
                            tok.next().compare("cart_quat_x") || tok.next().compare("cart_quat_y") || tok.next().compare("cart_quat_z") || tok.next().compare("cart_quat_w")) {
                        throw KukaduException("(SensorStorage) cartesian pos order not ok");
                    }

                    tok.putBackLast();

                }

            }

            // if none of the previous cases, go on...
            if(!foundDimension) {

                if(!nextToken.compare("cart_force_x")) {
                    cartsForceStartIdx = tok.getTokenIdx() - 1;
                    foundDimension = true;

                    if(tok.next().compare("cart_force_y") || tok.next().compare("cart_force_z") ||
                            tok.next().compare("cart_trq_x") || tok.next().compare("cart_trq_y") || tok.next().compare("cart_trq_z")) {
                        throw KukaduException("(SensorStorage) cartesian force order not ok");
                    }

                    tok.putBackLast();

                }

            }

            // if none of the previous cases, go on...
            if(!foundDimension) {

                // if first joint token found, all of them have to be in consecutive order
                if(!nextToken.compare(jointFrcLabels.at(0))) {

                    foundDimension = true;
                    jointsForceStartIdx = tok.getTokenIdx() - 1;

                    for(int i = 0; i < jointNames.size(); ++i) {
                        if(!nextToken.compare("force_joint_" + jointNames.at(i))) {

                            nextToken = tok.next();

                        } else {

                            throw KukaduException("(SensorStorage) joint force names order not ok");

                        }
                    }

                    tok.putBackLast();

                }

            }

            // if none of the previous cases, go on...
            if(!foundDimension) {

                // if first joint token found, all of them have to be in consecutive order
                if(!nextToken.compare(cartAbsForceLabels.at(0))) {

                    foundDimension = true;
                    cartsAbsForceStartIdx = tok.getTokenIdx() - 1;

                }

            }

        }

        // reading all present measurements
        auto mesData = readDmpData(inFile);
        if(mesData.second.n_rows <= 1)
            throw KukaduException("(SensorStorage) the provided file does not have the right format or contains to less data");

        // ignoring first line (something is wrong with first line)
        auto mes = mesData.second.rows(1, mesData.second.n_rows - 1);
        auto timeInMilliSeconds = mesData.first;
        timeInMilliSeconds.resize(timeInMilliSeconds.size() - 1);

        mat jointPos(1,1);
        mat jointFrcs(1,1);
        mat cartPos(1,1);
        mat cartFrcTrqs(1,1);
        mat cartAbsFrcs(1,1);

        if(jointsPosStartIdx >= 0)
            jointPos = mat(mes.cols(jointsPosStartIdx, jointsPosStartIdx + queue->getDegreesOfFreedom() - 1));

        if(jointsForceStartIdx >= 0)
            jointFrcs = mat(mes.cols(jointsForceStartIdx, jointsForceStartIdx + queue->getDegreesOfFreedom() - 1));

        if(cartsPosStartIdx >= 0)
            cartPos = mat(mes.cols(cartsPosStartIdx, cartsPosStartIdx + 7 - 1));

        if(cartsForceStartIdx >= 0)
            cartFrcTrqs = mat(mes.cols(cartsForceStartIdx, cartsForceStartIdx + 6 - 1));

        if(cartsAbsForceStartIdx >= 0)
            cartAbsFrcs = mat(mes.cols(cartsAbsForceStartIdx, cartsAbsForceStartIdx + 1 - 1));

        auto passedJointPosLabels = (jointsPosStartIdx >= 0) ? jointPosLabels : vector<string>();
        auto passedJointFrcLabels = (jointsForceStartIdx >= 0) ? jointFrcLabels : vector<string>();
        auto passedCartPosLabels = (cartsPosStartIdx >= 0) ? cartPosLabels : vector<string>();
        auto passedCartAbsFrcLabels = (cartsAbsForceStartIdx >= 0) ? cartAbsForceLabels : vector<string>();
        auto passedCartFrcLabels = (cartsForceStartIdx >= 0) ? cartFrcTrqLabels : vector<string>();

        KUKADU_SHARED_PTR<SensorData> dat = KUKADU_SHARED_PTR<SensorData>(new SensorData("time",
                                                                                         passedJointPosLabels,
                                                                                         passedJointFrcLabels,
                                                                                         passedCartPosLabels,
                                                                                         passedCartAbsFrcLabels,
                                                                                         passedCartFrcLabels,
                                                                                         timeInMilliSeconds, jointPos, jointFrcs, cartPos, cartAbsFrcs, cartFrcTrqs));

        inFile.close();

        KUKADU_MODULE_END_USAGE();

        return dat;

    }

}
