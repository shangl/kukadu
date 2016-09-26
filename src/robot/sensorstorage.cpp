#include <kukadu/robot/sensorstorage.hpp>
#include <kukadu/types/sensordata.hpp>

using namespace std;
using namespace arma;

namespace kukadu {

    void SensorStorage::initSensorStorage(std::vector<KUKADU_SHARED_PTR<ControlQueue> > queues, std::vector<KUKADU_SHARED_PTR<GenericHand> > hands, double pollingFrequency) {

        this->hands = hands;
        this->queues = queues;
        this->pollingFrequency = pollingFrequency;

        stopped = false;
        storageStopped = true;

        storeCartAbsFrc = false;
        storeTime = storeJntPos = storeCartPos = storeJntFrc = storeCartFrcTrq = storeHndJntPos = storeHndTctle = true;

    }

    SensorStorage::SensorStorage(std::vector<KUKADU_SHARED_PTR<ControlQueue> > queues, std::vector<KUKADU_SHARED_PTR<GenericHand> > hands, double pollingFrequency) {

        initSensorStorage(queues, hands, pollingFrequency);

    }

    void SensorStorage::setExportMode(int mode) {

        storeCartAbsFrc = storeTime = storeJntPos = storeCartPos = storeJntFrc = storeCartFrcTrq = storeHndJntPos = storeHndTctle = false;

        if(mode & STORE_TIME)
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

    }

    KUKADU_SHARED_PTR<kukadu_thread> SensorStorage::startDataStorage(std::string folderName) {

        if(createDirectory(folderName)) {

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

            thr = KUKADU_SHARED_PTR<kukadu_thread>();
            thr = KUKADU_SHARED_PTR<kukadu_thread>(new kukadu_thread(&SensorStorage::store, this));
            storageStopped = false;
            return thr;

        } else {

            return KUKADU_SHARED_PTR<kukadu_thread>();

        }

    }

    void SensorStorage::stopDataStorage() {

        stopped = true;

        ros::Rate r(40);
        while(!storageStopped)
            r.sleep();

        for(int i = 0; i < queueStreams.size(); ++i)
            queueStreams.at(i)->close();
        queueStreams.clear();

        for(int i = 0; i < handStreams.size(); ++i)
            handStreams.at(i)->close();
        handStreams.clear();

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

        storeData(true, queueStreams, std::vector<KUKADU_SHARED_PTR<SensorData> >());

    }

    void SensorStorage::storeData(bool storeHeader, std::string file, KUKADU_SHARED_PTR<SensorData> data) {

        vector<KUKADU_SHARED_PTR<SensorData> > dataVec;
        vector<string> filesVec;

        dataVec.push_back(data);
        filesVec.push_back(file);

        storeData(storeHeader, filesVec, dataVec);

    }

    void SensorStorage::storeData(bool storeHeader, std::vector<std::string> files, std::vector<KUKADU_SHARED_PTR<SensorData> > data) {

        std::vector<KUKADU_SHARED_PTR<ofstream> > queueStreams;
        for(int i = 0; i < files.size(); ++i) {
            string file = files.at(i);
            KUKADU_SHARED_PTR<ofstream> currentStream = KUKADU_SHARED_PTR<ofstream>(new ofstream());
            currentStream->open(file.c_str());
            queueStreams.push_back(currentStream);
        }
        storeData(storeHeader, queueStreams, data);

    }

    void SensorStorage::storeData(bool storeHeader, std::vector<KUKADU_SHARED_PTR<std::ofstream> > queueStreams, std::vector<KUKADU_SHARED_PTR<SensorData> > data) {

        stopped = false;
        double waitTime = 1.0 / pollingFrequency;
        ros::Rate rate(pollingFrequency);
        double currentTime = 0.0;

        bool firstTime = true;
        int iterationSize = (data.size()) ? data.size() : queues.size();

        double time = 0.0;

        for(int dataPointIdx = 0; !stopped && (!data.size() || dataPointIdx < data.at(0)->getTimes().n_elem); ++dataPointIdx) {

            for(int i = 0; i < iterationSize; ++i) {

                KUKADU_SHARED_PTR<ControlQueue> currentQueue = queues.at(i);
                KUKADU_SHARED_PTR<ofstream> currentOfStream = queueStreams.at(i);

                mes_result joints;
                mes_result cartPos;
                mes_result jntFrcTrq;
                mes_result cartFrcTrq;
                mes_result cartAbsFrcTrq;
                double absCartFrc = 0.0;

                // if not collect data live
                if(data.size()) {
                    time = data.at(i)->getTimes()(dataPointIdx);

                    if(storeJntPos) {
                        joints.time = time;
                        joints.joints = data.at(i)->getJointPosRow(dataPointIdx);
                    }

                    if(storeCartPos) {
                        cartPos.time = time;
                        cartPos.joints = data.at(i)->getCartPosRow(dataPointIdx);
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

                    if(storeCartPos)
                        joints = currentQueue->getCurrentJoints();

                    if(storeCartPos)
                        cartPos = currentQueue->getCurrentCartesianPos();

                    if(storeJntFrc)
                        jntFrcTrq = currentQueue->getCurrentJntFrc();

                    if(storeCartFrcTrq)
                        cartFrcTrq = currentQueue->getCurrentCartesianFrcTrq();

                    if(storeCartAbsFrc)
                        absCartFrc = currentQueue->getAbsoluteCartForce();

                    time = joints.time;

                }

                if(firstTime && storeHeader) {

                    vector<string> jointNames = currentQueue->getJointNames();
                    vector<string> labels;

                    if(storeTime)
                        labels.push_back("time");

                    if(storeJntPos) {

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

                    writeLabels(currentOfStream, labels);

                }

                if(storeTime)
                    *currentOfStream << time << "\t";

                if(storeJntPos) {
                    writeVectorInLine(currentOfStream, joints.joints);
                }

                if(storeCartPos) {
                    writeVectorInLine(currentOfStream, cartPos.joints);
                }

                if(storeJntFrc) {
                    writeVectorInLine(currentOfStream, jntFrcTrq.joints);
                }

                if(storeCartFrcTrq) {
                    writeVectorInLine(currentOfStream, cartFrcTrq.joints);
                }

                if(storeCartAbsFrc) {
                    vec absForce(1);
                    absForce(0) = absCartFrc;
                    writeVectorInLine(currentOfStream, absForce);
                }

                *currentOfStream << endl;
                currentTime = time;

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

        storageStopped = true;

    }

    KUKADU_SHARED_PTR<SensorData> SensorStorage::readStorage(KUKADU_SHARED_PTR<ControlQueue> queue, std::string file) {

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

            } else {

                // if first joint token found, all of them have to be in consecutive order
                if(!nextToken.compare("joint_" + jointNames.at(0))) {

                    foundDimension = true;
                    jointsPosStartIdx = tok.getTokenIdx();

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
                    cartsPosStartIdx = tok.getTokenIdx();
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
                    cartsForceStartIdx = tok.getTokenIdx();
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
                    jointsForceStartIdx = tok.getTokenIdx();

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
                    cartsAbsForceStartIdx = tok.getTokenIdx();

                }

            }

        }

        // reading all present measurements
        mat mes = readMovements(inFile);
        // ignoring first line (something is wrong with first line)
        mes = mes.rows(1, mes.n_rows - 1);

        vec times(1);
        mat jointPos(1,1);
        mat jointFrcs(1,1);
        mat cartPos(1,1);
        mat cartFrcTrqs(1,1);
        mat cartAbsFrcs(1,1);

        if(timeIdx >= 0)
            times = vec(mes.col(timeIdx));

        if(jointsPosStartIdx >= 0)
            jointPos = mat(mes.cols(jointsPosStartIdx, jointsPosStartIdx + queue->getMovementDegreesOfFreedom() - 1));

        if(jointsForceStartIdx >= 0)
            jointFrcs = mat(mes.cols(jointsForceStartIdx, jointsForceStartIdx + queue->getMovementDegreesOfFreedom() - 1));

        if(cartsPosStartIdx >= 0)
            cartPos = mat(mes.cols(cartsPosStartIdx, cartsPosStartIdx + 7 - 1));

        if(cartsForceStartIdx >= 0)
            cartFrcTrqs = mat(mes.cols(cartsForceStartIdx, cartsForceStartIdx + 6 - 1));

        if(cartsAbsForceStartIdx >= 0)
            cartAbsFrcs = mat(mes.cols(cartsAbsForceStartIdx, cartsAbsForceStartIdx + 1 - 1));

        KUKADU_SHARED_PTR<SensorData> dat = KUKADU_SHARED_PTR<SensorData>(new SensorData("time", jointPosLabels, jointFrcLabels, cartPosLabels, cartAbsForceLabels, cartFrcTrqLabels,
                                                                           times, jointPos, jointFrcs, cartPos, cartAbsFrcs, cartFrcTrqs));

        inFile.close();
        return dat;

    }

}
