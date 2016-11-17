#ifndef KUKADU_SENSORSTORAGE_H
#define KUKADU_SENSORSTORAGE_H

#include <string>
#include <vector>
#include <cstdlib>
#include <iostream>
#include <armadillo>
#include <ros/ros.h>
#include <kukadu/utils/types.hpp>
#include <kukadu/utils/utils.hpp>
#include <kukadu/types/sensordata.hpp>
#include <kukadu/robot/gripper/generichand.hpp>
#include <kukadu/types/kukadutypes.hpp>
#include <kukadu/robot/arm/controlqueue.hpp>
#include <kukadu/vision/visioninterface.hpp>

namespace kukadu {

    /**
     * \class SensorStorage
     *
     * \brief Can collect and store sensor information of the robot represented
     * by a kukadu::ControlQueue and a kukadu::GenericHand.
     *
     * \ingroup Robot
     */
    class SensorStorage {

    private:

        bool stopped;
        bool storeTime;
        bool storeJntPos;
        bool storeJntFrc;
        bool storeCartPos;
        bool storeHndTctle;
        bool storageStopped;
        bool storeHndJntPos;
        bool storeCartFrcTrq;
        bool storeCartAbsFrc;

        double pollingFrequency;

        KUKADU_SHARED_PTR<kukadu_thread> thr;

        std::vector<KUKADU_SHARED_PTR<GenericHand> > hands;
        std::vector<KUKADU_SHARED_PTR<ControlQueue> > queues;

        std::vector<KUKADU_SHARED_PTR<std::ofstream> > handStreams;
        std::vector<KUKADU_SHARED_PTR<std::ofstream> > queueStreams;

        void store();
        void writeVectorInLine(KUKADU_SHARED_PTR<std::ofstream> stream, arma::vec writeVec);
        void writeMatrixInLine(KUKADU_SHARED_PTR<std::ofstream> stream, arma::mat writeMat);
        void writeLabels(KUKADU_SHARED_PTR<std::ofstream> stream, std::vector<std::string> labels);
        void writeMatrixMetaInfo(KUKADU_SHARED_PTR<std::ofstream> stream, int matrixNum, int xDim, int yDim);
        void initSensorStorage(std::vector<KUKADU_SHARED_PTR<ControlQueue> > queues, std::vector<KUKADU_SHARED_PTR<GenericHand> > hands, double pollingFrequency);

    public:

        SensorStorage(std::vector<KUKADU_SHARED_PTR<ControlQueue> > queues, std::vector<KUKADU_SHARED_PTR<GenericHand> > hands, double pollingFrequency);

        void stopDataStorage();
        void setExportMode(int mode);
        void storeData(bool storeHeader, std::string file, KUKADU_SHARED_PTR<SensorData> data);
        void storeData(bool storeHeader, std::vector<std::string> files, std::vector<KUKADU_SHARED_PTR<SensorData> > data);
        void storeData(bool storeHeader, std::vector<KUKADU_SHARED_PTR<std::ofstream> > queueStreams, std::vector<KUKADU_SHARED_PTR<SensorData> > data);

        KUKADU_SHARED_PTR<kukadu_thread> startDataStorage(std::string folderName);
        static KUKADU_SHARED_PTR<SensorData> readStorage(KUKADU_SHARED_PTR<ControlQueue> queue, std::string file);

        //static const int STORE_TIME = 512;
        static const int STORE_RBT_JNT_POS = 1;
        static const int STORE_RBT_CART_POS = 2;
        static const int STORE_RBT_JNT_FTRQ = 4;
        static const int STORE_RBT_CART_FTRQ = 8;
        static const int STORE_HND_JNT_POS = 16;
        static const int STORE_HND_TCTLE = 32;
        static const int STORE_CART_ABS_FRC = 64;
        static const int STORE_SIM_OBJECT = 128;
        static const int STORE_VIS_OBJECT = 256;

    };

}

#endif
