#ifndef KUKADU_SENSORSTORAGE_H
#define KUKADU_SENSORSTORAGE_H

#include <string>
#include <vector>
#include <cstdlib>
#include <iostream>
#include <armadillo>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <kukadu/robot/hand.hpp>
#include <kukadu/robot/queue.hpp>
#include <kukadu/utils/types.hpp>
#include <kukadu/utils/utils.hpp>
#include <kukadu/types/sensordata.hpp>
#include <kukadu/types/kukadutypes.hpp>
#include <kukadu/storage/storagesingleton.hpp>

namespace kukadu {

    /**
     * \class SensorStorage
     *
     * \brief Can collect and store sensor information of the robot represented
     * by a kukadu::ControlQueue and a kukadu::GenericHand.
     *
     * \ingroup Robot
     */
    class SensorStorage : public TimedObject {

    private:

        bool stopped;
        bool storeTime;
        bool storeJntPos;
        bool storeJntFrc;
        bool storeCartPos;
        bool storeHndTctle;
        bool storeHndJntPos;
        bool storeCartFrcTrq;
        bool storeCartAbsFrc;

        double pollingFrequency;

        StorageSingleton& dbStorage;

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

        /*
         * \brief: Stores gathered data. If the SensorData data is null, the data is collected live. Otherwise the passed data is written. The write destination
         * is a file if the string file is not empty. Otherwise, data is stored to the database
         */
        void storeData(bool storeHeader, KUKADU_SHARED_PTR<SensorData> data, std::string file = "");
        void storeData(bool storeHeader, std::vector<KUKADU_SHARED_PTR<SensorData> > data, std::vector<std::string> files);
        void storeData(bool storeHeader, std::vector<KUKADU_SHARED_PTR<SensorData> > data, std::vector<KUKADU_SHARED_PTR<std::ofstream> > queueStreams);

    public:

        SensorStorage(StorageSingleton& storage, std::vector<KUKADU_SHARED_PTR<ControlQueue> > queues, std::vector<KUKADU_SHARED_PTR<GenericHand> > hands, double pollingFrequency);

        long long stopDataStorage();
        void setExportMode(int mode);

        /*
         * \brief: Starts the storage of data. If a parameter is passed, the data is stored to a file. Otherwise it is stored to the database
         */
        long long int startDataStorage(std::string folderName = "");
        static KUKADU_SHARED_PTR<SensorData> readStorage(KUKADU_SHARED_PTR<ControlQueue> queue, std::string file);

        static const int STORE_RBT_JNT_POS = 1;
        static const int STORE_RBT_CART_POS = 2;
        static const int STORE_RBT_JNT_FTRQ = 4;
        static const int STORE_RBT_CART_FTRQ = 8;
        static const int STORE_HND_JNT_POS = 16;
        static const int STORE_HND_TCTLE = 32;
        static const int STORE_CART_ABS_FRC = 64;
        static const int STORE_SIM_OBJECT = 128;
        static const int STORE_VIS_OBJECT = 256;

        // transfers joint information from a file to the database; returns the start and end timestamp of the stored trajectory
        static std::pair<long long int, long long int> transferArmDataToDb(StorageSingleton& dbStorage, KUKADU_SHARED_PTR<ControlQueue> queue, std::string file);

        static arma::vec loadSampleTimesInRangeFromDb(StorageSingleton& storage, KUKADU_SHARED_PTR<ControlQueue> queue, long long int startTime, long long int endTime);
        static arma::mat loadJointsFromDb(StorageSingleton& storage, KUKADU_SHARED_PTR<ControlQueue> queue, long long int startTime, long long int endTime);

        static void storeCartInformation(StorageSingleton& dbStorage, const int& robotId, const long long int& timeStamp, const std::string& referenceFrame, const std::string& linkName, geometry_msgs::Pose& cartesianPose,
                                  const arma::vec& frcTrq, const double& absFrc,
                                  const bool& storePos, const bool& storeFrc, const bool& storeAbsFrc);
        static void storeJointInfoToDatabase(StorageSingleton& dbStorage, const int& robotId, const long long int& timeStamp,
                                             std::vector<int>& jointIds, arma::vec& jointPositions, arma::vec& jointVelocities, arma::vec& jointAccelerations, bool storeForces,
                                             arma::vec& jointForces);

    };

}

#endif
