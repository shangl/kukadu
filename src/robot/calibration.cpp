#include <string>
#include <fstream>
#include <tf/tf.h>
#include <iostream>
#include <geometry_msgs/Pose.h>
#include <kukadu/utils/utils.hpp>
#include <kukadu/robot/calibration.hpp>
#include <kukadu/utils/kukadutokenizer.hpp>
#include <kukadu/storage/moduleusagesingleton.hpp>

using namespace std;
using namespace arma;

namespace kukadu {

    Calibrator::Calibrator(std::string originalFrame, std::string targetFrame) {
        this->originalFrame = originalFrame;
        this->targetFrame = targetFrame;
    }

    std::string Calibrator::getOriginalFrame() {
        return originalFrame;
    }

    std::string Calibrator::getTargetFrame() {
        return targetFrame;
    }

    tf::Transform Calibrator::calibrateTfTransform() {

        KUKADU_MODULE_START_USAGE();

        tf::Transform retTf = affineTransMatrixToTf(calibrateAffineTransMatrix());

        KUKADU_MODULE_END_USAGE();

        return retTf;

    }

    arma::mat Calibrator::calibrateAffineTransMatrix() {

        KUKADU_MODULE_START_USAGE();

        pair<mat, vec> calibration = calibrate();

        mat retMat(4, 4); retMat.fill(0.0);
        for(int i = 0; i < 3; ++i) {
            retMat(i, 3) = calibration.second(i);
            for(int j = 0; j < 3; ++j)
                retMat(i, j) = calibration.first(i, j);
        }
        retMat(3, 3) = 1.0;

        KUKADU_MODULE_END_USAGE();

        return retMat;

    }

    CameraCalibrator::CameraCalibrator(KUKADU_SHARED_PTR<ControlQueue> queue, KUKADU_SHARED_PTR<Localizer> localizer, std::string attachedObjectId, bool storeDataToFile, std::string storePath,
                                       double minCartDistance, double minQuatDistance)
     : Calibrator((localizer) ? localizer->getLocalizerFrame() : "camera_frame",
                  (queue) ? queue->getCartesianReferenceFrame() : "robot_frame") {

        this->storeDataToFile = storeDataToFile;
        this->storePath = storePath;
        this->attachedObjectId = attachedObjectId;

        this->queue = queue;
        this->localizer = localizer;

        this->minCartDistance = minCartDistance;
        this->minQuatDistance = minQuatDistance;

        runDataCollectionThread = false;

        initializeForNewRun();
        setReadDataFromRobot();

    }

    void CameraCalibrator::setReadDataFromRobot() {
        readFromFile = false;
    }

    void CameraCalibrator::setReadDataFromFile(std::string file) {

        KUKADU_MODULE_START_USAGE();

        if(inStream.is_open())
            inStream.close();
        inStream.open(file.c_str());
        readFromFile = true;

        KUKADU_MODULE_END_USAGE();

    }

    geometry_msgs::Pose CameraCalibrator::getPoseFromLine(const std::string& line) {
        geometry_msgs::Pose retPose;
        KukaduTokenizer tok(line, "(,) ");
        auto words = tok.split();
        retPose.position.x = atof(words.at(0).c_str());
        retPose.position.y = atof(words.at(1).c_str());
        retPose.position.z = atof(words.at(2).c_str());
        retPose.orientation.x = atof(words.at(3).c_str());
        retPose.orientation.y = atof(words.at(4).c_str());
        retPose.orientation.z = atof(words.at(5).c_str());
        retPose.orientation.w = atof(words.at(6).c_str());
        return retPose;
    }

    geometry_msgs::Pose CameraCalibrator::getCurrentPoseRobot() {

        if(readFromFile) {

            string line;
            if(getline(inStream, line))
                lastObjectPoseInRobotFrame = getPoseFromLine(line);
            return lastObjectPoseInRobotFrame;

        }

        lastObjectPoseInRobotFrame = queue->getCurrentCartesianPose();
        return lastObjectPoseInRobotFrame;

    }

    std::pair<bool, geometry_msgs::Pose> CameraCalibrator::getCurrentPoseCamera() {

        if(readFromFile) {

            string line;
            if(getline(inStream, line))
                lastObjectPoseInCamFrame = getPoseFromLine(line);
            return {true, lastObjectPoseInCamFrame};

        } else {

            auto poses = localizer->localizeObjects();
            if(poses.find(attachedObjectId) != poses.end())
                return {true, (lastObjectPoseInCamFrame = poses[attachedObjectId])};

        }

        return {false, geometry_msgs::Pose()};

    }

    void CameraCalibrator::dataCollectionRunner() {

        geometry_msgs::Pose prevPoseCamera; prevPoseCamera.position.x = prevPoseCamera.position.y = prevPoseCamera.position.z =
                prevPoseCamera.orientation.x = prevPoseCamera.orientation.y = prevPoseCamera.orientation.z = prevPoseCamera.orientation.w = 0.0;
        auto prevPoseRobot = prevPoseCamera;

        // store every 4th packet
        ros::Rate r(20);
        if(!readFromFile)
            ros::Rate r(4.0 * 1.0 / queue->getCycleTime());

        if(storeDataToFile)
            outFile.open(storePath.c_str());

        while(runDataCollectionThread) {

            auto cameraPoseRes = getCurrentPoseCamera();
            auto robotPose = getCurrentPoseRobot();

            bool localizedObject = cameraPoseRes.first;

            // if the object was localized at all
            if(localizedObject) {

                geometry_msgs::Pose& arPose = cameraPoseRes.second;

                // and the object was moved enough (only consider the much more precise robot kinematics)
                if(isSignificantlyDifferenct(prevPoseRobot, robotPose)) {

                    prevPoseCamera = arPose;
                    prevPoseRobot = robotPose;

                    // use the data point for estimation of the transformation matrix

                    if(storeDataToFile) {

                        outFile << "(" << arPose.position.x << ", " << arPose.position.y << ", " << arPose.position.z << ") " <<
                                "(" << arPose.orientation.x << ", " << arPose.orientation.y << ", " << arPose.orientation.z << ", " << arPose.orientation.w << ")" << endl;

                        outFile << "(" << robotPose.position.x << ", " << robotPose.position.y << ", " << robotPose.position.z << ") " <<
                                "(" << robotPose.orientation.x << ", " << robotPose.orientation.y << ", " << robotPose.orientation.z << ", " << robotPose.orientation.w << ")" << endl;


                        outFile.flush();

                    }

                    dataMutex.lock();

                        ++dataPointCount;
                        vec cameraPoint(3);
                        cameraPoint(0) = arPose.position.x;
                        cameraPoint(1) = arPose.position.y;
                        cameraPoint(2) = arPose.position.z;

                        vec robotPoint(3);
                        robotPoint(0) = robotPose.position.x;
                        robotPoint(1) = robotPose.position.y;
                        robotPoint(2) = robotPose.position.z;

                        samplesCamera.push_back(cameraPoint);
                        samplesRobot.push_back(robotPoint);

                        centroidCamera += cameraPoint;
                        centroidRobot += robotPoint;

                    dataMutex.unlock();

                }

            }

            if(!readFromFile)
                r.sleep();

        }

    }

    void CameraCalibrator::initializeForNewRun() {

        if(outFile.is_open())
            outFile.close();

        runDataCollectionThread = false;

        lastObjectPoseInCamFrame = geometry_msgs::Pose();
        lastObjectPoseInRobotFrame = geometry_msgs::Pose();

        dataMutex.lock();

            dataPointCount = 0;
            centroidRobot = vec(3);
            centroidCamera = vec(3);

            centroidRobot.fill(0.0);
            centroidCamera.fill(0.0);

            samplesCamera.clear();
            samplesRobot.clear();

        dataMutex.unlock();

    }

    void CameraCalibrator::startDataCollection() {

        KUKADU_MODULE_START_USAGE();

        if(!readFromFile && !queue)
            throw KukaduException("(CameraCalibrator) you must either read the data from a file or provide a valid control queue");

        if(!readFromFile && !localizer)
            throw KukaduException("(CameraCalibrator) you must either read the data from a file or provide a valid localizer");

        initializeForNewRun();

        runDataCollectionThread = true;
        collectionThread = kukadu_thread(&CameraCalibrator::dataCollectionRunner, this);

        KUKADU_MODULE_END_USAGE();

    }

    void CameraCalibrator::endDataCollection() {

        KUKADU_MODULE_START_USAGE();

        runDataCollectionThread = false;

        if(collectionThread.joinable())
            collectionThread.join();

        if(storeDataToFile)
            outFile.close();

        KUKADU_MODULE_END_USAGE();

    }

    CameraCalibrator::~CameraCalibrator() {
        endDataCollection();
    }

    bool CameraCalibrator::isSignificantlyDifferenct(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2) {

        vec p1Pos(3); p1Pos(0) = p1.position.x; p1Pos(1) = p1.position.y; p1Pos(2) = p1.position.z;
        vec p2Pos(3); p2Pos(0) = p2.position.x; p2Pos(1) = p2.position.y; p2Pos(2) = p2.position.z;

        auto diffVec = p1Pos - p2Pos;
        vec distVec = diffVec.t() * diffVec;
        double cartDistance = sqrt(distVec(0));

        if(cartDistance > minCartDistance)
            return true;

        tf::Quaternion p1Quat(p1.orientation.x, p1.orientation.y, p1.orientation.z, p1.orientation.w);
        tf::Quaternion p2Quat(p2.orientation.x, p2.orientation.y, p2.orientation.z, p2.orientation.w);

        double rotDistance = distQuat(p1Quat, p2Quat);

        if(rotDistance > minQuatDistance)
            return true;

        return false;

    }

    std::pair<arma::mat, arma::vec> CameraCalibrator::calibrate() {

        KUKADU_MODULE_START_USAGE();

        dataMutex.lock();

            auto normalizedCameraCentroid = centroidCamera / (double) dataPointCount;
            auto normalizedRobotCentroid = centroidRobot / (double) dataPointCount;

            mat h(3, 3); h.fill(0.0);
            for(int i = 0; i < samplesCamera.size(); ++i)
                h += (samplesCamera.at(i) - normalizedCameraCentroid) * (samplesRobot.at(i) - normalizedRobotCentroid).t();

            mat u(3, 3); mat v(3, 3); vec s(3);
            svd(u, s, v, h);

            if(det(v) < 0.0) {

                // if reflection case --> mirror the 3rd column
                for(int i = 0; i < v.n_rows; ++i)
                    v(2, i) = -v(2, i);

            }

            mat r = v * u.t();
            vec t = -r * normalizedCameraCentroid + normalizedRobotCentroid;

        dataMutex.unlock();

        KUKADU_MODULE_END_USAGE();

        return {r, t};

    }

}
