#include <string>
#include <fstream>
#include <tf/tf.h>
#include <iostream>
#include <geometry_msgs/Pose.h>
#include <kukadu/utils/utils.hpp>
#include <kukadu/robot/calibration.hpp>
#include <kukadu/utils/kukadutokenizer.hpp>

using namespace std;
using namespace arma;

namespace kukadu {

    KinectCalibrator::KinectCalibrator(KUKADU_SHARED_PTR<ControlQueue> queue, KUKADU_SHARED_PTR<Localizer> localizer, std::string attachedObjectId, bool storeDataToFile, std::string storePath,
                                       double minCartDistance, double minQuatDistance) {

        this->storeDataToFile = storeDataToFile;
        this->storePath = storePath;
        this->attachedObjectId = attachedObjectId;

        this->queue = queue;
        this->localizer = localizer;

        this->minCartDistance = minCartDistance;
        this->minQuatDistance = minQuatDistance;

        runDataCollectionThread = false;

        initializeForNewRun();

    }

    void KinectCalibrator::setReadDataFromRobot() {
        readFromFile = false;
    }

    void KinectCalibrator::setReadDataFromFile(std::string file) {
        if(inStream.is_open())
            inStream.close();
        inStream.open(file.c_str());
        readFromFile = true;
    }

    geometry_msgs::Pose KinectCalibrator::getPoseFromLine(const std::string& line) {
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

    geometry_msgs::Pose KinectCalibrator::getCurrentPoseRobot() {

        if(readFromFile) {

            string line;
            if(getline(inStream, line))
                lastObjectPoseInRobotFrame = getPoseFromLine(line);
            return lastObjectPoseInRobotFrame;

        }

        return (lastObjectPoseInRobotFrame = queue->getCurrentCartesianPose());

    }

    void KinectCalibrator::dataCollectionRunner() {

        geometry_msgs::Pose prevPoseCamera; prevPoseCamera.position.x = prevPoseCamera.position.y = prevPoseCamera.position.z =
                prevPoseCamera.orientation.x = prevPoseCamera.orientation.y = prevPoseCamera.orientation.z = prevPoseCamera.orientation.w = 0.0;
        auto prevPoseRobot = prevPoseCamera;

        // store every 4th packet
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
                        centroidCamera(0) += arPose.position.x;
                        centroidCamera(1) += arPose.position.y;
                        centroidCamera(2) += arPose.position.z;

                        centroidRobot(0) += robotPose.position.x;
                        centroidRobot(1) += robotPose.position.y;
                        centroidRobot(2) += robotPose.position.z;

                        samplesCamera.push_back(centroidCamera);
                        samplesRobot.push_back(centroidRobot);

                    dataMutex.unlock();

                }

            }

            if(!readFromFile)
                r.sleep();

        }

    }

    std::pair<bool, geometry_msgs::Pose> KinectCalibrator::getCurrentPoseCamera() {

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

    void KinectCalibrator::initializeForNewRun() {

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

    void KinectCalibrator::startDataCollection() {

        initializeForNewRun();

        runDataCollectionThread = true;
        collectionThread = kukadu_thread(&KinectCalibrator::dataCollectionRunner, this);

    }

    void KinectCalibrator::endDataCollection() {

        runDataCollectionThread = false;

        if(collectionThread.joinable())
            collectionThread.join();

        if(storeDataToFile)
            outFile.close();

    }

    KinectCalibrator::~KinectCalibrator() {
        endDataCollection();
    }

    bool KinectCalibrator::isSignificantlyDifferenct(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2) {

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

    std::pair<arma::mat, arma::vec> KinectCalibrator::calibrate() {

        dataMutex.lock();

            auto normalizedCameraCentroid = centroidCamera / (double) dataPointCount;
            auto normalizedRobotCentroid = centroidRobot / (double) dataPointCount;

            mat h(3, 3); h.fill(0.0);
            for(int i = 0; i < samplesCamera.size(); ++i)
                h += (samplesCamera.at(i) - normalizedCameraCentroid) * (samplesRobot.at(i) - normalizedRobotCentroid).t();

            mat u(3, 3); mat v(3, 3); vec s(3);
            svd(u, s, v, h);

            if(det(v))
                // if reflection case --> mirror the 3rd column
                for(int i = 0; i < v.n_rows; ++i)
                    v(2, i) = -v(2, i);

            mat r = v * u.t();
            vec t = - r * normalizedCameraCentroid + normalizedRobotCentroid;

        dataMutex.unlock();

        return {r, t};

    }

}
