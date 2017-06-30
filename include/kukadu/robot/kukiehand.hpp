#ifndef KUKADU_KUKIEHAND_H
#define KUKADU_KUKIEHAND_H

#ifndef USEBOOST
#include <limits>
#else
#include <climits>
#endif
#include <queue>
#include <kukadu/robot/hand.hpp>
#include <sensor_msgs/JointState.h>
#include <iis_robot_dep/TactileSensor.h>

namespace kukadu {

    /**
     * \class RosSchunk
     *
     * \brief Provides control capabilities for the Schunk SDH robotic hand with ROS binding
     * Implements the GenericHand interface for the Schunk SDH robotic hand. Note that using this class the programm has to be executed with root rights
     * \ingroup Robot
     */
    class KukieHand : public GenericHand {

    private:

        kukadu_grasps currentGraspId;

        bool waitForReached;
        bool stopCollecting;
        bool firstJointNamesRetrieval;

        int degOfFreedom;
        int previousCurrentPosQueueSize;

        ros::NodeHandle node;
        ros::Publisher trajPub;
        ros::Subscriber stateSub;
        ros::Subscriber tactileSub;

        std::vector<std::string> jointNames;
        std::vector<arma::mat> currentTactileReadings;

        std::string hand;

        std::vector<double> generateParallelPose(double percentage);
        std::vector<double> generateCentricalPose(double percentage);
        std::vector<double> generateSphericalPose(double percentage);
        std::vector<double> generateCylindricalPose(double percentage);

        bool targetReached;
        bool isFirstCallback;
        bool movementStarted;

        bool vectorsDeviate(const std::vector<double> v1, const std::vector<double> v2, double tolerance);
        std::vector<double> initCurrentPos;
        std::vector<double> currentPos;
        std::queue<std::vector<double> > previousCurrentPosQueue;

        kukadu_mutex currentPosMutex;
        kukadu_mutex tactileMutex;

        void stateCallback(const sensor_msgs::JointState& state);
        void tactileCallback(const iis_robot_dep::TactileSensor& state);

        void construct(std::string hand, bool simulation);

    protected:

        virtual void installHardwareInstanceInternal();

    public:

        KukieHand(StorageSingleton& storage, std::string robotName, bool simulation);
        KukieHand(StorageSingleton& storage, ros::NodeHandle node, bool simulation, std::string hand);
        ~KukieHand();

        virtual void connectHand();
        virtual void safelyDestroy();
        virtual void disconnectHand();
        virtual void setGrasp(kukadu_grasps grasp);
        virtual void publishSingleJoint(int idx, double pos);
        virtual void closeHand(double percentage, double velocity);

        void moveJoints(arma::vec joints);
        void setWaitForReached(bool waitForReached);

        virtual std::vector<std::string> getJointNames();

        virtual arma::vec getCurrentJoints();

        virtual std::vector<arma::mat> getTactileSensing();

#ifndef USEBOOST
        static auto constexpr SDH_IGNORE_JOINT = std::numeric_limits<double>::max();
#else
        static double const SDH_IGNORE_JOINT;
#endif

    };

}

#endif
