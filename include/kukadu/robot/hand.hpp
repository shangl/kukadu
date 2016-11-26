#ifndef KUKADU_HAND_H
#define KUKADU_HAND_H

#include <limits>
#include <vector>
#include <ros/ros.h>
#include <armadillo>
#include <kukadu/types/kukadutypes.hpp>
#include <kukadu/utils/destroyableobject.hpp>

namespace kukadu {

    enum kukadu_grasps {eGID_CENTRICAL, eGID_CYLINDRICAL, eGID_PARALLEL, eGID_SPHERICAL};

    /**
     * \class GenericHand
     *
     * \brief The GenericHand provides a very elementary interface to control robot hands mounted on a robot arm
     * This class provides very an interface for the very basic functionalities such as "connect to hand" or "close hand"
     * \ingroup Robot
     */
    class GenericHand : public DestroyableObject, public TimedObject {

    private:


    public:

        /** \brief Initializes the connection to the hand
         *
         */
        virtual void connectHand() = 0;

        /** \brief Opens and closes the hand according to the provided closing percentage and velocity
         * \param percentage closing percentage (0.0 - hand fully open, 1.0 hand fully closed)
         * \param velocity closing velocity in range between 0 and 1
         */
        virtual void closeHand(double percentage, double velocity) = 0;

        virtual void moveJoints(arma::vec joints) = 0;

        /** \brief Closes connection between host computer and hand
         *
         */
        virtual void disconnectHand() = 0;

        virtual std::vector<arma::mat> getTactileSensing() = 0;

        virtual std::string getHandName() = 0;

        virtual void setWaitForReached(bool waitForReached) = 0;

        virtual void setGrasp(kukadu_grasps grasp) = 0;

    };

    /**
     * \class PlottingHand
     *
     * \brief Provides control capabilities for the Schunk SDH robotic hand with ROS binding
     * Implements the GenericHand interface for the Schunk SDH robotic hand. Note that using this class the programm has to be executed with root rights
     * \ingroup Robot
     */
    class PlottingHand : public GenericHand {

    private:

        int sensingPatchCount;
        std::pair<int, int> patchDimensions;

    public:

        PlottingHand(int sensingPatchCount, std::pair<int, int> patchDimensions);

        virtual void connectHand();

        virtual void closeHand(double percentage, double velocity);

        virtual void moveJoints(arma::vec joints);

        virtual void disconnectHand();

        virtual std::vector<arma::mat> getTactileSensing();

        virtual std::string getHandName();

        virtual void safelyDestroy() {}

        virtual void setWaitForReached(bool waitForReached) { }

        virtual void setGrasp(kukadu_grasps grasp) { }

    };

}

#endif
