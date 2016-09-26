#ifndef KinestheticTeacher_HPP
#define KinestheticTeacher_HPP


#include <mutex>
#include <std_msgs/Float64MultiArray.h>
#include <cmath>
#include <eigen_conversions/eigen_msg.h>
#include <iostream>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <Eigen/Dense>
#include "kukadu/robot/sensors/AutoCompensatingFilter.hpp"
#include "kukadu/robot/arm/kukiecontrolqueue.hpp"

namespace kukadu
{

class KinestheticTeacher{

public:

    virtual void init()=0;
    virtual void startTeaching()=0;
    virtual void stopTeaching()=0;
    virtual void startRecording()=0;
    virtual void stopRecording()=0;
    virtual void quit()=0;


};
}

#endif
