#include <kukadu/vision/visioninterface.hpp>

#include<iostream>
#include<string>

using namespace std;

namespace kukadu {

    VisionInterface::VisionInterface(int sleepTime, ros::NodeHandle node)  {

        // (@senka) arTagTracker was not initialized in the original code, so i initialize it with false (dont know if this is the intended behaviour)
        this->construct(sleepTime, node, "", false);

    }

    VisionInterface::VisionInterface(int sleepTime, std::string cameraTag, ros::NodeHandle node){

        this->construct(sleepTime, node, cameraTag, false);

    }

    void VisionInterface::setArTagTracker(){

        this->arTagTracker = true;
        this->subArTag = node.subscribe(arTagTopic, 1, &VisionInterface::arTagCallback, this);

        this->firstSet = false;
        ros::Rate lRate(sleepTime);
        while(!firstSet){
            ros::spinOnce();
            lRate.sleep();
        }


    }

    void VisionInterface::construct(int sleepTime, ros::NodeHandle node, std::string cameraTag, bool arTagTracker){

        this->node = node;
        this->sleepTime = sleepTime;

        arTagTopic = "arMarker/tf";
        Eigen::Matrix4f Tm;
        Tm <<   // use inverse of octave
                -6.3153e-02, -1.0343e+00, -1.8485e-02,  7.2820e-01,
                -6.9606e-01,  5.6802e-02, -6.8370e-01,  1.5751e-01,
                5.6291e-01 , 4.3940e-02 ,-7.0540e-01 , 6.5800e-01,
                -2.8443e-15,  1.7948e-15,  2.7121e-15,  1.0000e+00;

        tfChestKin = Matrix4f2Transform(Tm);


    }

    void VisionInterface::arTagCallback(const tf::tfMessage& msg){

        firstSet = true;

        geometry_msgs::TransformStamped t = msg.transforms.at(0);
        tf::Vector3 origin;
        origin.setValue(t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);
        tf::Quaternion rotation (t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w);

        tf::Transform pose;
        pose.setOrigin(origin);
        pose.setRotation(rotation);

        pose = tfChestKin.inverse() * pose;

        currentArPose.position.x = pose.getOrigin().getX();
        currentArPose.position.y = pose.getOrigin().getY();
        currentArPose.position.z = pose.getOrigin().getZ();
        currentArPose.orientation.x = pose.getRotation().getX();
        currentArPose.orientation.y = pose.getRotation().getY();
        currentArPose.orientation.z = pose.getRotation().getZ();
        currentArPose.orientation.w = pose.getRotation().getW();

    }

     geometry_msgs::Pose VisionInterface::getArPose(){
         return currentArPose;

     }

}
