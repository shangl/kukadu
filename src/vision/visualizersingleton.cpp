#include <future>
#include <thread>
#include <ros/ros.h>
#include <kukadu/vision/visualizersingleton.hpp>

namespace kukadu {

    VisualizerSingleton::VisualizerSingleton() {

    }

    VisualizerSingleton& VisualizerSingleton::get() {
        static VisualizerSingleton instance;
        return instance;
    }

    void VisualizerSingleton::startWindow() {

        static auto as = std::async(std::launch::async, [this] {
                ros::Rate r(10);
                while(true) {
                    windowMutex.lock();
                    vis.spinOnce();
                    windowMutex.unlock();
                    r.sleep();
                }
                return 0;
            });

        vis.initCameraParameters();

    }

    void VisualizerSingleton::drawBox(std::string id, geometry_msgs::Pose pose, arma::vec dimensions) {
        vis.removeShape(id);
        Eigen::Vector3f trans; trans << pose.position.x, pose.position.y, pose.position.z;
        Eigen::Quaternionf rot(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
        vis.addCube(trans, rot, dimensions(0), dimensions(1), dimensions(2), id);
    }

    void VisualizerSingleton::showPointCloud(std::string id, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {

        windowMutex.lock();

        if(!visPointClouds.size())
            vis.setBackgroundColor(0, 0, 0);

        // if point cloud is not there yet, insert it
        if(std::find(visPointClouds.begin(), visPointClouds.end(), id) == visPointClouds.end()) {

            vis.addPointCloud<pcl::PointXYZRGB>(cloud, id);
            vis.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id);

            visPointClouds.push_back(id);

        }
        // else update it
        else {

            vis.updatePointCloud(cloud, id);

        }

        windowMutex.unlock();

    }

}
