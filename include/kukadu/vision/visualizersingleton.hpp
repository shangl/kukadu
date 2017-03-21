#ifndef KUKADU_VISUALIZERSINGLETON_H
#define KUKADU_VISUALIZERSINGLETON_H

#include <mutex>
#include <string>
#include <vector>
#include <armadillo>
#include <geometry_msgs/Pose.h>
#include <kukadu/types/kukadutypes.hpp>
#include <pcl/visualization/pcl_visualizer.h>

namespace kukadu {

    class VisualizerSingleton {

    private:

        std::mutex windowMutex;

        pcl::visualization::PCLVisualizer vis;
        std::vector<std::string> visPointClouds;

        VisualizerSingleton();

    public:

        static VisualizerSingleton& get();

        void startWindow();
        void drawBox(std::string id, geometry_msgs::Pose pose, arma::vec dimensions);
        void showPointCloud(std::string id, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
        void drawLine(std::string id, double pointX1, double pointY1, double pointZ1, double pointX2, double pointY2, double pointZ2);

    };

}

#endif
