#ifndef KUKADU_PCLTOOLS_H
#define KUKADU_PCLTOOLS_H

#include <vector>
#include <armadillo>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <kukadu/types/kukadutypes.hpp>
#include <pcl/visualization/pcl_visualizer.h>

namespace kukadu {

    struct FitCube {
        Eigen::Vector3f translation;
        Eigen::Quaternionf rotation;
        double width, height, depth;
    };

    class PCLTools {

    private:

        bool isVisInit;
        bool keepShowingVis;

        KUKADU_SHARED_PTR<kukadu_thread> visThread;
        KUKADU_SHARED_PTR<pcl::visualization::PCLVisualizer> viewer;

        std::vector<std::pair<std::string, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> > visPointClouds;

        void runVisThread();

    public:

        PCLTools();

        static FitCube fitBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
        static pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmentPlanar(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, bool negative);
        static pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterCluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, bool negative);

        void stopVisualizationWindow();
        void visDrawBox(std::string id, struct FitCube dim);
        void visDrawPlaneWithNormal(std::string id, arma::vec r0, arma::vec n);

        void visualizePointCloud(std::string id, pcl::PointCloud<pcl::PointXYZ>::Ptr pc);
        void visualizePointCloud(std::string id, pcl::PointCloud<pcl::PointXYZI>::Ptr pc);
        void visualizePointCloud(std::string id, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc);
        void visualizePointCloud(std::string id, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pc);

        void updateVisualizedPointCloud(std::string id, pcl::PointCloud<pcl::PointXYZ>::Ptr pc);
        void updateVisualizedPointCloud(std::string id, pcl::PointCloud<pcl::PointXYZI>::Ptr pc);
        void updateVisualizedPointCloud(std::string id, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc);
        void updateVisualizedPointCloud(std::string id, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pc);

        KUKADU_SHARED_PTR<kukadu_thread> initializeVisualizationWindow();
        KUKADU_SHARED_PTR<pcl::visualization::PCLVisualizer> getVisualizer();

    };

}

#endif // PCLTOOLS_H
