#include <utility>
#include <pcl/common/pca.h>
#include <pcl/conversions.h>
#include <pcl/common/common.h>
#include <boost/shared_ptr.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <kukadu/vision/kinect.hpp>
#include <kukadu/vision/pcltools.hpp>
#include <kukadu/vision/pcstdtrans.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <kukadu/storage/moduleusagesingleton.hpp>
#include <pcl/filters/statistical_outlier_removal.h>

using namespace std;
using namespace pcl;
using namespace arma;

namespace kukadu {

	PCLTools::PCLTools() {
		isVisInit = false;
		keepShowingVis = false;
	}

	void PCLTools::visualizePointCloud(std::string id, pcl::PointCloud<pcl::PointXYZ>::Ptr pc) {
        KUKADU_MODULE_START_USAGE();
        visualizePointCloud(id, PCTransformator::fakeRgb(pc));
        KUKADU_MODULE_END_USAGE();
	}

    void PCLTools::visualizePointCloud(std::string id, pcl::PointCloud<pcl::PointXYZI>::Ptr pc) {
        KUKADU_MODULE_START_USAGE();
        visualizePointCloud(id, PCTransformator::removeIntensity(pc));
        KUKADU_MODULE_END_USAGE();
    }

    void PCLTools::visualizePointCloud(std::string id, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pc) {
        KUKADU_MODULE_START_USAGE();
        visualizePointCloud(id, pc);
        KUKADU_MODULE_END_USAGE();
    }

    void PCLTools::visualizePointCloud(std::string id, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc) {

        KUKADU_MODULE_START_USAGE();

        pair<string, PointCloud<PointXYZRGB>::Ptr> nextPcPair(id, pc);
        visPointClouds.push_back(nextPcPair);

        viewer->setBackgroundColor(0, 0, 0);
        for(int currIdx = 0; currIdx < visPointClouds.size(); ++currIdx) {
            pair<string, PointCloud<PointXYZRGB>::Ptr> currPcPair = visPointClouds.at(currIdx);
            viewer->addPointCloud<PointXYZRGB>(currPcPair.second, currPcPair.first);
            viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 1, currPcPair.first);
        }
        viewer->initCameraParameters();

        KUKADU_MODULE_END_USAGE();

    }

	void PCLTools::stopVisualizationWindow() {

        KUKADU_MODULE_START_USAGE();

		isVisInit = false;
		keepShowingVis = false;

        if(visThread && visThread->joinable())
            visThread->join();

		visPointClouds.clear();

        KUKADU_MODULE_END_USAGE();

	}

	void PCLTools::runVisThread() {

        KUKADU_MODULE_START_USAGE();

        viewer = make_shared<pcl::visualization::PCLVisualizer>("3D Viewer");
		viewer->setBackgroundColor(0, 0, 0);
		viewer->initCameraParameters();

		isVisInit = true;

		ros::Rate s(10);
		while(!viewer->wasStopped() && keepShowingVis) {
			viewer->spinOnce(100);
			s.sleep();
		}

		isVisInit = false;
		keepShowingVis = false;

        KUKADU_MODULE_END_USAGE();

	}

    KUKADU_SHARED_PTR<kukadu_thread> PCLTools::initializeVisualizationWindow() {

        KUKADU_MODULE_START_USAGE();

		keepShowingVis = true;

        visThread = make_shared<thread>(&PCLTools::runVisThread, this);
		ros::Rate s(10);
		while(!isVisInit)
			s.sleep();

        KUKADU_MODULE_END_USAGE();

		return visThread;

	}

	// according to http://www.pcl-users.org/Finding-oriented-bounding-box-of-a-cloud-td4024616.html
    FitCube PCLTools::fitBox(PointCloud<PointXYZRGB>::Ptr cloud) {

        KUKADU_MODULE_START_USAGE();

		FitCube retCube;
        PCA<PointXYZRGB> pca;
        PointCloud<PointXYZRGB> proj;

		pca.setInputCloud(cloud);
		pca.project(*cloud, proj);

        PointXYZRGB proj_min;
        PointXYZRGB proj_max;
		getMinMax3D(proj, proj_min, proj_max);

        PointXYZRGB min;
        PointXYZRGB max;
		pca.reconstruct(proj_min, min);
		pca.reconstruct(proj_max, max);

		// Rotation of PCA
		Eigen::Matrix3f rot_mat = pca.getEigenVectors();

		// Translation of PCA
		Eigen::Vector3f cl_translation = pca.getMean().head(3);

		Eigen::Matrix3f affine_trans;

		// Reordering of principal components
		affine_trans.col(2) << (rot_mat.col(0).cross(rot_mat.col(1))).normalized();
		affine_trans.col(0) << rot_mat.col(0);
		affine_trans.col(1) << rot_mat.col(1);

		retCube.rotation = Eigen::Quaternionf(affine_trans);
		Eigen::Vector4f t = pca.getMean();

		retCube.translation = Eigen::Vector3f(t.x(), t.y(), t.z());

		retCube.width = fabs(proj_max.x - proj_min.x);
		retCube.height = fabs(proj_max.y - proj_min.y);
		retCube.depth = fabs(proj_max.z - proj_min.z);

        KUKADU_MODULE_END_USAGE();

		return retCube;

	}

    PointCloud<PointXYZRGB>::Ptr PCLTools::segmentPlanar(PointCloud<PointXYZRGB>::Ptr cloud, bool negative) {

        KUKADU_MODULE_START_USAGE();

		// Create the segmentation object for the planar model and set all the parameters
        PointCloud<PointXYZRGB>::Ptr cloud_f(new PointCloud<PointXYZRGB>);
        SACSegmentation<PointXYZRGB> seg;
		PointIndices::Ptr inliers(new PointIndices);
		ModelCoefficients::Ptr coefficients(new ModelCoefficients);
        PointCloud<PointXYZRGB>::Ptr cloud_plane(new PointCloud<PointXYZRGB>());

		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setMaxIterations(100);
		seg.setDistanceThreshold(0.02);

		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud(cloud);
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.size () == 0) {
			cerr << "Could not estimate a planar model for the given dataset." << endl;
			return cloud_f;
		}

		// Extract the planar inliers from the input cloud
        ExtractIndices<PointXYZRGB> extract;
		extract.setInputCloud(cloud);
		extract.setIndices(inliers);
		extract.setNegative(false);

		// Get the points associated with the planar surface
		extract.filter(*cloud_plane);

		// Remove the planar inliers, extract the rest
		extract.setNegative(negative);
		extract.filter(*cloud_f);

        KUKADU_MODULE_END_USAGE();

		return cloud_f;

	}

	KUKADU_SHARED_PTR<pcl::visualization::PCLVisualizer> PCLTools::getVisualizer() {
		return viewer;
	}

	void PCLTools::visDrawPlaneWithNormal(std::string id, arma::vec r0, arma::vec n) {

        KUKADU_MODULE_START_USAGE();

		pcl::ModelCoefficients plane_coeff;
		plane_coeff.values.resize(4);
		plane_coeff.values[0] = n(0);
		plane_coeff.values[1] = n(1);
		plane_coeff.values[2] = n(2);
		plane_coeff.values[3] = -(n(0) * r0(0) + n(1) * r0(1) + n(2) * r0(2));
		viewer->addPlane(plane_coeff, r0(0), r0(1), r0(2), id);

		arma::vec newLineEndPoint(3);
		newLineEndPoint(0) = r0(0) + n(0);
		newLineEndPoint(1) = r0(1) + n(1);
		newLineEndPoint(2) = r0(2) + n(2);
        PointXYZRGB p1, p2;
		p1.data[0] = r0(0); p1.data[1] = r0(1); p1.data[2] = r0(2);
		p2.data[0] = newLineEndPoint(0); p2.data[1] = newLineEndPoint(1); p2.data[2] = newLineEndPoint(2);
		viewer->addLine(p1, p2, id + "line");

        KUKADU_MODULE_END_USAGE();

	}

    void PCLTools::visDrawLine(std::string id, double pointX1, double pointY1, double pointZ1, double pointX2, double pointY2, double pointZ2) {
        pcl::ModelCoefficients line_coeff;
        line_coeff.values.resize(6);
        line_coeff.values[0] = pointX1;
        line_coeff.values[1] = pointY1;
        line_coeff.values[2] = pointZ1;
        line_coeff.values[3] = pointX2 - pointX1;
        line_coeff.values[4] = pointY2 - pointY1;
        line_coeff.values[5] = pointZ2 - pointZ1;
        viewer->addLine(line_coeff, id);
    }

    void PCLTools::updateVisualizedPointCloud(std::string id, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc) {
        KUKADU_MODULE_START_USAGE();
		viewer->updatePointCloud(pc, id);
        KUKADU_MODULE_END_USAGE();
	}

    void PCLTools::updateVisualizedPointCloud(std::string id, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pc) {
        KUKADU_MODULE_START_USAGE();
        viewer->updatePointCloud(pc, id);
        KUKADU_MODULE_END_USAGE();
    }

    void PCLTools::updateVisualizedPointCloud(std::string id, pcl::PointCloud<pcl::PointXYZ>::Ptr pc) {
        KUKADU_MODULE_START_USAGE();
        viewer->updatePointCloud(pc, id);
        KUKADU_MODULE_END_USAGE();
    }

    void PCLTools::updateVisualizedPointCloud(std::string id, pcl::PointCloud<pcl::PointXYZI>::Ptr pc) {
        KUKADU_MODULE_START_USAGE();
        viewer->updatePointCloud(PCTransformator::removeIntensity(pc), id);
        KUKADU_MODULE_END_USAGE();
    }

	void PCLTools::visDrawBox(std::string id, struct FitCube dim) {
        viewer->removeShape(id);
		viewer->addCube(dim.translation, dim.rotation, dim.width, dim.height, dim.depth, id);
	}

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCLTools::filterCluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, bool negative) {

        KUKADU_MODULE_START_USAGE();

        // Create the filtering object
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK(50);
        sor.setStddevMulThresh (1.0);
        sor.filter(*cloud);

        int count = 0;
        vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusterPointers;
        for(pcl::PointCloud<pcl::PointXYZRGB>::iterator it = cloud->begin(); it != cloud->end(); it++, ++count) {

            if(!(count % 4)) {

                vec point(3);
                point(0) = it->x; point(1) = it->y; point(2) = it->z;

                bool foundPlace = false;
                for(int i = 0; i < clusterPointers.size() && !foundPlace; ++i) {

                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr currentCluster = clusterPointers.at(i);
                    for(pcl::PointCloud<pcl::PointXYZRGB>::iterator it2 = currentCluster->begin(); it2 != currentCluster->end(); it2++) {
                        vec clusterPoint(3);
                        clusterPoint(0) = it2->x; clusterPoint(1) = it2->y; clusterPoint(2) = it2->z;
                        vec difVec = point - clusterPoint;
                        vec distVec = difVec.t() * difVec;
                        double distance = distVec(0);
                        if(distance < pow(0.05, 2)) {
                            foundPlace = true;
                            currentCluster->push_back(*it);
                            break;
                        }
                    }

                }

                if(!foundPlace) {
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr nextCluster(new pcl::PointCloud<pcl::PointXYZRGB>());
                    nextCluster->push_back(*it);
                    clusterPointers.push_back(nextCluster);
                }

            }

        }

        int maxClusterIdx = 0;
        int maxClusterSize = 0;
        for(int i = 0; i < clusterPointers.size(); ++i) {
            int currentClusterSize = (clusterPointers.at(i))->size();
            if(currentClusterSize > maxClusterSize) {
                maxClusterIdx = i;
                maxClusterSize = currentClusterSize;
            }
        }

        KUKADU_MODULE_END_USAGE();

        return clusterPointers.at(maxClusterIdx);

    }

}
