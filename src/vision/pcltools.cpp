#include <kukadu/vision/pcltools.hpp>

using namespace std;
using namespace pcl;

namespace kukadu {

	PCLTools::PCLTools() {
		isVisInit = false;
		keepShowingVis = false;
	}

	void PCLTools::visualizePointCloud(std::string id, pcl::PointCloud<pcl::PointXYZ>::Ptr pc) {

		pair<string, PointCloud<PointXYZ>::Ptr> nextPcPair(id, pc);
		visPointClouds.push_back(nextPcPair);

		viewer->setBackgroundColor(0, 0, 0);
		for(int currIdx = 0; currIdx < visPointClouds.size(); ++currIdx) {
			pair<string, PointCloud<PointXYZ>::Ptr> currPcPair = visPointClouds.at(currIdx);
			viewer->addPointCloud<PointXYZ>(currPcPair.second, currPcPair.first);
			viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 1, currPcPair.first);
		}
		viewer->initCameraParameters();

	}

	void PCLTools::stopVisualizationWindow() {

		isVisInit = false;
		keepShowingVis = false;

		visPointClouds.clear();

	}

	void PCLTools::runVisThread() {

		viewer = KUKADU_SHARED_PTR<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("3D Viewer"));
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

	}

	KUKADU_SHARED_PTR<boost::thread> PCLTools::initializeVisualizationWindow() {

		keepShowingVis = true;

		visThread = KUKADU_SHARED_PTR<boost::thread>(new boost::thread(&PCLTools::runVisThread, this));
		ros::Rate s(10);
		while(!isVisInit)
			s.sleep();

		return visThread;

	}

	// according to http://www.pcl-users.org/Finding-oriented-bounding-box-of-a-cloud-td4024616.html
	FitCube PCLTools::fitBox(PointCloud<PointXYZ>::Ptr cloud) {

		FitCube retCube;
		PCA<PointXYZ> pca;
		PointCloud<PointXYZ> proj;

		pca.setInputCloud(cloud);
		pca.project(*cloud, proj);

		PointXYZ proj_min;
		PointXYZ proj_max;
		getMinMax3D(proj, proj_min, proj_max);

		PointXYZ min;
		PointXYZ max;
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

		return retCube;

	}

	PointCloud<PointXYZ>::Ptr PCLTools::segmentPlanar(PointCloud<PointXYZ>::Ptr cloud, bool negative) {

		// Create the segmentation object for the planar model and set all the parameters
		PointCloud<PointXYZ>::Ptr cloud_f(new PointCloud<PointXYZ>);
		SACSegmentation<PointXYZ> seg;
		PointIndices::Ptr inliers(new PointIndices);
		ModelCoefficients::Ptr coefficients(new ModelCoefficients);
		PointCloud<PointXYZ>::Ptr cloud_plane(new PointCloud<PointXYZ> ());

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
		ExtractIndices<PointXYZ> extract;
		extract.setInputCloud(cloud);
		extract.setIndices(inliers);
		extract.setNegative(false);

		// Get the points associated with the planar surface
		extract.filter(*cloud_plane);

		// Remove the planar inliers, extract the rest
		extract.setNegative(negative);
		extract.filter(*cloud_f);

		return cloud_f;

	}

	KUKADU_SHARED_PTR<pcl::visualization::PCLVisualizer> PCLTools::getVisualizer() {
		return viewer;
	}

	void PCLTools::visDrawPlaneWithNormal(std::string id, arma::vec r0, arma::vec n) {

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
		PointXYZ p1, p2;
		p1.data[0] = r0(0); p1.data[1] = r0(1); p1.data[2] = r0(2);
		p2.data[0] = newLineEndPoint(0); p2.data[1] = newLineEndPoint(1); p2.data[2] = newLineEndPoint(2);
		viewer->addLine(p1, p2, id + "line");

	}

	void PCLTools::updateVisualizedPointCloud(std::string id, pcl::PointCloud<pcl::PointXYZ>::Ptr pc) {
		viewer->updatePointCloud(pc, id);
	}

	void PCLTools::visDrawBox(std::string id, struct FitCube dim) {
		viewer->addCube(dim.translation, dim.rotation, dim.width, dim.height, dim.depth, id);
	}

}
