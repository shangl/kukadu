#include <map>
#include <vector>
#include <armadillo>
#include <pcl/common/pca.h>
#include <pcl/filters/filter.h>
#include <kukadu/vision/localizer.hpp>
#include <kukadu/vision/visualizersingleton.hpp>
#include <kukadu/storage/moduleusagesingleton.hpp>

#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/statistical_outlier_removal.h>

using namespace std;
using namespace pcl;
using namespace arma;

namespace kukadu {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmentPlanar(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, bool negative) {

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
        if (inliers->indices.size() == 0) {
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

        return cloud_f;

    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterCluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, bool negative) {

        // Create the filtering object
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK(50);
        sor.setStddevMulThresh(1.0);
        sor.filter(*cloud);

        int count = 0;
        vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusterPointers;
        for (pcl::PointCloud<pcl::PointXYZRGB>::iterator it = cloud->begin(); it != cloud->end(); it++, ++count) {

            if (!(count % 4)) {

                vec point(3);
                point(0) = it->x;
                point(1) = it->y;
                point(2) = it->z;

                bool foundPlace = false;
                for (int i = 0; i < clusterPointers.size() && !foundPlace; ++i) {

                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr currentCluster = clusterPointers.at(i);
                    for (pcl::PointCloud<pcl::PointXYZRGB>::iterator it2 = currentCluster->begin();
                         it2 != currentCluster->end(); it2++) {
                        vec clusterPoint(3);
                        clusterPoint(0) = it2->x;
                        clusterPoint(1) = it2->y;
                        clusterPoint(2) = it2->z;
                        vec difVec = point - clusterPoint;
                        vec distVec = difVec.t() * difVec;
                        double distance = distVec(0);
                        if (distance < pow(0.05, 2)) {
                            foundPlace = true;
                            currentCluster->push_back(*it);
                            break;
                        }
                    }

                }

                if (!foundPlace) {
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr nextCluster(new pcl::PointCloud<pcl::PointXYZRGB>());
                    nextCluster->push_back(*it);
                    clusterPointers.push_back(nextCluster);
                }

            }

        }

        int maxClusterIdx = 0;
        int maxClusterSize = 0;
        for (int i = 0; i < clusterPointers.size(); ++i) {
            int currentClusterSize = (clusterPointers.at(i))->size();
            if (currentClusterSize > maxClusterSize) {
                maxClusterIdx = i;
                maxClusterSize = currentClusterSize;
            }
        }

        return clusterPointers.at(maxClusterIdx);

    }

    PoseEstimator::PoseEstimator(StorageSingleton &dbStorage, std::string poseEstimatorName) : storage(dbStorage) {
        this->poseEstimatorName = poseEstimatorName;

    }

    std::pair<geometry_msgs::Pose, arma::vec> PoseEstimator::estimatePose(std::string id) {
        auto poseVectorPair = estimatePoseInternal(id);
        int poseEstimatorId = storage.getCachedLabelId("pose_estimators", "estimator_id", "class_name", poseEstimatorName);

        stringstream s;
        s << "INSERT INTO localize_objects (object_id, pose_estimator_id, x_coordinate, y_coordinate, z_coordinate, quat, timestamp, frame_id) ";
        s << "VALUES (" << id << ", " << poseEstimatorId << ", " << poseVectorPair.first.pose.orientation.x << ", " << poseVectorPair.first.pose.orientation.y << ", " << poseVectorPair.first.pose.orientation.z << ", " << poseVectorPair.first.pose.orientation.w << ", ";
        s << TimedObject::getCurrentTime() << ", ";
        s << storage.getCachedLabelId("reference_frames", "frame_id", "frame_name", poseVectorPair.first.header.frame_id);
        s << ")";

        storage.executeStatementPriority(s.str());
        return {poseVectorPair.first.pose, poseVectorPair.second};
    }


    void PoseEstimator::install() {
        if(!storage.checkLabelExists("pose_estimators", "class_name", this->poseEstimatorName)) {
            int estimatorId = storage.getNextIdInTable("pose_estimators", "estimator_id");

            stringstream s;
            s << "INSERT INTO pose_estimators (estimator_id, class_name) VALUES (" << estimatorId << ", " << this->poseEstimatorName << ")";
            storage.executeStatementPriority(s.str());
            s.str("");

            std::vector<std::string> availableObjects = this->getAvailableObjects();
            for(auto object = availableObjects.begin(); object != availableObjects.end(); ++object) {
                int objectId = -1;
                if(!storage.checkLabelExists("objects", "object_name", *object)) {
                    objectId = storage.getNextIdInTable("objects", "object_id");
                    s << "INSERT INTO objects (object_id, object_name) VALUES (" << objectId << ", " <<  *object << ")";
                    storage.executeStatementPriority(s.str());
                    s.str("");
                }

                if(objectId == -1) {
                    objectId = storage.getCachedLabelId("objects", "object_id", "object_name", *object);
                }

                s << "INSERT INTO recogniseable_objects(estimator_id, object_id) VALUES (" << estimatorId << ", " << objectId << ")";
                storage.executeStatementPriority(s.str());
                s.str("");
            }

            this->installInternal();
        }
    }

    PCBlobDetector::PCBlobDetector(std::shared_ptr<Kinect> kinect, StorageSingleton& dbStorage, std::string targetFrame,
                                   arma::vec center, double xOffset, double yOffset, bool visualizeResult) : PoseEstimator(dbStorage, "PCBlobDetector") {

        this->kinect = kinect;
        this->targetFrame = targetFrame;
        this->center = center;
        this->xOffset = xOffset;
        this->yOffset = yOffset;
        this->visualizeResult = visualizeResult;

    }

    std::vector<std::string> PCBlobDetector::getAvailableObjects(){

    }

    void PCBlobDetector::installInternal() {

    }

    std::string PCBlobDetector::getLocalizerFrame() {
        return targetFrame;
    }

    std::pair<geometry_msgs::PoseStamped, arma::vec> PCBlobDetector::estimatePoseInternal(std::string id) {

        KUKADU_MODULE_START_USAGE();

        geometry_msgs::PoseStamped retPose;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = kinect->getCurrentColorPointCloud();
        retPose.header.frame_id = kinect->getTargetFrame();

        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

        OpenBoxFilter boxFilter(center, xOffset, yOffset);

        // filter out some stuff
        cloud = boxFilter.transformPc(cloud);
        cloud = segmentPlanar(cloud, true);
        cloud = filterCluster(cloud, false);
        cloud = segmentPlanar(cloud, false);

        pcl::PCA<pcl::PointXYZRGB> pca;
        pca.setInputCloud(cloud);

        pcl::PointCloud<pcl::PointXYZRGB> projected;
        pca.project(*cloud, projected);

        pcl::PointXYZRGB pointMin, pointMax;
        pcl::getMinMax3D(projected, pointMin, pointMax);

        auto objectCenter = pca.getMean().head(3);
        auto eigenVectors = pca.getEigenVectors();

        retPose.pose.position.x = objectCenter(0);
        retPose.pose.position.y = objectCenter(1);
        retPose.pose.position.z = objectCenter(2);

        /*

        vec stdBaseX(3); stdBaseX(0) = 1.0; stdBaseX(1) = 0.0; stdBaseX(2) = 0.0;
        vec stdBaseY(3); stdBaseY(0) = 0.0; stdBaseY(1) = 1.0; stdBaseY(2) = 0.0;
        vec stdBaseZ(3); stdBaseZ(0) = 0.0; stdBaseZ(1) = 0.0; stdBaseZ(2) = 1.0;

        vec targetBaseX(3); targetBaseX(0) = eigenVectors(0, 0); targetBaseX(1) = eigenVectors(1, 0); targetBaseX(2) = eigenVectors(2, 0);
        vec targetBaseY(3); targetBaseY(0) = eigenVectors(0, 1); targetBaseY(1) = eigenVectors(1, 1); targetBaseY(2) = eigenVectors(2, 1);
        vec targetBaseZ(3); targetBaseZ(0) = eigenVectors(0, 2); targetBaseZ(1) = eigenVectors(1, 2); targetBaseZ(2) = eigenVectors(2, 2);

        vector<vec> stdBaseVec = {stdBaseX, stdBaseY, stdBaseZ};
        vector<vec> targetBaseVec = {targetBaseX, targetBaseY, targetBaseZ};

        mat rotationMatrix = computeRotFromCorrespondences(stdBaseVec, targetBaseVec);
        vec rpy = rotationMatrixToRpy(rotationMatrix);

        tf::Quaternion quat = rpyToQuat(rpy(0), rpy(1), rpy(2));
        retPose.orientation.x = quat.getX();
        retPose.orientation.y = quat.getY();
        retPose.orientation.z = quat.getZ();
        retPose.orientation.w = quat.getW();

        */

        Eigen::Matrix3f affine_trans;
        affine_trans.col(2) << (eigenVectors.col(0).cross(eigenVectors.col(1))).normalized();
        affine_trans.col(0) << eigenVectors.col(0);
        affine_trans.col(1) << eigenVectors.col(1);

        Eigen::Quaternionf rotation(affine_trans);
        retPose.pose.orientation.x = rotation.x();
        retPose.pose.orientation.y = rotation.y();
        retPose.pose.orientation.z = rotation.z();
        retPose.pose.orientation.w = rotation.w();

        vec dimensions(3);
        dimensions(0) = pointMax.x - pointMin.x;
        dimensions(1) = pointMax.y - pointMin.y;
        dimensions(2) = pointMax.z - pointMin.z;

        if (visualizeResult) {
            auto &vis = VisualizerSingleton::get();
            vis.showPointCloud("pc", cloud);
            vis.drawBox("fixbox", retPose.pose, dimensions);
        }

        KUKADU_MODULE_END_USAGE();

        return {retPose, dimensions};

    }

    geometry_msgs::Pose PCBlobDetector::localizeObject(std::string id) {

        KUKADU_MODULE_START_USAGE();

        auto result = estimatePose(id);

        KUKADU_MODULE_END_USAGE();

        return result.first;

    }

    std::map<std::string, geometry_msgs::Pose> PCBlobDetector::localizeObjects() {

        KUKADU_MODULE_START_USAGE();

        map<string, geometry_msgs::Pose> retMap;
        retMap["blob"] = localizeObject(0);

        KUKADU_MODULE_END_USAGE();

        return retMap;

    }

    std::vector<geometry_msgs::Pose> PCBlobDetector::localizeObjects(std::vector<std::string> ids) {

        KUKADU_MODULE_START_USAGE();

        vector<geometry_msgs::Pose> retPoses;
        retPoses.push_back(localizeObject(0));

        KUKADU_MODULE_END_USAGE();

        return retPoses;

    }

}
