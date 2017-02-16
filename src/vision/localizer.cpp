#include <map>
#include <vector>
#include <kukadu/vision/pcltools.hpp>
#include <kukadu/vision/localizer.hpp>

using namespace std;

namespace kukadu {

    PCBlobDetector::PCBlobDetector(boost::shared_ptr<Kinect> kinect, std::string targetFrame, arma::vec center, double xOffset, double yOffset) {
        this->kinect = kinect;
        this->targetFrame = targetFrame;
        this->center = center;
        this->xOffset = xOffset;
        this->yOffset = yOffset;
    }

    std::string PCBlobDetector::getLocalizerFrame() {
        return targetFrame;
    }

    geometry_msgs::Pose PCBlobDetector::localizeObject(std::string id) {

        geometry_msgs::Pose retPose;
        PCLTools pt;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = kinect->getCurrentColorPointCloud();

        OpenBoxFilter boxFilter(center, xOffset, yOffset);

        // filter out some stuff
        cloud = boxFilter.transformPc(cloud);
        cloud = pt.segmentPlanar(cloud, true);
        cloud = pt.filterCluster(cloud, false);
        cloud = pt.segmentPlanar(cloud, false);

        FitCube cube = pt.fitBox(cloud);

        retPose.position.x = cube.translation.coeff(0);
        retPose.position.y = cube.translation.coeff(1);
        retPose.position.z = cube.translation.coeff(2);

        retPose.orientation.x = cube.rotation.x();
        retPose.orientation.y = cube.rotation.y();
        retPose.orientation.z = cube.rotation.z();
        retPose.orientation.w = cube.rotation.w();

        return retPose;

    }

    std::map<std::string, geometry_msgs::Pose> PCBlobDetector::localizeObjects() {

        map<string, geometry_msgs::Pose> retMap;
        retMap["blob"] = localizeObject(0);

    }

    std::vector<geometry_msgs::Pose> PCBlobDetector::localizeObjects(std::vector<std::string> ids) {

        vector<geometry_msgs::Pose> retPoses;
        retPoses.push_back(localizeObject(0));
        return retPoses;

    }

}
