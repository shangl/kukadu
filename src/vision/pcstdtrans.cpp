#include <kukadu/utils/utils.hpp>
#include <kukadu/vision/pcstdtrans.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/conditional_removal.h>

using namespace pcl;
using namespace std;
using namespace arma;

namespace kukadu {

    pcl::PointCloud<pcl::PointXYZ>::Ptr PCTransformator::removeRgb(pcl::PointCloud<pcl::PointXYZRGB>::Ptr toTransform) {

        PointCloud<PointXYZ> retPc;
        pcl::copyPointCloud(*toTransform, retPc);
        return retPc.makeShared();

    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCTransformator::fakeRgb(pcl::PointCloud<pcl::PointXYZ>::Ptr toTransform) {

        PointCloud<PointXYZRGB> retPc;
        pcl::copyPointCloud(*toTransform, retPc);
        return retPc.makeShared();

    }

    PlanarCutTransformator::PlanarCutTransformator(arma::vec normalVec, arma::vec plainOriginVec) {
            setPlane(normalVec, plainOriginVec);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr PlanarCutTransformator::transformPc(pcl::PointCloud<pcl::PointXYZ>::Ptr pc) {
        return removeRgb(transformPc(fakeRgb(pc)));
    }

    void PlanarCutTransformator::setPlane(arma::vec normalVec, arma::vec plainOriginalVec) {
        this->normalVec = normalise(normalVec);
        this->plainOriginVec = plainOriginalVec;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr PlanarCutTransformator::transformPc(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc) {

        pcl::PointCloud<pcl::PointXYZRGB> retPc;

        pcl::PointCloud<pcl::PointXYZRGB>::iterator pointIt = pc->begin();
        pcl::PointCloud<pcl::PointXYZRGB>::iterator lastIt = pc->end();

        while(pointIt != lastIt) {
            pcl::PointXYZRGB currentPoint = *pointIt;
            vec r = stdToArmadilloVec(createJointsVector(3, currentPoint.x, currentPoint.y, currentPoint.z));
            vec comp = normalVec.t() * (r - plainOriginVec);
            double coordinate = comp(0);
            if(coordinate >= 0) {
                retPc.push_back(*pointIt);
            } else {
                // point gets kicked out
            }
            ++pointIt;
        }

        return retPc.makeShared();

    }

    OpenBoxFilter::OpenBoxFilter(arma::vec center, double xOffset, double yOffset) {
        setBox(center, xOffset, yOffset);
    }

    void OpenBoxFilter::setBox(arma::vec center, double xOffset, double yOffset) {
        this->center = center;
        this->xOffset = xOffset;
        this->yOffset = yOffset;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr OpenBoxFilter::transformPc(pcl::PointCloud<pcl::PointXYZ>::Ptr pc) {
        return removeRgb(transformPc(fakeRgb(pc)));
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr OpenBoxFilter::transformPc(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {

        vec n = stdToArmadilloVec(createJointsVector(3, 0.0, 0.0, 1.0));
        vec r0 = stdToArmadilloVec(createJointsVector(3, 0.0, 0.0, center(2)));
        PlanarCutTransformator planarCut(n, r0);
        cloud = planarCut.transformPc(cloud);

        // cut right part (from robot view)
        n = stdToArmadilloVec(createJointsVector(3, -1.0, 0.0, 0.0));
        r0 = stdToArmadilloVec(createJointsVector(3, center(0) + xOffset, 0.0, 0.0));
        planarCut.setPlane(n, r0);
        cloud = planarCut.transformPc(cloud);

        // cut front part (from robot view)
        n = stdToArmadilloVec(createJointsVector(3, 0.0, 1.0, 0.0));
        r0 = stdToArmadilloVec(createJointsVector(3, 0.0, center(1) - yOffset, 0.0));
        planarCut.setPlane(n, r0);
        cloud = planarCut.transformPc(cloud);

        // cut left part (from robot view)
        n = stdToArmadilloVec(createJointsVector(3, 1.0, 0, 0.0));
        r0 = stdToArmadilloVec(createJointsVector(3, center(0) - xOffset, 0.0, 0.0));
        planarCut.setPlane(n, r0);
        cloud = planarCut.transformPc(cloud);

        // cut back part (from robot view)
        n = stdToArmadilloVec(createJointsVector(3, 0.0, -1.0, 0.0));
        r0 = stdToArmadilloVec(createJointsVector(3, 0.0, center(1) + yOffset, 0.0));
        planarCut.setPlane(n, r0);
        cloud = planarCut.transformPc(cloud);

        return cloud;

    }

    ColorFilter::ColorFilter(std::vector<int> rgbRangeStart, std::vector<int> rgbRangeEnd) {
        this->rgbRangeStart = rgbRangeStart;
        this->rgbRangeEnd = rgbRangeEnd;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ColorFilter::transformPc(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());

        // build the condition
        int rMax = rgbRangeEnd.at(0);
        int rMin = rgbRangeStart.at(0);
        int gMax = rgbRangeEnd.at(1);
        int gMin = rgbRangeStart.at(1);
        int bMax = rgbRangeEnd.at(2);
        int bMin = rgbRangeStart.at(2);

        pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond(new pcl::ConditionAnd<pcl::PointXYZRGB>());
        color_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("r", pcl::ComparisonOps::LT, rMax)));
        color_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("r", pcl::ComparisonOps::GT, rMin)));
        color_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("g", pcl::ComparisonOps::LT, gMax)));
        color_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("g", pcl::ComparisonOps::GT, gMin)));
        color_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("b", pcl::ComparisonOps::LT, bMax)));
        color_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("b", pcl::ComparisonOps::GT, bMin)));

        // build the filter
        pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem(color_cond);
        condrem.setInputCloud(cloud);
        condrem.setKeepOrganized(true);

        // apply filter
        condrem.filter(*cloud_filtered);

        return cloud_filtered;

    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr ColorFilter::transformPc(pcl::PointCloud<pcl::PointXYZ>::Ptr pc) {
        return pc;
    }

}
