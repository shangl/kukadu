#include <vector>
#include <iostream>
#include <armadillo>
#include <boost/foreach.hpp>
#include <boost/foreach_fwd.hpp>
#include <kukadu/utils/utils.hpp>
#include <kukadu/vision/pcltools.hpp>
#include <kukadu/types/kukadutypes.hpp>
#include <kukadu/vision/pcstdtrans.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/conditional_removal.h>
#include <boost/accumulators/statistics.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <kukadu/storage/moduleusagesingleton.hpp>

using namespace pcl;
using namespace std;
using namespace arma;

namespace kukadu {

    pcl::PointCloud<pcl::PointXYZ>::Ptr PCTransformator::removeRgb(pcl::PointCloud<pcl::PointXYZRGB>::Ptr toTransform) {

        KUKADU_MODULE_START_USAGE();

        PointCloud<PointXYZ> retPc;
        pcl::copyPointCloud(*toTransform, retPc);
        pcl::PointCloud<pcl::PointXYZ>::Ptr retCloud = retPc.makeShared();

        KUKADU_MODULE_END_USAGE();

        return retCloud;

    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCTransformator::fakeRgb(pcl::PointCloud<pcl::PointXYZ>::Ptr toTransform) {

        KUKADU_MODULE_START_USAGE();

        PointCloud<PointXYZRGB> retPc;
        pcl::copyPointCloud(*toTransform, retPc);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr retCloud = retPc.makeShared();

        KUKADU_MODULE_END_USAGE();

        return retCloud;

    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr PCTransformator::removeIntensity(pcl::PointCloud<pcl::PointXYZI>::Ptr toTransform) {

        KUKADU_MODULE_START_USAGE();

        PointCloud<PointXYZ> retPc;
        pcl::copyPointCloud(*toTransform, retPc);
        pcl::PointCloud<pcl::PointXYZ>::Ptr retCloud = retPc.makeShared();

        KUKADU_MODULE_END_USAGE();

        return retCloud;

    }

    PlanarCutTransformator::PlanarCutTransformator(arma::vec normalVec, arma::vec plainOriginVec) {
            setPlane(normalVec, plainOriginVec);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr PlanarCutTransformator::transformPc(pcl::PointCloud<pcl::PointXYZ>::Ptr pc) {
        KUKADU_MODULE_START_USAGE();
        pcl::PointCloud<pcl::PointXYZ>::Ptr retCloud = removeRgb(transformPc(fakeRgb(pc)));
        KUKADU_MODULE_END_USAGE();
        return retCloud;
    }

    void PlanarCutTransformator::setPlane(arma::vec normalVec, arma::vec plainOriginalVec) {

        KUKADU_MODULE_START_USAGE();

        double squaredSum = 0.0;
        for(int i = 0; i < normalVec.n_elem; ++i)
            squaredSum += pow(normalVec(i), 2.0);
        double len = sqrt(squaredSum);
        for(int i = 0; i < normalVec.n_elem; ++i)
            normalVec(i) /= len;

        this->normalVec = normalVec;

        this->plainOriginVec = plainOriginalVec;

        KUKADU_MODULE_END_USAGE();

    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr PlanarCutTransformator::transformPc(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc) {

        KUKADU_MODULE_START_USAGE();

        pcl::PointCloud<pcl::PointXYZRGB> retPc;
        pcl::PointCloud<pcl::PointXYZRGB> pcCopy = *pc;

        pcl::PointCloud<pcl::PointXYZRGB>::iterator pointIt = pcCopy.begin();
        pcl::PointCloud<pcl::PointXYZRGB>::iterator lastIt = pcCopy.end();

        while(pointIt != lastIt) {

            pcl::PointXYZRGB currentPoint = *pointIt;
            if(currentPoint.x == currentPoint.x && currentPoint.y == currentPoint.y &&
                    currentPoint.z == currentPoint.z) {

                vec r(3); r(0) = currentPoint.x; r(1) = currentPoint.y; r(2) = currentPoint.z;
                vec rMinPlainOrigVec = r - plainOriginVec;

                vec projVec = normalVec.t() * rMinPlainOrigVec;
                double projection = projVec(0);

                if(projection >= 0)
                    retPc.push_back(*pointIt);
                else {
                    // point gets kicked out
                }
            }
            ++pointIt;

        }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr retCloud = retPc.makeShared();

        KUKADU_MODULE_END_USAGE();

        return retCloud;

    }

    OpenBoxFilter::OpenBoxFilter(arma::vec center, double xOffset, double yOffset) {
        setBox(center, xOffset, yOffset);
    }

    void OpenBoxFilter::setBox(arma::vec center, double xOffset, double yOffset) {
        KUKADU_MODULE_START_USAGE();
        this->center = center;
        this->xOffset = xOffset;
        this->yOffset = yOffset;
        KUKADU_MODULE_END_USAGE();
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr OpenBoxFilter::transformPc(pcl::PointCloud<pcl::PointXYZ>::Ptr pc) {
        KUKADU_MODULE_START_USAGE();
        pcl::PointCloud<pcl::PointXYZ>::Ptr retCloud = removeRgb(transformPc(fakeRgb(pc)));
        KUKADU_MODULE_END_USAGE();
        return retCloud;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr OpenBoxFilter::transformPc(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {

        KUKADU_MODULE_START_USAGE();

        std::vector<double> n = createJointsVector(3, 0.0, 0.0, 1.0);
        std::vector<double> r0 = createJointsVector(3, center.at(0), center.at(1), center.at(2));
        PlanarCutTransformator planarCut(n, r0);
        cloud = planarCut.transformPc(cloud);

        // cut right part (from robot view)
        n = createJointsVector(3, -1.0, 0.0, 0.0);
        r0 = createJointsVector(3, center.at(0) + xOffset, center.at(1), center.at(2));
        planarCut.setPlane(n, r0);
        cloud = planarCut.transformPc(cloud);

        // cut front part (from robot view)
        n = createJointsVector(3, 0.0, 1.0, 0.0);
        r0 = createJointsVector(3, center.at(0), center.at(1) - yOffset, center.at(2));
        planarCut.setPlane(n, r0);
        cloud = planarCut.transformPc(cloud);

        // cut left part (from robot view)
        n = createJointsVector(3, 1.0, 0.0, 0.0);
        r0 = createJointsVector(3, center.at(0) - xOffset, center.at(1), center.at(2));
        planarCut.setPlane(n, r0);
        cloud = planarCut.transformPc(cloud);

        // cut back part (from robot view)
        n = createJointsVector(3, 0.0, -1.0, 0.0);
        r0 = createJointsVector(3, center.at(0), center.at(1) + yOffset, center.at(2));
        planarCut.setPlane(n, r0);
        cloud = planarCut.transformPc(cloud);

        KUKADU_MODULE_END_USAGE();

        return cloud;

    }

    ColorFilter::ColorFilter(std::vector<int> rgbRangeStart, std::vector<int> rgbRangeEnd) {
        this->rgbRangeStart = rgbRangeStart;
        this->rgbRangeEnd = rgbRangeEnd;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ColorFilter::transformPc(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {

        KUKADU_MODULE_START_USAGE();

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

        KUKADU_MODULE_END_USAGE();

        return cloud_filtered;

    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr ColorFilter::transformPc(pcl::PointCloud<pcl::PointXYZ>::Ptr pc) {
        return pc;
    }

    IntensityFunctor::IntensityFunctor(int intensity) {
        this->intensity = intensity;
    }

    bool IntensityFunctor::matchPoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const pcl::PointXYZ& currentPoint) {
        return true;
    }

    bool IntensityFunctor::matchPoint(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const pcl::PointXYZRGB& currentPoint) {

        // appearance based brightness
        int brightness = computeIntensity(currentPoint);
        if(brightness > intensity)
            return true;

        return false;

    }

    int IntensityFunctor::computeIntensity(const pcl::PointXYZRGB& currentPoint) {

        Eigen::Vector3i rgb = currentPoint.getRGBVector3i();

        int r = rgb.coeff(0);
        int g = rgb.coeff(1);
        int b = rgb.coeff(2);

        return (r + r + b + g + g + g) / 6;

    }

    void IntensityFunctor::initializeFilterForIteration(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
        // nothing to do
    }

    void IntensityFunctor::initializeFilterForIteration(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
        // nothing to do
    }

    CustomLambdaFilter::CustomLambdaFilter(CustomFunctor& f) : func(f) {
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr CustomLambdaFilter::transformPc(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

        KUKADU_MODULE_START_USAGE();

        func.initializeFilterForIteration(cloud);

        pcl::PointCloud<pcl::PointXYZ> retCloud;
        for(PointCloud<pcl::PointXYZ>::iterator pointIt = cloud->begin(); pointIt != cloud->end(); ++pointIt) {

            PointXYZ& p = *pointIt;
            if(func.matchPoint(cloud, p))
                retCloud.push_back(p);

        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr retCloudPtr = retCloud.makeShared();

        KUKADU_MODULE_END_USAGE();

        return retCloudPtr;

    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr CustomLambdaFilter::transformPc(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {

        KUKADU_MODULE_START_USAGE();

        func.initializeFilterForIteration(cloud);

        pcl::PointCloud<pcl::PointXYZRGB> retCloud;
        for(PointCloud<pcl::PointXYZRGB>::iterator pointIt = cloud->begin(); pointIt != cloud->end(); ++pointIt) {

            PointXYZRGB& p = *pointIt;
            if(func.matchPoint(cloud, p))
                retCloud.push_back(p);

        }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr retCloudPtr = retCloud.makeShared();

        KUKADU_MODULE_END_USAGE();

        return retCloudPtr;

    }

    HaloFunctor::HaloFunctor(int centerIntensity, double radius, double intensityVariance) : intFunc(centerIntensity) {
        this->radius = radius;
        this->intensityVariance = intensityVariance;
    }

    void HaloFunctor::initializeFilterForIteration(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
        // only supported for xyz
        throw KukaduException("Halo functor is only available for clouds with rgb data");
    }

    void HaloFunctor::initializeFilterForIteration(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
        intFunc.initializeFilterForIteration(cloud);
        rgbKdTree.setInputCloud(cloud);
    }

    bool HaloFunctor::matchPoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const pcl::PointXYZ& currentPoint) {
        // only supported for xyz
        throw KukaduException("Halo functor is only available for clouds with rgb data");
    }

    bool HaloFunctor::matchPoint(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const pcl::PointXYZRGB& currentPoint) {

        // if we have a bright point candidate --> search for a halo around it (high variance of the whole neighbourhood)
        if(intFunc.matchPoint(cloud, currentPoint)) {

            vector<int> pointIdxRadiusSearch;
            vector<float> pointRadiusSquaredDistance;

            if(rgbKdTree.radiusSearch(currentPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {

                vector<int> intensities(pointIdxRadiusSearch.size());
                //cout << intensities.size() << endl;
                for(int i = 0; i < pointIdxRadiusSearch.size(); i++)
                    intensities[i] = intFunc.computeIntensity(cloud->at(pointIdxRadiusSearch.at(i)));

                boost::accumulators::accumulator_set<int, boost::accumulators::stats<boost::accumulators::tag::variance> > acc;
                for_each(intensities.begin(), intensities.end(), boost::bind<void>(boost::ref(acc), _1));

                if(boost::accumulators::variance(acc) > intensityVariance)
                    return true;

            }

        }

        return false;

    }

}
