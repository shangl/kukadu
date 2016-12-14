#include <kukadu/vision/planarcuttransformator.hpp>

#include <iostream>
#include <armadillo>

#include <kukadu/utils/utils.hpp>

using namespace std;
using namespace arma;

namespace kukadu {

    PlanarCutTransformator::PlanarCutTransformator(arma::vec normalVec, arma::vec plainOriginVec) {
        setPlane(normalVec, plainOriginVec);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr PlanarCutTransformator::transformPc(pcl::PointCloud<pcl::PointXYZ>::Ptr pc) {

        pcl::PointCloud<pcl::PointXYZ> retPc;

        pcl::PointCloud<pcl::PointXYZ>::iterator pointIt = pc->begin();
        pcl::PointCloud<pcl::PointXYZ>::iterator lastIt = pc->end();

        while(pointIt != lastIt) {
            pcl::PointXYZ currentPoint = *pointIt;
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
    void PlanarCutTransformator::setPlane(arma::vec normalVec, arma::vec plainOriginalVec) {
        this->normalVec = normalise(normalVec);
        this->plainOriginVec = plainOriginalVec;
    }

}
