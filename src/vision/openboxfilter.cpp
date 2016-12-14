#include <kukadu/vision/openboxfilter.hpp>

#include <iostream>
#include <armadillo>

#include <kukadu/utils/utils.hpp>
#include <kukadu/vision/planarcuttransformator.hpp>

using namespace std;
using namespace arma;

namespace kukadu {

    OpenBoxFilter::OpenBoxFilter(arma::vec center, double xOffset, double yOffset) {
        setBox(center, xOffset, yOffset);
    }

    void OpenBoxFilter::setBox(arma::vec center, double xOffset, double yOffset) {
        this->center = center;
        this->xOffset = xOffset;
        this->yOffset = yOffset;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr OpenBoxFilter::transformPc(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

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

}
