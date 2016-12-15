#ifndef PCTRANSFORMATOR_H
#define PCTRANSFORMATOR_H

#include <armadillo>
#include <pcl_ros/transforms.h>

namespace kukadu {

    class PCTransformator {

    private:


    public:

        virtual pcl::PointCloud<pcl::PointXYZ>::Ptr transformPc(pcl::PointCloud<pcl::PointXYZ>::Ptr pc) = 0;
        virtual pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformPc(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc) = 0;

        static pcl::PointCloud<pcl::PointXYZ>::Ptr removeIntensity(pcl::PointCloud<pcl::PointXYZI>::Ptr toTransform);
        static pcl::PointCloud<pcl::PointXYZ>::Ptr removeRgb(pcl::PointCloud<pcl::PointXYZRGB>::Ptr toTransform);
        static pcl::PointCloud<pcl::PointXYZRGB>::Ptr fakeRgb(pcl::PointCloud<pcl::PointXYZ>::Ptr toTransform);

    };

    class PlanarCutTransformator : public PCTransformator {

    private:

        arma::vec normalVec;
        arma::vec plainOriginVec;

    public:

        PlanarCutTransformator(arma::vec normalVec, arma::vec plainOriginVec);

        virtual pcl::PointCloud<pcl::PointXYZ>::Ptr transformPc(pcl::PointCloud<pcl::PointXYZ>::Ptr pc);
        virtual pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformPc(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc);

        void setPlane(arma::vec normalVec, arma::vec plainOriginalVec);

    };

    class OpenBoxFilter : public PCTransformator {

    private:

        double xOffset;
        double yOffset;

        arma::vec center;

    public:

        OpenBoxFilter(arma::vec center, double xOffset, double yOffset);

        virtual pcl::PointCloud<pcl::PointXYZ>::Ptr transformPc(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
        virtual pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformPc(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc);

        void setBox(arma::vec center, double xOffset, double yOffset);

    };

    class ColorFilter : public PCTransformator {

    private:

        std::vector<int> rgbRangeStart;
        std::vector<int> rgbRangeEnd;

    public:

        ColorFilter(std::vector<int> rgbRangeStart, std::vector<int> rgbRangeEnd);

        virtual pcl::PointCloud<pcl::PointXYZ>::Ptr transformPc(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
        virtual pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformPc(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc);

    };

    class CustomFunctor {

    public:

        virtual bool matchPoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const pcl::PointXYZ& currentPoint) = 0;
        virtual bool matchPoint(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const pcl::PointXYZRGB& currentPoint) = 0;

    };

    class CustomLambdaFilter : public PCTransformator {

    private:

        CustomFunctor& func;

    public:

        CustomLambdaFilter(CustomFunctor& f);
        virtual pcl::PointCloud<pcl::PointXYZ>::Ptr transformPc(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
        virtual pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformPc(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc);

    };

    class IntensityFunctor : public CustomFunctor {

    private:

        int intensity;

    public:

        IntensityFunctor(int intensity);

        virtual bool matchPoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const pcl::PointXYZ& currentPoint);
        virtual bool matchPoint(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const pcl::PointXYZRGB& currentPoint);

    };

}



#endif // PCTRANSFORMATOR_H
