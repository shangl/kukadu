#ifndef KUKADU_POSEESTIMATORFACTORY_H
#define KUKADU_POSEESTIMATORFACTORY_H

#include <kukadu/vision/localizer.hpp>

namespace kukadu {

    class PoseEstimatorFactory {

    private:

        StorageSingleton& storage;

        KUKADU_SHARED_PTR<Kinect> kinect;

        static std::map<std::string, std::function<KUKADU_SHARED_PTR<PoseEstimator>(std::shared_ptr<Kinect>, StorageSingleton& )> > poseEstimatorFactories;

        PoseEstimatorFactory();

    public:

        static PoseEstimatorFactory& get();

        KUKADU_SHARED_PTR<PoseEstimator> loadPoseEstimator(std::string poseEstimatorName);

        std::vector<std::string> listAvailablePoseEstimators();

        bool poseEstimatorExists(std::string poseEstimatorName);

    };

}
#endif
