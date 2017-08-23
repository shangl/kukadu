#ifndef KUKADU_POSEESTIMATORFACTORY_H
#define KUKADU_POSEESTIMATORFACTORY_H

#include <kukadu/vision/localizer.hpp>
#include <kukadu/storage/storagesingleton.hpp>
#include <geometry_msgs/PoseStamped.h>

namespace kukadu {

    class PoseEstimatorFactory : public StorageHolder {

    private:

        static std::map<std::string, std::function<KUKADU_SHARED_PTR<PoseEstimator>(KUKADU_SHARED_PTR<Hardware>, StorageSingleton& )> > poseEstimatorFactories;

        PoseEstimatorFactory();

    public:

        static PoseEstimatorFactory& get();

        KUKADU_SHARED_PTR<PoseEstimator> loadPoseEstimator(std::string poseEstimatorName, std::shared_ptr<kukadu::Hardware> hardware);

        std::vector<std::string> listAvailablePoseEstimators();

        bool poseEstimatorExists(std::string poseEstimatorName);

        geometry_msgs::PoseStamped getPoseFor(std::string id);

        std::vector< double > getDimensionsFor(std::string id);

    };

}
#endif
