#include <kukadu/robot/hardwarefactory.hpp>
#include <kukadu/vision/poseestimatorfactory.hpp>


using namespace std;
namespace kukadu {

    std::map<std::string, std::function<KUKADU_SHARED_PTR<PoseEstimator>(KUKADU_SHARED_PTR<Hardware>, StorageSingleton& )> > PoseEstimatorFactory::poseEstimatorFactories{
            {"PCBlobDetector", [](KUKADU_SHARED_PTR<Hardware> camera, StorageSingleton& dbStorage) {
                return make_shared<PCBlobDetector>(KUKADU_DYNAMIC_POINTER_CAST<Kinect>(camera), dbStorage);
            }}
    };

    PoseEstimatorFactory::PoseEstimatorFactory() : StorageHolder(StorageSingleton::get()) {
    }

    PoseEstimatorFactory &PoseEstimatorFactory::get() {
        static PoseEstimatorFactory instance;
        return instance;
    }

    KUKADU_SHARED_PTR<PoseEstimator> PoseEstimatorFactory::loadPoseEstimator(std::string poseEstimatorName) {
        return poseEstimatorFactories[poseEstimatorName](HardwareFactory::get().loadHardware("camera"), getStorage());
    }

    std::vector<std::string> PoseEstimatorFactory::listAvailablePoseEstimators() {
        vector<string> poseEstimators;
        auto poseEstimatorRes = getStorage().executeQuery("select class_name from pose_estimators");
        while (poseEstimatorRes->next()) poseEstimators.push_back(poseEstimatorRes->getString("class_name"));
        return poseEstimators;
    }

    bool PoseEstimatorFactory::poseEstimatorExists(std::string poseEstimatorName) {
        auto poseEstimatorList = listAvailablePoseEstimators();
        if (std::find(poseEstimatorList.begin(), poseEstimatorList.end(), poseEstimatorName) != poseEstimatorList.end()) return true;
        return false;
    }
}
