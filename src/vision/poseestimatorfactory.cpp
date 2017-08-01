#include <kukadu/robot/hardwarefactory.hpp>
#include <kukadu/vision/poseestimatorfactory.hpp>


using namespace std;
namespace kukadu {

    std::map<std::string, std::function<KUKADU_SHARED_PTR<PoseEstimator>(std::shared_ptr<Kinect>, StorageSingleton& )> > PoseEstimatorFactory::poseEstimatorFactories{
            {"PCBlobDetector", [](std::shared_ptr<Kinect> kinect, StorageSingleton& dbStorage) {
                return make_shared<PCBlobDetector>(kinect, storage);
            }}
    };

    PoseEstimatorFactory::PoseEstimatorFactory() : storage(StorageSingleton::get()) {
        kinect = HardwareFactory::get().loadHardware("camera");
    }

    PoseEstimatorFactory &PoseEstimatorFactory::get() {
        static PoseEstimatorFactory instance;
        return instance;
    }

    KUKADU_SHARED_PTR<PoseEstimator> PoseEstimatorFactory::loadPoseEstimator(std::string poseEstimatorName) {
        return hardwareFactories[poseEstimatorName](kinect, storage);
    }

    std::vector<std::string> PoseEstimatorFactory::listAvailablePoseEstimators() {
        vector<string> poseEstimators;
        auto poseEstimatorRes = storage.executeQuery("select class_name from pose_estimators");
        while (poseEstimatorRes->next()) poseEstimators.push_back(poseEstimatorRes->getString("class_name"));
        return poseEstimators;
    }

    bool PoseEstimatorFactory::poseEstimatorExists(std::string poseEstimatorName) {
        auto poseEstimatorList = listAvailableSkills();
        if (std::find(poseEstimatorList.begin(), poseEstimatorList.end(), poseEstimatorName) != poseEstimatorList.end()) return true;
        return false;
    }
}
