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

    KUKADU_SHARED_PTR<PoseEstimator> PoseEstimatorFactory::loadPoseEstimator(std::string poseEstimatorName, std::shared_ptr<kukadu::Hardware> hardware) {
        return poseEstimatorFactories[poseEstimatorName](hardware, getStorage());
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

    geometry_msgs::PoseStamped PoseEstimatorFactory::getPoseFor(std::string id) {
        auto& storage = getStorage();
        if(storage.checkLabelExists("objects", "object_name", id)) {
            int objectId = storage.getCachedLabelId("objects", "object_id", "object_name", id);

            stringstream s;
            s << "SELECT DISTINCT x_position as xp, y_position as yp, z_position as zp, x_orientation as xo, y_orientation as yo, z_orientation as zo, w_orientation as wo, frame_id FROM localized_objects WHERE object_id=";
            s << objectId << " AND timestamp=(SELECT max(timestamp) FROM localized_objects WHERE object_id=" << objectId << ")";

            auto result = storage.executeQuery(s.str());
            geometry_msgs::PoseStamped poseStamped;

            if(result.get()->next()){
                double xp, yp, zp, xo, yo, zo, wo;
                int frame_id;
                xo = result.get()->getDouble("xo");
                xp = result.get()->getDouble("xp");
                yo = result.get()->getDouble("yo");
                yp = result.get()->getDouble("yp");
                zo = result.get()->getDouble("zo");
                zp = result.get()->getDouble("zp");
                wo = result.get()->getDouble("wo");
                frame_id = result.get()->getInt("frame_id");

                poseStamped.header.frame_id = frame_id;
                poseStamped.pose.position.x = xp;
                poseStamped.pose.position.y = yp;
                poseStamped.pose.position.z = zp;
                poseStamped.pose.orientation.x = xo;
                poseStamped.pose.orientation.y = yo;
                poseStamped.pose.orientation.z = zo;
                poseStamped.pose.orientation.w = wo;

                return poseStamped;
            } else throw KukaduException("No position for this object available.");
        } else throw KukaduException("This object does not exist.");
    }

    std::vector< double > PoseEstimatorFactory::getDimensionsFor(std::string id) {
        auto& storage = getStorage();
        if(storage.checkLabelExists("objects", "object_name", id)) {
            int objectId = storage.getCachedLabelId("objects", "object_id", "object_name", id);

            stringstream s;
            s << "SELECT DISTINCT x_dimension as xd, y_dimension as yd, z_dimension as zd FROM localized_objects WHERE object_id=";
            s << objectId << " AND timestamp=(SELECT max(timestamp) FROM localized_objects WHERE object_id=" << objectId << ")";

            auto result = storage.executeQuery(s.str());

            if(result.get()->next()){
                double xd, yd, zd;
                xd = result.get()->getDouble("xd");
                yd = result.get()->getDouble("yd");
                zd = result.get()->getDouble("zd");

                return {xd, yd, zd};
            } else throw KukaduException("No dimensions for this object available.");
        } else throw KukaduException("This object does not exist.");
    }


}
