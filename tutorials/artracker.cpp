#include <kukadu/kukadu.hpp>

using namespace std;
using namespace kukadu;

int main(int argc, char** args) {

    ros::init(argc, args, "kukadu_artracker_demo"); ros::NodeHandle node; sleep(1);
    ros::AsyncSpinner spinner(10); spinner.start();

    auto& storage = StorageSingleton::get();

    ArLocalizer arLocal(node, "camera/rgb/image_raw", true);
    ros::Rate r(1);
    while(true) {
        std::map<std::string, geometry_msgs::Pose> poses = arLocal.localizeObjects();
        for(std::map<std::string, geometry_msgs::Pose>::iterator it = poses.begin(); it != poses.end(); ++it) {
            string id = (*it).first;
            geometry_msgs::Pose p = (*it).second;
            cout << "found id = " << id << " with pose (" << p.position.x << ", " << p.position.y << ", " << p.position.z << ") " <<
                    "(" << p.orientation.x << ", " << p.orientation.y << ", " << p.orientation.z << ", " << p.orientation.w << ")" << endl;
        }
        r.sleep();
    }

    storage.waitForEmptyCache();

    return EXIT_SUCCESS;

}
