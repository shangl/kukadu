#define USEBOOST

#include <kukadu/kukadu.hpp>

using namespace std;
using namespace kukadu;

int main(int argc, char** args) {

    ros::init(argc, args, "kukadu_controlqueue_demo"); ros::NodeHandle node; sleep(1);

    ARToolKitPlusNode ar(node);
    ros::spin();

    return EXIT_SUCCESS;

}
