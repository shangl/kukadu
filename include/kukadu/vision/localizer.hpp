#ifndef KUKADU_LOCALIZER_H
#define KUKADU_LOCALIZER_H

#include <vector>
#include <string>
#include <geometry_msgs/Pose.h>

namespace kukadu {

    class Localizer {

    private:

    public:

        virtual std::string getLocalizerFrame() = 0;

        virtual geometry_msgs::Pose localizeObject(std::string id) = 0;
        virtual std::map<std::string, geometry_msgs::Pose> localizeObjects() = 0;
        virtual std::vector<geometry_msgs::Pose> localizeObjects(std::vector<std::string> ids) = 0;

    };

}

#endif
