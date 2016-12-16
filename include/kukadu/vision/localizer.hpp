#ifndef KUKADU_LOCALIZER_H
#define KUKADU_LOCALIZER_H

#include <vector>
#include <string>
#include <geometry_msgs/Pose.h>

namespace kukadu {

    class Localizer {

    private:

    public:

        virtual geometry_msgs::Pose localizeObject(std::string id) = 0;
        virtual std::vector<geometry_msgs::Pose> localizeObjects(std::vector<std::string> ids) = 0;

    };

}

#endif
