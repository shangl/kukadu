
#ifndef FrcTrqSensorFilter_HPP
#define FrcTrqSensorFilter_HPP

#include "geometry_msgs/Pose.h"
#include <kukadu/utils/types.hpp>

namespace kukadu
{

class FrcTrqSensorFilter{

public:

    virtual mes_result getProcessedReading()=0;
    virtual void updateFilter(mes_result newReadings)=0;
};

}

#endif
