
#ifndef StandardFilter_HPP
#define StandardFilter_HPP

#include "kukadu/robot/sensors/FrcTrqSensorFilter.hpp"

namespace kukadu
{

class StandardFilter : public FrcTrqSensorFilter{

    mes_result processedReadings;
public:
    void updateFilter(mes_result newReadings);
    mes_result getProcessedReading();

};

}

#endif
