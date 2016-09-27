#ifndef KUKADU_STANDARDFILTER_HPP
#define KUKADU_STANDARDFILTER_HPP

#include "kukadu/robot/sensors/frctrqsensorfilter.hpp"

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
