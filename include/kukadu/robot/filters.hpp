#ifndef KUKADU_FILTERS_H
#define KUKADU_FILTERS_H

#include <kukadu/utils/types.hpp>
#include <kukadu/types/kukadutypes.hpp>

namespace kukadu {

    class FrcTrqSensorFilter {

    public:

        virtual mes_result getProcessedReading() = 0;
        virtual void updateFilter(mes_result newReadings) = 0;

    };

    class StandardFilter : public FrcTrqSensorFilter {

    private:

        mes_result processedReadings;

    public:

        virtual void updateFilter(mes_result newReadings);

        virtual mes_result getProcessedReading();

    };

}

#endif
