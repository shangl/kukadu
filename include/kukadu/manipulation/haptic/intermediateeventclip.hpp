#ifndef KUKADU_INTERMEDIATEEVENTCLIP_H
#define KUKADU_INTERMEDIATEEVENTCLIP_H

#include <string>
#include <vector>
#include <kukadu/types/kukadutypes.hpp>
#include <kukadu/manipulation/sensingcontroller.hpp>
#include <kukadu/learning/projective_simulation/clips.hpp>

namespace kukadu {

    class IntermediateEventClip : public Clip {

    private:

        int hapticMode;

        std::string caption;

        KUKADU_SHARED_PTR<SensingController> sensingEvent;

    public:

        IntermediateEventClip(KUKADU_SHARED_PTR<SensingController> sensingEvent,
                              int level, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, std::string clipValues, int immunity);

        IntermediateEventClip(KUKADU_SHARED_PTR<SensingController> sensingEvent,
                              int level, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, KUKADU_SHARED_PTR<std::vector<int> > clipValues, int immunity);

        virtual std::string toString() const;
        std::pair<int, KUKADU_SHARED_PTR<Clip> > jumpNextRandom();

        KUKADU_SHARED_PTR<SensingController> getSensingController();

    };

}

#endif // INTERMEDIATEEVENTCLIP_H
