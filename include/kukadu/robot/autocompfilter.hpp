#ifndef KUKADU_AUTOCOMPFILTER_H
#define KUKADU_AUTOCOMPFILTER_H

#include <kukadu/robot/queue.hpp>
#include <kukadu/utils/types.hpp>
#include <kukadu/planning/planning.hpp>
#include <kukadu/types/kukadutypes.hpp>

namespace kukadu {

    class AutoCompensatingFilter: public FrcTrqSensorFilter {

    #ifndef USEBOOST
        static auto constexpr  SENSOR_MISS_ALIGNMENT_COMPARED_TO_END_EFFECTOR = -2.27;

        static auto constexpr FORCE_MIN_LIMIT = 0.3;   // Limits that auto scaling cannot go below
        static auto constexpr TORQUE_MIN_LIMIT = 0.02; // Limits that auto scaling cannot go below

        static auto constexpr MIN_THRESHOLD = 0.1; // Minimum sensor reading

        static auto constexpr  SMOOTHING = 0.9;
        static auto constexpr  DC_FILTER1 = 0.99999;
        static auto constexpr  DC_FILTER2 = 0.99999;

        static auto constexpr  FORGET_SHRINK = 0.99;
        static auto constexpr  FORGET_EXPAND = 0.05;
        static auto constexpr  ACCEPTANCE_INTERVAL = 0.90;
    #else
        static const double SENSOR_MISS_ALIGNMENT_COMPARED_TO_END_EFFECTOR = -2.27;

        static const double FORCE_MIN_LIMIT = 0.3;   // Limits that auto scaling cannot go below
        static const double TORQUE_MIN_LIMIT = 0.02; // Limits that auto scaling cannot go below

        static const double MIN_THRESHOLD = 0.1; // Minimum sensor reading

        static const double SMOOTHING = 0.9;
        static const double DC_FILTER1 = 0.99999;
        static const double DC_FILTER2 = 0.99999;

        static const double FORGET_SHRINK = 0.99;
        static const double FORGET_EXPAND = 0.05;
        static const double ACCEPTANCE_INTERVAL = 0.90;
    #endif

        bool firstReading;

        int cnt;
        double currentTime;
        kukadu_mutex readingMutex;

        KUKADU_SHARED_PTR<kukadu::ControlQueue> robotinoQueue;
        KUKADU_SHARED_PTR<kukadu::Kinematics> mvKin;

        std::vector<std::vector<double> > newDataSet;
        std::vector<double> processedFilterReadings;
        std::vector<double> smoothingFilterMemory;
        std::vector<double> DCFilter1Memory;
        std::vector<double> DCFilter2Memory;
        std::vector<std::pair<double,double> > limitsFilterMemory;

        int sign(double x);
        void lowpassFilter(std::vector<double>& filter,std::vector<double> newVal, double forgettingFactor);
        void limitsFilter(std::vector<std::pair<double,double> >& filter,std::vector<std::vector<double> > newDataSet, double forgettingFactorShrink, double forgettingFactorExpand, double percentageWithinLimits);
        std::vector<double> removeBias(std::vector<double> values,std::vector<double> bias);
        std::vector<double> scaleReadings(std::vector<double> msg, bool coupleLimits=false);
        std::vector<double> limitandThresholdReadings(std::vector<double> msg);
        std::vector<double> projectVectors(double vecX,double vecY,double vecZ,double alpha,double beta,double gamma);
        std::vector<double> projectReadings(std::vector<double> readings, geometry_msgs::Pose currentPose);

    public:

        AutoCompensatingFilter(KUKADU_SHARED_PTR<kukadu::ControlQueue> myQueue,KUKADU_SHARED_PTR<kukadu::Kinematics> myKin);

        void printFilteredSensorVal();
        void updateFilter(mes_result newReadings);

        mes_result getProcessedReading();

    };

}

#endif
