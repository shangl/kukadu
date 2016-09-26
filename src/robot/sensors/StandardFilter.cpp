#include "kukadu/robot/sensors/StandardFilter.hpp"

using namespace std;

namespace kukadu{

//Filter::Filter(){
//    cnt=0;
//    newDataSet.resize(6);
//    firstReading=true;

//    processedFilterReadings=DCFilter1Memory=DCFilter2Memory=smoothingFilterMemory={0.0,0.0,0.0,0.0,0.0,0.0};

//    limitsFilterMemory = {std::pair<double,double>(-1.0 * FORCE_MIN_LIMIT,1.0 * FORCE_MIN_LIMIT),
//                          std::pair<double,double>(-1.0 * FORCE_MIN_LIMIT,1.0 * FORCE_MIN_LIMIT),
//                          std::pair<double,double>(-1.0 * FORCE_MIN_LIMIT,1.0 * FORCE_MIN_LIMIT),
//                          std::pair<double,double>(-1.0 * TORQUE_MIN_LIMIT,1.0 * TORQUE_MIN_LIMIT),
//                          std::pair<double,double>(-1.0 * TORQUE_MIN_LIMIT,1.0 * TORQUE_MIN_LIMIT),
//                          std::pair<double,double>(-1.0 * TORQUE_MIN_LIMIT,1.0 * TORQUE_MIN_LIMIT)};
//}




 void StandardFilter::updateFilter(mes_result currentReading){
    processedReadings= currentReading;
 }

 mes_result StandardFilter::getProcessedReading(){
    return processedReadings;
 }


}
