#ifndef KUKADU_PLANNING_ALL_H
#define KUKADU_PLANNING_ALL_H
    #include <kukadu/planning/moveit.hpp>
    #include <kukadu/planning/planning.hpp>
    #ifndef USEBOOST
        #include <kukadu/planning/komo.hpp>
    #else
        #warning Komo planner is disabled due to used old C++ standard
    #endif
#endif
