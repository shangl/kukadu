#ifndef KUKADU_VISION_H
#define KUKADU_VISION_H
    #define USEBOOST
    #ifdef USEBOOST
        #include <kukadu/vision/kinect.hpp>
        #include <kukadu/vision/pcltools.hpp>
        #include <kukadu/vision/pcstdtrans.hpp>
        #include <kukadu/vision/arlocalizer.hpp>
        #include <kukadu/vision/visioninterface.hpp>
    #endif
    #undef USEBOOST
#endif
