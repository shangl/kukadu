#ifndef KUKADU_VISION_H
#define KUKADU_VISION_H
    #include <kukadu/vision/kinect.hpp>
    #include <kukadu/vision/localizer.hpp>
    #include <kukadu/vision/visualizersingleton.hpp>
    #include <kukadu/vision/poseestimatorfactory.hpp>
    #ifdef USEBOOST
        #include <kukadu/vision/pcltools.hpp>
        #include <kukadu/vision/pcstdtrans.hpp>
        #include <kukadu/vision/arlocalizer.hpp>
    #endif
#endif
