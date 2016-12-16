#ifndef KUKADU_ARLOCALIZER_H
#define KUKADU_ARLOCALIZER_H

#include <kukadu/vision/localizer.hpp>
#include <kukadu/types/kukadutypes.hpp>

#include <list>
#include <ros/ros.h>
#include <opencv/cv.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <kukadu/ARParamConfig.h>
#include <tf/transform_datatypes.h>
#include <kukadu/vision/arlocalizer/ar.h>
#include <kukadu/vision/arlocalizer/arMulti.h>
#include <kukadu/vision/arlocalizer/CameraAdvImpl.h>
#include <kukadu/vision/arlocalizer/ARToolKitPlus.h>

namespace kukadu {

    #define ARTOOLKITPLUS_IMAGE_SRC "camera/rgb/image_raw"
    #define ARTOOLKITPLUS_DEBUG_WINDOWS_NAME "artoolkitplus"
    #define ARTOOLKITPLUS_DEFAULT_DISTORTED_INPUT true
    #define ARTOOLKITPLUS_DEFAULT_SKIP_FRAMES 0
    #define ARTOOLKITPLUS_DEFAULT_SHOW_CAMERA_IMAGE true
    #define ARTOOLKITPLUS_DEFAULT_BASE_FRAME "camera"
    #define ARTOOLKITPLUS_DEFAULT_PATTERN_FRAME "pattern"
    #define ARTOOLKITPLUS_DEFAULT_PATTERN_FILE ""
    #define ARTOOLKITPLUS_DEFAULT_TF_PREFIX ""

    #define ARTOOLKITPLUS_MARKER_MODE_BCH "bch"
    #define ARTOOLKITPLUS_MARKER_MODE_SIMPEL "simple"
    #define ARTOOLKITPLUS_DEFAULT_MARKER_MODE ARTOOLKITPLUS_MARKER_MODE_BCH // "SIMPLE
    #define ARTOOLKITPLUS_DEFAULT_PATTERN_WITH 0.1
    #define ARTOOLKITPLUS_DEFAULT_THRESHOLD 0
    #define ARTOOLKITPLUS_DEFAULT_BOARDER_WIDTH 0
    #define ARTOOLKITPLUS_DEFAULT_UNDIST_INTERATIONS 10
    #define ARTOOLKITPLUS_UNDIST_MODE_NONE "none"
    #define ARTOOLKITPLUS_UNDIST_MODE_STD "std"
    #define ARTOOLKITPLUS_UNDIST_MODE_LUT "lut"
    #define ARTOOLKITPLUS_DEFAULT_UNDIST_MODE ARTOOLKITPLUS_UNDIST_MODE_STD
    #define ARTOOLKITPLUS_POSE_ESTIMATION_MODE_NORMAL "normal"
    #define ARTOOLKITPLUS_POSE_ESTIMATION_MODE_CONT "cont"
    #define ARTOOLKITPLUS_POSE_ESTIMATION_MODE_RPP "rpp"
    #define ARTOOLKITPLUS_DEFAULT_POSE_ESTIMATION_MODE ARTOOLKITPLUS_POSE_ESTIMATION_MODE_RPP
    #define ARTOOLKITPLUS_DEFAULT_TRACKER_SINGLE_MARKER true
    #define ARTOOLKITPLUS_DEFAULT_TRACKER_MULTI_MARKER false
    #define ARTOOLKITPLUS_DEFAULT_MULIT_MARKER_LITE_DETECTION true

    class TrackerSingleMarker;
    class TrackerMultiMarker;
    class Tracker;

    class ARTag2D : public ARMarkerInfo {
    public:
      static const int NO_PATTERN = -1;
      static const int PATTERN = 0;
      int belongsToPattern;
      ARTag2D()
       : belongsToPattern(NO_PATTERN) {
      }
      ARTag2D(const ARTag2D &tag)
       : belongsToPattern(tag.belongsToPattern) {
        setARMarkerInfo(tag);
      }
      ARTag2D(const ARMarkerInfo &tag)
       : belongsToPattern(NO_PATTERN) {
        setARMarkerInfo(tag);
      }
      void setARMarkerInfo(const ARMarkerInfo &tag) {
        ARMarkerInfo *p = (ARMarkerInfo *) this;
        *p = tag;
      }
    };

    class ARToolKitPlusNode {

    public:

        ARToolKitPlusNode(ros::NodeHandle& n);

        ARToolKitPlusNode(ros::NodeHandle& n,
                                             int skip_frames,
                                             bool show_camera_image,
                                             std::string tf_prefix,
                                             bool tracker_single_marker,
                                             bool tracker_multi_marker,
                                             std::string pattern_frame,
                                             std::string pattern_file,
                                             MARKER_MODE marker_mode,
                                             double pattern_width,
                                             double threshold,
                                             double border_width,
                                             int undist_iterations,
                                             bool distorted_input,
                                             UNDIST_MODE undist_mode,
                                             POSE_ESTIMATOR pose_estimation_mode,
                                             std::string imageTopic,
                                             bool use_multi_marker_lite_detection);

    private:

        /*** begin parameters ***/
        int skip_frames;
        bool show_camera_image;
        std::string tf_prefix;
        bool tracker_single_marker;
        bool tracker_multi_marker;
        std::string pattern_frame;
        std::string pattern_file;
        MARKER_MODE marker_mode;
        double pattern_width;
        double threshold;
        double border_width;
        int undist_iterations;
        bool distorted_input;
        UNDIST_MODE undist_mode;
        POSE_ESTIMATOR pose_estimation_mode;
        std::string imageTopic;
        bool use_multi_marker_lite_detection;
        /*** end parameters ***/

        ros::NodeHandle n_;
        ros::NodeHandle n_param_;
        int callback_counter_;
        image_transport::ImageTransport imageTransport_;
        image_transport::CameraSubscriber cameraSubscriber_;
        tf::TransformBroadcaster transformBroadcaster_;
        boost::shared_ptr<kukadu::TrackerSingleMarker> trackerSingleMarker_;
        boost::shared_ptr<kukadu::TrackerMultiMarker> trackerMultiMarker_;
        std::vector<kukadu::ARTag2D> arTags2D_;
        std::list<tf::StampedTransform> markerTransforms_;
        const kukadu::ARMultiMarkerInfoT* arMultiMarkerInfo_;
        Logger* logger_;
        kukadu::CameraAdvImpl* camera;
        ros::Publisher pub_perceptions_;

        void initTrackerSingleMarker(const sensor_msgs::CameraInfoConstPtr& camer_info_);
        void initTrackerMultiMarker(const sensor_msgs::CameraInfoConstPtr& camer_info_);

        void updateParameterTrackerSingleMarker(const sensor_msgs::CameraInfoConstPtr& camer_info);
        void updateParameterTrackerMultiMarker(const sensor_msgs::CameraInfoConstPtr& camer_info);

        void imageCallback(const sensor_msgs::ImageConstPtr& image_msg,
                           const sensor_msgs::CameraInfoConstPtr& info_msg);
        void estimatePoses(const std_msgs::Header &header);
        void publishTf();
        void generateDebugImage(cv::Mat &img);
        int matrix2Tf(const kukadu::ARFloat M[3][4], tf::Transform &transform);
        void readParam();
        void init();

    };

}

#endif
