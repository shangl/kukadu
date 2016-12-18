#include <kukadu/vision/arlocalizer.hpp>
#include <kukadu/utils/utils.hpp>

#include <iostream>
#include <opencv/cv.hpp>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <kukadu/vision/arlocalizer/TrackerMultiMarkerImpl.h>
#include <kukadu/vision/arlocalizer/TrackerSingleMarkerImpl.h>

using namespace std;

namespace kukadu {

    ArLocalizer::ArLocalizer(ros::NodeHandle& n, std::string imageTopic, bool show_camera_image) {

        toolkit = KUKADU_SHARED_PTR<ARToolKitPlusNode>(new ARToolKitPlusNode(n, imageTopic, show_camera_image));

    }

    std::map<std::string, geometry_msgs::Pose> ArLocalizer::localizeObjects() {
        return toolkit->getDetectedPoses();
    }

    geometry_msgs::Pose ArLocalizer::localizeObject(std::string id) {

        std::map<std::string, geometry_msgs::Pose> detectedPoses = toolkit->getDetectedPoses();
        return detectedPoses[id];

    }

    std::vector<geometry_msgs::Pose> ArLocalizer::localizeObjects(std::vector<std::string> ids) {

        std::vector<geometry_msgs::Pose> retPoses;
        std::map<std::string, geometry_msgs::Pose> detectedPoses = toolkit->getDetectedPoses();
        for(int i = 0; i < ids.size(); ++i)
            retPoses.push_back(detectedPoses[ids.at(i)]);

        return retPoses;

    }

    class ArLogger : public Logger {
        void artLog(const char* nStr) {
            printf("%s", nStr);
        }
    };

    ARToolKitPlusNode::ARToolKitPlusNode(ros::NodeHandle& n) : n_(n), n_param_("~"), callback_counter_(0), imageTransport_(n_) {

        skip_frames = 0;
        show_camera_image = true;
        tf_prefix = "";
        tracker_single_marker = false;
        tracker_multi_marker = true;
        pattern_frame = "pattern";
        pattern_file = resolvePath("$KUKADU_HOME/cfg/arlocalizer/markerboard_0000-0011.cfg");
        marker_mode = MARKER_ID_BCH;
        pattern_width = 0.1;
        threshold = 0;
        border_width = 0.125;
        undist_iterations = 10;
        distorted_input = true;
        undist_mode = UNDIST_STD;
        pose_estimation_mode = POSE_ESTIMATOR_RPP;
        imageTopic = "camera/rgb/image_raw";
        use_multi_marker_lite_detection = true;

        init();
        cameraSubscriber_ = imageTransport_.subscribeCamera(ARTOOLKITPLUS_IMAGE_SRC, 1, &ARToolKitPlusNode::imageCallback, this);

    }

    ARToolKitPlusNode::ARToolKitPlusNode(ros::NodeHandle& n, std::string imageTopic, bool show_camera_image) : n_(n), n_param_("~"), callback_counter_(0), imageTransport_(n_) {

        skip_frames = 0;
        this->show_camera_image = show_camera_image;
        tf_prefix = "";
        tracker_single_marker = false;
        tracker_multi_marker = true;
        pattern_frame = "pattern";
        pattern_file = resolvePath("$KUKADU_HOME/cfg/arlocalizer/markerboard_0000-0011.cfg");
        marker_mode = MARKER_ID_BCH;
        pattern_width = 0.1;
        threshold = 0;
        border_width = 0.125;
        undist_iterations = 10;
        distorted_input = true;
        undist_mode = UNDIST_STD;
        pose_estimation_mode = POSE_ESTIMATOR_RPP;
        this->imageTopic = imageTopic;
        use_multi_marker_lite_detection = true;

        init();
        cameraSubscriber_ = imageTransport_.subscribeCamera(ARTOOLKITPLUS_IMAGE_SRC, 1, &ARToolKitPlusNode::imageCallback, this);

    }

    ARToolKitPlusNode::ARToolKitPlusNode(ros::NodeHandle& n,
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
                                         bool use_multi_marker_lite_detection) : n_(n), n_param_("~"), callback_counter_(0), imageTransport_(n_) {

        this->skip_frames = skip_frames;
        this->show_camera_image = show_camera_image;
        this->tf_prefix = tf_prefix;
        this->tracker_single_marker = tracker_single_marker;
        this->tracker_multi_marker = tracker_multi_marker;
        this->pattern_frame = pattern_frame;
        this->pattern_file = pattern_file;
        this->marker_mode = marker_mode;
        this->pattern_width = pattern_width;
        this->threshold = threshold;
        this->border_width = border_width;
        this->undist_iterations = undist_iterations;
        this->distorted_input = distorted_input;
        this->undist_mode = undist_mode;
        this->pose_estimation_mode = pose_estimation_mode;
        this->imageTopic = imageTopic;
        this->use_multi_marker_lite_detection = use_multi_marker_lite_detection;

        init();
        cameraSubscriber_ = imageTransport_.subscribeCamera(ARTOOLKITPLUS_IMAGE_SRC, 1, &ARToolKitPlusNode::imageCallback, this);

    }

    class ARCamera: public kukadu::CameraAdvImpl {
    public:

        virtual ~ARCamera() {
        };

        bool isInputDistorted() {
            if (undist_iterations == 1) {
                return false;
            } else {
                return true;
            }
        }

        ARCamera(const sensor_msgs::CameraInfoConstPtr& _camer_info, int _undist_iterations, bool _input_distorted) {


            // kukadu::CameraAdvImpl Parameter
            if (_input_distorted) {
                // using the ros Intrinsic camera matrix for the raw (distorted) images.
                this->fc[0] = (ARFloat) _camer_info->K[0];
                this->fc[1] = (ARFloat) _camer_info->K[4];
                this->cc[0] = (ARFloat) _camer_info->K[2];
                this->cc[1] = (ARFloat) _camer_info->K[5];

                undist_iterations = _undist_iterations;

                this->kc[0] = (ARFloat) _camer_info->D[0];
                this->kc[1] = (ARFloat) _camer_info->D[1];
                this->kc[2] = (ARFloat) _camer_info->D[2];
                this->kc[3] = (ARFloat) _camer_info->D[3];
                this->kc[4] = (ARFloat) _camer_info->D[4];
                this->kc[5] = (ARFloat) 0.;

            } else {
                // using the ros Projection/camera matrix
                this->fc[0] = (ARFloat) _camer_info->P[0];
                this->fc[1] = (ARFloat) _camer_info->P[5];
                this->cc[0] = (ARFloat) _camer_info->P[2];
                this->cc[1] = (ARFloat) _camer_info->P[6];

                undist_iterations = 1;

                for (int i = 0; i < 6; i++)
                    this->kc[i] = (ARFloat) 0.;

            }

            // kukadu::Camera Parameter
            // fileName

            // ARToolKit::ARParam Parameter
            xsize = _camer_info->width;
            ysize = _camer_info->height;

            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 4; j++)
                    this->mat[i][j] = (ARFloat) 0.;

            mat[0][0] = fc[0]; // fc_x
            mat[1][1] = fc[1]; // fc_y
            mat[0][2] = cc[0]; // cc_x
            mat[1][2] = cc[1]; // cc_y
            mat[2][2] = 1.0;

            if (_input_distorted == false) {
                // using the ros Projection/camera matrix
                mat[0][3] = (ARFloat) _camer_info->P[3];
                mat[1][3] = (ARFloat) _camer_info->P[7];
            }

            for (int i = 0; i < 4; i++)
                this->dist_factor[i] = this->kc[i];
        }
    };

    void ARToolKitPlusNode::initTrackerMultiMarker(const sensor_msgs::CameraInfoConstPtr& camer_info) {

        trackerMultiMarker_ = boost::shared_ptr<kukadu::TrackerMultiMarker>(new kukadu::TrackerMultiMarkerImpl<AR_TRACKER_PARAM>(camer_info->width, camer_info->height));
        const char* description = trackerMultiMarker_->getDescription();

        if(logger_ == NULL) logger_ = new ArLogger();
        trackerMultiMarker_->setLogger(logger_);

        // set a logger so we can output error messages
        trackerMultiMarker_->setLogger(logger_);
        trackerMultiMarker_->setPixelFormat(kukadu::PIXEL_FORMAT_LUM);

        camera = new ARCamera(camer_info, undist_iterations, distorted_input);

        if(!fileExists(pattern_file))
            throw KukaduException("(ArLocalizer) pattern file does not exist");

        if (!trackerMultiMarker_->init(camera, pattern_file.c_str(), 1.0f, 1000.0f)) {
            ROS_ERROR("ERROR: init() failed");
        }

    }

    void ARToolKitPlusNode::initTrackerSingleMarker(const sensor_msgs::CameraInfoConstPtr& camer_info) {

        trackerSingleMarker_ = boost::shared_ptr<kukadu::TrackerSingleMarker>(new kukadu::TrackerSingleMarkerImpl<AR_TRACKER_PARAM>(camer_info->width, camer_info->height));
        const char* description = trackerSingleMarker_->getDescription();

        // set a logger so we can output error messages
        trackerSingleMarker_->setLogger(logger_);
        trackerSingleMarker_->setPixelFormat(kukadu::PIXEL_FORMAT_LUM);

        ARCamera *camera = new ARCamera(camer_info, undist_iterations, distorted_input);
        if (!trackerSingleMarker_->init(camera, 1.0f, 1000.0f)) {
            ROS_ERROR("ERROR: init() failed");
        }

    }

    void ARToolKitPlusNode::updateParameterTrackerSingleMarker(const sensor_msgs::CameraInfoConstPtr& camer_info) {

        ARCamera * camera = (ARCamera *) trackerSingleMarker_->getCamera();
        if (camera->isInputDistorted() != distorted_input) {
            delete camera;
            camera = new ARCamera(camer_info, undist_iterations, distorted_input);
            trackerSingleMarker_->setCamera(camera);
        }

        // define size of the marker
        trackerSingleMarker_->setPatternWidth(pattern_width);

        // the marker in the BCH test image has a thin border...
        if (border_width > 0) {
            trackerSingleMarker_->setBorderWidth(border_width);
        } else {
            trackerSingleMarker_->setBorderWidth((marker_mode == MARKER_ID_BCH) ? 0.125f : 0.250f);
        }

        // set a threshold. alternatively we could also activate automatic thresholding
        if (threshold > 0) {
            trackerSingleMarker_->activateAutoThreshold(false);
            trackerSingleMarker_->setThreshold(threshold);
        } else {
            trackerSingleMarker_->activateAutoThreshold(true);
        }
        // let's use lookup-table undistortion for high-speed
        // note: LUT only works with images up to 1024x1024
        trackerSingleMarker_->setUndistortionMode((kukadu::UNDIST_MODE) undist_mode);

        // RPP is more robust than ARToolKit's standard pose estimator
        trackerSingleMarker_->setPoseEstimator((kukadu::POSE_ESTIMATOR) pose_estimation_mode);
        trackerSingleMarker_->setMarkerMode(marker_mode);
        }

        void ARToolKitPlusNode::updateParameterTrackerMultiMarker(const sensor_msgs::CameraInfoConstPtr& camer_info) {

        ARCamera * camera = (ARCamera *) trackerMultiMarker_->getCamera();
        if (camera->isInputDistorted() != distorted_input) {
            delete camera;
            camera = new ARCamera(camer_info, undist_iterations, distorted_input);
            trackerMultiMarker_->setCamera(camera);
        }

        trackerMultiMarker_->setUseDetectLite(use_multi_marker_lite_detection);

        // the marker in the BCH test image has a thin border...
        if (border_width > 0) {
            trackerMultiMarker_->setBorderWidth(border_width);
        } else {
            trackerMultiMarker_->setBorderWidth((marker_mode == MARKER_ID_BCH) ? 0.125f : 0.250f);
        }

        // set a threshold. alternatively we could also activate automatic thresholding
        if (threshold > 0) {
            trackerMultiMarker_->activateAutoThreshold(false);
            trackerMultiMarker_->setThreshold(threshold);
        } else {
            trackerMultiMarker_->activateAutoThreshold(true);
        }
        // let's use lookup-table undistortion for high-speed
        // note: LUT only works with images up to 1024x1024
        trackerMultiMarker_->setUndistortionMode((kukadu::UNDIST_MODE) undist_mode);

        // RPP is more robust than ARToolKit's standard pose estimator
        trackerMultiMarker_->setPoseEstimator((kukadu::POSE_ESTIMATOR) pose_estimation_mode);

        // switch to simple ID based markers
        // use the tool in tools/IdPatGen to generate markers
        trackerMultiMarker_->setMarkerMode(marker_mode);

    }

    void ARToolKitPlusNode::imageCallback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& camer_info_) {
        callback_counter_++;
        if((callback_counter_ % (skip_frames+1) ) != 0) {
            return;
        }
        cv_bridge::CvImagePtr img;
        try {
            img = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        if (tracker_single_marker) {
            if(trackerSingleMarker_ == NULL) initTrackerSingleMarker(camer_info_);

            updateParameterTrackerSingleMarker(camer_info_);
            kukadu::ARMarkerInfo* arMarkerInfo;
            int nNumMarkers;
            int markerId = trackerSingleMarker_->calc(img->image.data, -1, true, &arMarkerInfo, &nNumMarkers);
            float conf = (float) trackerSingleMarker_->getConfidence();
            arTags2D_.resize(nNumMarkers);
            for(int i = 0; i < arTags2D_.size(); i++) {
                arTags2D_[i] = arMarkerInfo[i];
            }
        }
        if (tracker_multi_marker) {
            if(trackerMultiMarker_ == NULL) initTrackerMultiMarker(camer_info_);
            updateParameterTrackerMultiMarker(camer_info_);
            int nNumMarkers = trackerMultiMarker_->calc(img->image.data);

            arMultiMarkerInfo_ = trackerMultiMarker_->getMultiMarkerConfig();
            arTags2D_.clear();
            arTags2D_.reserve(trackerMultiMarker_->getNumDetectedMarkers());
            /// Sort out marker which are part of multi marker patterns
            for(int i = 0; i < trackerMultiMarker_->getNumDetectedMarkers(); i++) {
                const kukadu::ARMarkerInfo &singleMarker = trackerMultiMarker_->getDetectedMarker(i);
                arTags2D_.push_back(singleMarker);
                arTags2D_.back().belongsToPattern = kukadu::ARTag2D::NO_PATTERN;
                for(int j = 0; (j < arMultiMarkerInfo_->marker_num); j++) {
                    const kukadu::ARMultiEachMarkerInfoT &multiMarker = arMultiMarkerInfo_->marker[j];
                    if(singleMarker.id == multiMarker.patt_id) {
                        arTags2D_.back().belongsToPattern = kukadu::ARTag2D::PATTERN;
                        break;
                    }
                }
            }
        }

        estimatePoses(image_msg->header);

        publishTf();

        if (show_camera_image) {
            cv::Mat img_debug;
            cvtColor(img->image, img_debug, CV_GRAY2BGR);
            generateDebugImage(img_debug);
            cv::imshow("artracker" + std::string(" - debug"), img_debug);
            cv::waitKey(5);
        }
    }

    void ARToolKitPlusNode::estimatePoses(const std_msgs::Header &header) {

        ARFloat center[2];
        center[0] = 0;
        center[1] = 0;
        ARFloat pose[3][4];
        tf::Transform trans;
        tf::StampedTransform st;
        char frame[0xFF];

        tfMutex.lock();

            markerTransforms_.clear();

            if(trackerMultiMarker_) {

                if( arMultiMarkerInfo_->marker_num > 0) {
                    const ARFloat *p = trackerMultiMarker_->getModelViewMatrix();

                    for(int r = 0; r < 3; r++) {
                        pose[r][0] = p[r+0];
                        pose[r][1] = p[r+4];
                        pose[r][2] = p[r+8];
                        pose[r][3] = p[r+12];
                    }

                    matrix2Tf(pose, trans);
                    std::string child_frame = tf::resolve(tf_prefix, pattern_frame);
                    st = tf::StampedTransform(trans, header.stamp, header.frame_id, child_frame);
                    markerTransforms_.push_back(st);

                }

            }

            for(std::vector<kukadu::ARTag2D>::iterator arTag =  arTags2D_.begin(); arTag != arTags2D_.end(); arTag++) {

                if (arTag->id < 0)
                    continue;
                if (arTag->belongsToPattern != kukadu::ARTag2D::NO_PATTERN)
                    continue;

                sprintf(frame, "t%i", arTag->id);

                if(trackerMultiMarker_)
                    trackerMultiMarker_->executeSingleMarkerPoseEstimator(&(*arTag), center, pattern_width, pose);

                if(trackerSingleMarker_)
                    trackerSingleMarker_->executeSingleMarkerPoseEstimator(&(*arTag), center, pattern_width, pose);

                matrix2Tf(pose, trans);
                std::string child_frame = tf::resolve(tf_prefix, frame);
                st = tf::StampedTransform(trans, header.stamp, header.frame_id, child_frame);

                markerTransforms_.push_back(st);

            }

        tfMutex.unlock();

    }

    std::map<std::string, geometry_msgs::Pose> ARToolKitPlusNode::getDetectedPoses() {

        std::map<std::string, geometry_msgs::Pose> posesMap;

        tfMutex.lock();

            for(list<tf::StampedTransform>::iterator it = markerTransforms_.begin(); it != markerTransforms_.end(); ++it) {
                tf::StampedTransform& currentTransform = *it;
                tf::Quaternion currentRot = currentTransform.getRotation();
                tf::Vector3 currentTranslation = currentTransform.getOrigin();
                geometry_msgs::Pose currentPose;
                currentPose.position.x = currentTranslation.getX();
                currentPose.position.y = currentTranslation.getY();
                currentPose.position.z = currentTranslation.getZ();
                currentPose.orientation.x = currentRot.getX();
                currentPose.orientation.y = currentRot.getY();
                currentPose.orientation.z = currentRot.getZ();
                currentPose.orientation.w = currentRot.getW();
                posesMap[string(currentTransform.child_frame_id_)] = currentPose;
            }

        tfMutex.unlock();

        return posesMap;

    }

    void ARToolKitPlusNode::init() {

        if (show_camera_image) {
            cv::namedWindow("artracker" + std::string(" - debug"), 1);
        }

    }

    void ARToolKitPlusNode::generateDebugImage(cv::Mat &img) {
        char text[0xFF];
        cv::Point a, b, c;
        cv::Scalar green(0, 255, 0);
        cv::Scalar green2(0, 200, 0);
        cv::Scalar red(0, 0, 255);
        cv::Scalar blue(255, 0, 0);
        int fondFace = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;

        std::vector<kukadu::ARTag2D>::const_iterator artag;
        for (artag = arTags2D_.begin(); artag != arTags2D_.end(); artag++) {
            cv::Scalar lineColour = green;
            if (artag->id < 0) lineColour = red;
            a = cv::Point(artag->vertex[3][0], artag->vertex[3][1]);
            if (artag->belongsToPattern != kukadu::ARTag2D::NO_PATTERN) {
                a = cv::Point(artag->vertex[0][0], artag->vertex[0][1]);
                b = cv::Point(artag->vertex[2][0], artag->vertex[2][1]);
                cv::line(img, a, b, lineColour, 1);
                a = cv::Point(artag->vertex[1][0], artag->vertex[1][1]);
                b = cv::Point(artag->vertex[3][0], artag->vertex[3][1]);
                cv::line(img, a, b, lineColour, 1);
            } else {
              for (int v = 0; v < 4; v++) {
                  b = cv::Point(artag->vertex[v][0], artag->vertex[v][1]);
                  cv::line(img, a, b, lineColour, 1);
                  a = b;
              }
            }
            sprintf(text, "%i", artag->id);
            cv::putText(img, text, cv::Point(artag->pos[0], artag->pos[1]), fondFace, 0.2, green);
        }
    }

    int ARToolKitPlusNode::matrix2Tf(const ARFloat M[3][4], tf::Transform &transform) {
        tf::Matrix3x3 R(M[0][0], M[0][1], M[0][2], M[1][0], M[1][1], M[1][2], M[2][0], M[2][1], M[2][2]);
        tf::Vector3 T(M[0][3], M[1][3], M[2][3]);
        //transform = tf::Transform(R, T); // this causes a TF to MSG: Quaternion Not Properly Normalized message
        tf::Quaternion quat;
        R.getRotation(quat);
        transform = tf::Transform(quat, T);
    }

    void ARToolKitPlusNode::publishTf() {

        // disabled direct tf

        /*
        for(std::list<tf::StampedTransform>::iterator it =  markerTransforms_.begin(); it != markerTransforms_.end(); it++) {
            transformBroadcaster_.sendTransform(*it);
        }
        */
    }

}
