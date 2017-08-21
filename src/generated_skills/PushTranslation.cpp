#include <kukadu/generated_skills/PushTranslation.hpp>

using namespace arma;

namespace kukadu {
    namespace skill {
        struct rectAlignment {
            bool aligned;
            int alignmentType;
            double rotationError;
        };
    }


    PushTranslation::PushTranslation(kukadu::StorageSingleton &storage,
                                     std::vector<KUKADU_SHARED_PTR<kukadu::Hardware> > hardware)
            : Controller(storage, "PushTranslation", hardware, 0.01) {
        auto queue = KUKADU_DYNAMIC_POINTER_CAST<ControlQueue>(getUsedHardware()[0]);
        komoPlanner = std::make_shared<Komo>(queue, resolvePath("$KUKADU_HOME/external/komo/share/data/kuka/data/iis_robot.kvg"), resolvePath("$KUKADU_HOME/external/komo/share/data/kuka/config/MT.cfg"), "left");
    }

    bool PushTranslation::requiresGraspInternal() {
        return false;
    }

    bool PushTranslation::producesGraspInternal() {
        return false;
    }

    std::shared_ptr<kukadu::ControllerResult> PushTranslation::executeInternal() {
        /* think pushForward is never set
        if (pushForward)
            pushForward->execute();
        */
        auto queue = KUKADU_DYNAMIC_POINTER_CAST<ControlQueue>(getUsedHardware()[0]);
        queue->install();
        queue->start();

        auto hand = KUKADU_DYNAMIC_POINTER_CAST<KukieHand>(getUsedHardware()[1]);
        hand->install();
        hand->start();

        double palmShift = 0.14;

        auto moveHomeSkill = kukadu::SkillFactory::get().loadSkill("MoveHome", {queue});
        moveHomeSkill->execute();

        auto handBlockingSkill = kukadu::SkillFactory::get().loadSkill("PushHandPos", {hand});
        handBlockingSkill->execute();

        auto kinect = KUKADU_DYNAMIC_POINTER_CAST<Kinect>(getUsedHardware()[2]);
        hand->install();
        hand->start();
        auto localizer = std::make_shared<PCBlobDetector>(kinect, getStorage(), "origin",
                                                          stdToArmadilloVec({0.7, 0.3, 0.04}), 0.3, 0.4, true);


        auto response = localizer->estimatePose("");

        vec rpy = quatToRpy(response.first.orientation);

        double roll = rpy(0);
        double pitch = rpy(1);
        double yaw = rpy(1);

        // yaw += M_PI / 2.0;

        double bookOrientation = computeBookOrientation(yaw);

        double length = response.second(0) + 0.02;
        double width = response.second(1) + 0.02;
        double height = response.second(2);

        vec bookLocation = stdToArmadilloVec(
                {response.first.position.x, response.first.position.y, response.first.position.z});

        // correction for shifted frame
        bookLocation(0) = bookLocation(0) - 0.41;
        bookLocation(1) = bookLocation(1) + 0.42;
        bookLocation(2) = -0.03;

        cout << "book with dimensions (length = " << length << ", width = " << width << ", height = " << height
             << ") at position (" <<
             bookLocation(0) << ", " << bookLocation(1) << ", " << bookLocation(2) << ") with rotation of " <<
             bookOrientation << " rad found - it should be at (x = " << pushTo(0) << ", y = " << pushTo(1) << ")"
             << endl;

        auto quadrantInfo = checkQuadrant(yaw);
        yaw = quadrantInfo.first;

        struct skill::rectAlignment rectAligned;

        cout << yaw << endl;
        if ((yaw >= (7.0 * M_PI / 4.0) || yaw <= (M_PI / 4.0)) ||
            (yaw >= (3.0 * M_PI / 4.0) && yaw <= (5.0 * M_PI / 4.0)))
            rectAligned.alignmentType = VERTICAL_ALIGNMENT;
        else
            rectAligned.alignmentType = HORICONTAL_ALIGNMENT;

        cout << "alignment type: " << ((rectAligned.alignmentType == HORICONTAL_ALIGNMENT) ? "horicontal" : "vertical")
             << endl;

        double distance = 0.0;
        bool finishedPushing = false;

        while (!finishedPushing) {

            double horicontalDistance = bookLocation(1) - pushTo(1);

            /*
            if(horicontalDistance > 0.02)
                horicontalDistance -= 0.02;
            else if(horicontalDistance < -0.02)
                horicontalDistance += 0.02;
                */

            double verticalDistance = bookLocation(0) - pushTo(0);

            int pushDirection;
            if (abs(horicontalDistance) > Y_TRANSLATION_TOLERANCE)
                pushDirection = (horicontalDistance > 0) ? PUSH_RIGHT : PUSH_LEFT;
            else if (abs(verticalDistance) > X_TRANSLATION_TOLERANCE) {

                pushDirection = (verticalDistance > 0) ? PUSH_DOWN : PUSH_UP;

                // vertical direction ignored for now (that is done by the final push controller)
                // if(pushDirection == PUSH_DOWN)
                finishedPushing = true;

            } else
                finishedPushing = true;

            // compute again with some offset (very dirty, but thats just how it is right now)
            if (bookLocation(1) > pushTo(1))
                bookLocation(1) += 0.07;
            else if (bookLocation(1) < pushTo(1))
                bookLocation(1) -= 0.07;
            horicontalDistance = bookLocation(1) - pushTo(1);
            if (horicontalDistance > 0.02)
                horicontalDistance -= 0.03;
            else if (horicontalDistance < -0.02)
                horicontalDistance += 0.03;

            if (!finishedPushing) {

                geometry_msgs::Pose startPoseWithOffset = pushStartPose(pushDirection, bookLocation, length, width,
                                                                        height,
                                                                        rectAligned.alignmentType, 0.12,
                                                                        0.149 + 0.3 + 0.07, 0.03);

                geometry_msgs::Pose startPoseWoHeightOffset = pushStartPose(pushDirection, bookLocation, length, width,
                                                                            height,
                                                                            rectAligned.alignmentType, 0.12, 0.3, 0.03);

                geometry_msgs::Pose startPose = pushStartPose(pushDirection, bookLocation, length, width, height,
                                                              rectAligned.alignmentType, 0.12, 0.3, 0.0);


                auto cartesianPtpSkill = kukadu::SkillFactory::get().loadSkill("CartesianPtp", {queue});

                std::dynamic_pointer_cast<kukadu::CartesianPtp>(cartesianPtpSkill)->setCartesians(startPoseWithOffset);
                cartesianPtpSkill->execute();

                std::dynamic_pointer_cast<kukadu::CartesianPtp>(cartesianPtpSkill)->setCartesians(
                        startPoseWoHeightOffset);
                cartesianPtpSkill->execute();

                geometry_msgs::Pose lastPose = startPose;

                // first set the appropriate distance, then push and then update location (no success checking here)
                switch (pushDirection) {
                    case PUSH_RIGHT:
                        cout << "pushing right" << endl;
                        distance = abs(horicontalDistance) + 0.00;
                        lastPose = executePush(startPose, distance, pushDirection);
                        bookLocation(1) = pushTo(1);
                        break;
                    case PUSH_LEFT:
                        cout << "pushing left" << endl;
                        distance = abs(horicontalDistance) - 0.00;
                        lastPose = executePush(startPose, distance, pushDirection);
                        bookLocation(1) = pushTo(1);
                        break;
                    case PUSH_UP:
                        cout << "pushing up" << endl;
                        distance = abs(verticalDistance);
                        lastPose = executePush(startPose, distance, pushDirection);
                        bookLocation(0) = pushTo(0);
                        break;
                    case PUSH_DOWN:
                        cout << "pushing down" << endl;
                        distance = abs(verticalDistance);
                        lastPose = executePush(startPose, distance, pushDirection);
                        bookLocation(0) = pushTo(0);
                        break;
                }

                lastPose.position.z += 0.1;
                std::dynamic_pointer_cast<kukadu::CartesianPtp>(cartesianPtpSkill)->setCartesians(lastPose);
                cartesianPtpSkill->execute();
            }

        }

        return nullptr;
    }

    geometry_msgs::Pose
    PushTranslation::pushStartPose(int pushDirection, arma::vec bookLocation, double bookLength, double bookWidth,
                                   double bookHeight, int bookAlignment,
                                   double fingerPalmOffset, double zOffset, double desiredBookDistance) {

        geometry_msgs::Pose handPose;

        vec bookOffset;

        // not very efficient written, but should work out
        if (bookAlignment == HORICONTAL_ALIGNMENT) {

            if (pushDirection == PUSH_LEFT) {

                // standard offset for pushing to the right (seen from the robot)
                bookOffset = stdToArmadilloVec(
                        createJointsVector(3, 0.0, -bookLength / 2.0 + fingerPalmOffset - desiredBookDistance,
                                           zOffset));

                tf::Quaternion rot = rpyToQuat(-M_PI / 2.0, M_PI / 2.0, 0.0);
                handPose.orientation.x = rot.getX();
                handPose.orientation.y = rot.getY();
                handPose.orientation.z = rot.getZ();
                handPose.orientation.w = rot.getW();

            } else if (pushDirection == PUSH_RIGHT) {

                // standard offset for `` to the right (seen from the robot)
                bookOffset = stdToArmadilloVec(
                        createJointsVector(3, 0.0, bookLength / 2.0 + fingerPalmOffset + desiredBookDistance, zOffset));

                tf::Quaternion rot = rpyToQuat(-M_PI / 2.0, M_PI / 2.0, 0.0);
                handPose.orientation.x = rot.getX();
                handPose.orientation.y = rot.getY();
                handPose.orientation.z = rot.getZ();
                handPose.orientation.w = rot.getW();

            } else if (pushDirection == PUSH_DOWN) {

                // standard offset for pushing towards the robot body (down)
                bookOffset = stdToArmadilloVec(
                        createJointsVector(3, bookWidth / 2.0 - fingerPalmOffset + desiredBookDistance, 0.0, zOffset));

                //tf::Quaternion rot = rpyToQuat(M_PI, 0.0, -M_PI / 2.0);
                tf::Quaternion rot = rpyToQuat(M_PI, M_PI, -M_PI / 2.0);
                handPose.orientation.x = rot.getX();
                handPose.orientation.y = rot.getY();
                handPose.orientation.z = rot.getZ();
                handPose.orientation.w = rot.getW();

            } else if (pushDirection == PUSH_UP) {

                // standard offset for pushing away from the robot body (up)
                bookOffset = stdToArmadilloVec(
                        createJointsVector(3, -bookWidth / 2.0 + fingerPalmOffset - desiredBookDistance, 0.0, zOffset));

                tf::Quaternion rot = rpyToQuat(M_PI, 0.0, -M_PI / 2.0);
                handPose.orientation.x = rot.getX();
                handPose.orientation.y = rot.getY();
                handPose.orientation.z = rot.getZ();
                handPose.orientation.w = rot.getW();

            } else {
                throw "(PushingTranslationController) push direction not known";
            }

        } else {

            if (pushDirection == PUSH_LEFT || pushDirection == PUSH_RIGHT) {

                // standard offset for pushing to the right (seen from the robot)
                if (pushDirection == PUSH_LEFT)
                    bookOffset = stdToArmadilloVec(
                            createJointsVector(3, 0.0, -bookWidth / 2.0 + fingerPalmOffset - desiredBookDistance,
                                               zOffset));
                else
                    bookOffset = stdToArmadilloVec(
                            createJointsVector(3, 0.0, bookWidth / 2.0 + fingerPalmOffset + desiredBookDistance,
                                               zOffset));

                tf::Quaternion rot = rpyToQuat(-M_PI / 2.0, M_PI / 2.0, 0.0);
                handPose.orientation.x = rot.getX();
                handPose.orientation.y = rot.getY();
                handPose.orientation.z = rot.getZ();
                handPose.orientation.w = rot.getW();

            } else if (pushDirection == PUSH_DOWN) {

                // standard offset for pushing towards the robot body (down)
                bookOffset = stdToArmadilloVec(
                        createJointsVector(3, bookLength / 2.0 - fingerPalmOffset + desiredBookDistance, 0.0, zOffset));

                //tf::Quaternion rot = rpyToQuat(M_PI, 0.0, -M_PI / 2.0);
                tf::Quaternion rot = rpyToQuat(M_PI, M_PI, -M_PI / 2.0);
                handPose.orientation.x = rot.getX();
                handPose.orientation.y = rot.getY();
                handPose.orientation.z = rot.getZ();
                handPose.orientation.w = rot.getW();

            } else if (pushDirection == PUSH_UP) {

                // standard offset for pushing away from the robot body (up)
                bookOffset = stdToArmadilloVec(
                        createJointsVector(3, -bookLength / 2.0 + fingerPalmOffset - desiredBookDistance, 0.0,
                                           zOffset));

                tf::Quaternion rot = rpyToQuat(M_PI, 0.0, -M_PI / 2.0);
                handPose.orientation.x = rot.getX();
                handPose.orientation.y = rot.getY();
                handPose.orientation.z = rot.getZ();
                handPose.orientation.w = rot.getW();

            } else {
                throw "(PushingTranslationController) push direction not known";
            }


        }

        vec finalPalmPosition = bookLocation + bookOffset;
        handPose.position.x = finalPalmPosition(0);
        handPose.position.y = finalPalmPosition(1);
        handPose.position.z = finalPalmPosition(2);

        return handPose;

    }


    std::string PushTranslation::getClassName() {
        return "PushTranslation";
    }

    void PushTranslation::createSkillFromThisInternal(std::string skillName) {
        // nothing to do
    }

    double PushTranslation::computeBookOrientation(double angle) {

        double yaw = angle;

        yaw = fmod(yaw, M_PI);
        cout << "yaw after first calculation: " << yaw << endl;
        if (yaw > M_PI / 2.0)
            yaw = yaw - M_PI;

        return yaw;

    }

    std::pair<double, int> PushTranslation::checkQuadrant(double alpha) {

        // normalize it
        alpha = fmod(alpha, (2.0 * M_PI));

        // shift it to positive
        alpha += 2.0 * M_PI;

        // normalize again
        alpha = fmod(alpha, (2.0 * M_PI));

        if (alpha >= 0 && alpha < (M_PI / 2.0))
            return {alpha, 0};
        else if (alpha >= (M_PI / 2.0) && alpha < M_PI)
            return {alpha, 1};
        else if (alpha >= M_PI && alpha < (3.0 / 2.0 * M_PI))
            return {alpha, 2};
        else if (alpha >= (3.0 / 2.0 * M_PI) && alpha < (2.0 * M_PI))
            return {alpha, 3};

        throw KukaduException("(PushControllers.checkQuadrant) internal logic error");

    }

    geometry_msgs::Pose PushTranslation::executePush(geometry_msgs::Pose startPose, double distance, int direction) {

        jumpingStep = 0.01;

        std::vector<vec> pushJointPlan;
        std::vector<geometry_msgs::Pose> pushCartIntermedPlan;

        auto queue = KUKADU_DYNAMIC_POINTER_CAST<ControlQueue>(getUsedHardware()[0]);
        queue->install();
        queue->start();

        auto cartesianPtpSkill = kukadu::SkillFactory::get().loadSkill("CartesianPtp", {queue});

        std::dynamic_pointer_cast<kukadu::CartesianPtp>(cartesianPtpSkill)->setCartesians(startPose);
        cartesianPtpSkill->execute();

        geometry_msgs::Pose lastCart = startPose;
        for(double i = 0.0; i < distance; i += std::min(jumpingStep, std::abs(distance - i))) {

            if(direction == PUSH_DOWN)
                startPose.position.x -= jumpingStep;
            else if(direction == PUSH_UP)
                startPose.position.x += jumpingStep;
            else if(direction == PUSH_LEFT)
                startPose.position.y += jumpingStep;
            else if(direction == PUSH_RIGHT)
                startPose.position.y -= jumpingStep;

            pushCartIntermedPlan.push_back(startPose);

        }

        pushJointPlan = komoPlanner->planCartesianTrajectory(pushCartIntermedPlan, true);

        queue->setNextTrajectory(pushJointPlan);
        queue->synchronizeToQueue(1);

        return lastCart;

    }
}

