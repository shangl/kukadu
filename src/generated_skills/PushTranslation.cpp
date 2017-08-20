#include <kukadu/generated_skills/PushTranslation.hpp>

namespace kukadu {
    PushTranslation::PushTranslation(kukadu::StorageSingleton &storage,
                                     std::vector<KUKADU_SHARED_PTR<kukadu::Hardware> > hardware)
            : Controller(storage, "PushTranslation", hardware, 0.01) {
        this->pushForward = false;
    }

    bool PushTranslation::requiresGraspInternal() {
        return false;
    }

    bool PushTranslation::producesGraspInternal() {
        return false;
    }
	void PushTranslation::setPushForward(bool pushForward) {
        this->pushForward = pushForward;
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

        auto kinect = KUKADU_DYNAMIC_POINTER_CAST<Camera>(getUsedHardware()[2]);
        hand->install();
        hand->start();
       // auto localizer = std::make_shared<PCBlobDetector>(kinect, getStorage(), "origin", stdToArmadilloVec({0.7, 0.3, 0.04}), 0.3, 0.4, true);

        //auto response = localizer->estimatePose("");

        /*vec rpy = quatToRpy(response.first.orientation);

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

        struct rectAlignment rectAligned;

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
*/
            /*
            if(horicontalDistance > 0.02)
                horicontalDistance -= 0.02;
            else if(horicontalDistance < -0.02)
                horicontalDistance += 0.02;
                */
/*
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

                ptpWithSimulation(startPoseWithOffset, executionQueue, executionQueue, useReal);
                ptpWithSimulation(startPoseWoHeightOffset, executionQueue, executionQueue, useReal);

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
                ptpWithSimulation(lastPose, executionQueue, executionQueue, useReal);

            }

        }*/

        return nullptr;
    }

    std::string PushTranslation::getClassName() {
        return "PushTranslation";
    }

    void PushTranslation::createSkillFromThisInternal(std::string skillName) {
        // nothing to do
    }
}
