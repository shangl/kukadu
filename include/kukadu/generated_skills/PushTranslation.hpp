#ifndef KUKADU_GENERATED_SKILLS_PUSHTRANSLATION_H
#define KUKADU_GENERATED_SKILLS_PUSHTRANSLATION_H

#include <kukadu/robot.hpp>
#include <kukadu/control.hpp>
#include <kukadu/planning/komo.hpp>

namespace kukadu {
    class PushTranslation : public kukadu::Controller {

    private:
        double jumpingStep;

        KUKADU_SHARED_PTR<kukadu::Komo> komoPlanner;

        arma::vec pushTo;

        std::pair<double, int> checkQuadrant(double alpha);

        double computeBookOrientation(double angle);

        geometry_msgs::Pose
        pushStartPose(int pushDirection, arma::vec bookLocation, double bookLength, double bookWidth, double bookHeight,
                      int bookAlignment, double fingerPalmOffset, double zOffset, double desiredBookDistance);

        geometry_msgs::Pose executePush(geometry_msgs::Pose startPose, double distance, int direction);

    protected:

        virtual void createSkillFromThisInternal(std::string skillName);

    public:

        PushTranslation(kukadu::StorageSingleton &storage, std::vector<KUKADU_SHARED_PTR<kukadu::Hardware> > hardware);

        bool requiresGraspInternal();

        bool producesGraspInternal();

        std::shared_ptr<kukadu::ControllerResult> executeInternal();

        std::string getClassName();

        void setPushToX(double x);
        void setPushToY(double y);

        static constexpr int HORICONTAL_ALIGNMENT = 0;
        static constexpr int VERTICAL_ALIGNMENT = 1;

        static constexpr double ROTATION_TOLERANCE = 0.3;
        static constexpr double Y_TRANSLATION_TOLERANCE = 0.02;
        static constexpr double X_TRANSLATION_TOLERANCE = 0.02;

        static constexpr int PUSH_UP = 0;
        static constexpr int PUSH_DOWN = 1;
        static constexpr int PUSH_LEFT = 2;
        static constexpr int PUSH_RIGHT = 3;
    };
}

#endif
