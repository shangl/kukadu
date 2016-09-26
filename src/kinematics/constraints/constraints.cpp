#include <kukadu/kinematics/constraints/constraints.hpp>
#include <kukadu/utils/utils.hpp>

using namespace std;

namespace kukadu {

    MoveItConstraint::MoveItConstraint(robot_model::RobotModelPtr robotModel, planning_scene::PlanningScenePtr planningScene, robot_model::JointModelGroup* modelGroup) {

        this->planningScene = planningScene;
        this->modelGroup = modelGroup;
        this->robotModel = robotModel;

    }

    bool MoveItConstraint::stateOk(arma::vec joint, geometry_msgs::Pose cartPose) {

        moveit::core::RobotState state(robotModel);
        double jointStateArray[joint.n_elem];
        for(int i = 0; i < joint.n_elem; ++i)
            jointStateArray[i] = joint[i];
        state.setJointGroupPositions(modelGroup, jointStateArray);
        state.update();

        /*
        if(planningScene->isStateColliding(state, modelGroup->getName())) {
            vector<string> collidingLinks;
            collision_detection::CollisionRequest requ;
            collision_detection::CollisionResult res;
            requ.contacts = true;
            requ.max_contacts = 10000;
            planningScene->getCollidingLinks(collidingLinks, state);
            planningScene->checkCollision(requ, res, state);

            cout << cartPose << endl;

            for(collision_detection::CollisionResult::ContactMap::iterator it = res.contacts.begin(); it != res.contacts.end(); ++it) {
                cout << "collission: " << it->first.first << " " << it->first.second << endl;
            }

            cout << "colliding links: ";
            for(int i = 0; i < collidingLinks.size(); ++i) {
                cout << collidingLinks.at(i) << endl;
            }

        }
        */

        auto isOk = !planningScene->isStateColliding(state, modelGroup->getName());
        if(!isOk)
            planningScene->isStateColliding(state, modelGroup->getName(), true);

        return isOk;

    }

    std::string MoveItConstraint::getConstraintName() {
        return string("MoveItConstraint");
    }

    bool TableConstraint::stateOk(arma::vec joint, geometry_msgs::Pose cartPose) {

        if(cartPose.position.z < 0) {
            if(cartPose.position.x > 0) {
                return false;
            }
        }

        return true;

    }

    std::string TableConstraint::getConstraintName() {
        return string("TableConstraint");
    }

}
