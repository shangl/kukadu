#include <kukadu/kinematics/kinematics.hpp>
#include <kukadu/utils/utils.hpp>

using namespace std;

namespace kukadu {

    Kinematics::Kinematics(std::vector<std::string> jointNames) {
        Constraints.clear();
        this->jointNames = jointNames;
    }

    std::vector<std::string> Kinematics::generateDefaultJointNames(int jointCount)  {

        std::vector<std::string> jointNames;
        for(int i = 0; i < jointCount; ++i) {
            stringstream s;
            s << "joint" << i;
            jointNames.push_back(s.str());
        }

        return jointNames;

    }

    void Kinematics::setJointNames(std::vector<std::string> jointNames) {
        this->jointNames = jointNames;
    }

    std::vector<std::string> Kinematics::getJointNames() {
        return jointNames;
    }

    void Kinematics::addConstraint(KUKADU_SHARED_PTR<Constraint> Constraint) {
        Constraints.push_back(Constraint);
    }

    void Kinematics::removeConstraint(KUKADU_SHARED_PTR<Constraint> Constraint) {
        std::remove(Constraints.begin(), Constraints.end(), Constraint);
    }

    int Kinematics::getConstraintsCount() {
        return Constraints.size();
    }

    int  Kinematics::getConstraintIdx(KUKADU_SHARED_PTR<Constraint> Constraint) {
        return std::find(Constraints.begin(), Constraints.end(), Constraint) - Constraints.begin();
    }

    KUKADU_SHARED_PTR<Constraint> Kinematics::getConstraintByIdx(int idx) {
        return Constraints.at(idx);
    }

    std::vector<arma::vec> Kinematics::computeIk(arma::vec currentJointState, const geometry_msgs::Pose &goal) {
        return computeIk(armadilloToStdVec(currentJointState), goal);
    }

    bool Kinematics::checkAllConstraints(arma::vec currentState, geometry_msgs::Pose pose) {

        for(int i = 0; i < getConstraintsCount(); ++i) {

            KUKADU_SHARED_PTR<Constraint> currRest = getConstraintByIdx(i);
            if(!currRest->stateOk(currentState, pose))
                return false;

        }

        return true;

    }

}
