#include <kukadu/types/controllerresult.hpp>

using namespace std;

namespace kukadu {

    ControllerResult::ControllerResult(arma::vec t, std::vector<arma::vec> ys, bool success) {
        this->t = t;
        this->y = ys;
        this->success = success;
    }

    arma::vec ControllerResult::getTimes() {
        return t;
    }

    std::vector<arma::vec> ControllerResult::getYs() {
        return y;
    }

    void ControllerResult::setSuccess(bool success) {
        this->success = success;
    }

    bool ControllerResult::getSuccess() {
        return success;
    }

    arma::vec ControllerResult::getYCol(int colIdx) {

        arma::vec retVec(y.size());
        for(int i = 0; i < y.size(); ++i)
            retVec(i) = (y.at(i))(colIdx);
        return retVec;

    }

}
