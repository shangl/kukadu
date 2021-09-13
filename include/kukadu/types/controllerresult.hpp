#ifndef KUKADU_CONTROLLER_RESULT_H
#define KUKADU_CONTROLLER_RESULT_H

#include <vector>
#include <armadillo>

namespace kukadu {

    class ControllerResult {

    private:

        bool success;

        arma::vec t;
        std::vector<arma::vec> y;

    public:

        ControllerResult(arma::vec t, std::vector<arma::vec> ys, bool success);

        arma::vec getTimes();
        std::vector<arma::vec> getYs();

        arma::vec getYCol(int colIdx);

        void setSuccess(bool success);

        bool getSuccess();

        // needs one virtual method in order to make ControllerResult polymorphic (required for dynamic pointer cast)
        virtual ~ControllerResult() { }

    };

}

#endif // CONTROLLER_RESULT_H
