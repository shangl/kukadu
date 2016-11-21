#ifndef CONTROLLER_RESULT_H
#define CONTROLLER_RESULT_H

#include <cstdio>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <wordexp.h>
#include <memory>
#include <armadillo>
#include <map>
#include <kukadu/types/kukadutypes.hpp>

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

        void setSuccess(bool success);

        bool getSuccess();

        // needs one virtual method in order to make ControllerResult polymorphic (required for dynamic pointer cast)
        virtual ~ControllerResult() { }

    };

}

#endif // CONTROLLER_RESULT_H
