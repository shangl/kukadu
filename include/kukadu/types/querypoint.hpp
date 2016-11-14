#ifndef KUKADU_QUERYPOINT_H
#define KUKADU_QUERYPOINT_H

#include <string>
#include <vector>
#include <armadillo>

#include <kukadu/control/dmp.hpp>
#include <kukadu/types/kukadutypes.hpp>

namespace kukadu {

    class QueryPoint {

    private:

        std::string fileDmpPath;
        std::string fileDataPath;
        std::string fileQueryPath;

        arma::vec queryPoint;

        KUKADU_SHARED_PTR<Dmp> internalDmp;

    public:

        QueryPoint(const QueryPoint& qp);
        QueryPoint(std::string fileQueryPath, std::string fileDataPath, std::string fileDmpPath, KUKADU_SHARED_PTR<Dmp> dmp, arma::vec queryPoint);

        void setDmp(KUKADU_SHARED_PTR<Dmp> dmp);
        void setQueryPoint(arma::vec queryPoint);

        std::string getFileDmpPath();
        std::string getFileDataPath();
        std::string getFileQueryPath();

        arma::vec getQueryPoint();

        KUKADU_SHARED_PTR<Dmp> getDmp();


    };

}

#endif
