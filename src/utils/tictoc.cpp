#include <kukadu/utils/tictoc.hpp>

#include <math.h>
#include <algorithm>
#include <sys/time.h>

using namespace std;

namespace kukadu {

    void TicToc::tic(std::string str) {

        timeval curTime;
        gettimeofday(&curTime, NULL);
        vector<string>::iterator iter;
        iter = find(strList.begin(), strList.end(), str);
        int idx = iter - strList.begin();
        if (idx == strList.size()) {
                strList.push_back(str);
                ticList.push_back(curTime);
                tocList.push_back(curTime);
        }
        ticList[idx] = curTime;

    }

    double TicToc::toc(std::string str) {

        timeval curTime;
        gettimeofday (&curTime, NULL);
        vector<string>::iterator iter;
        iter = find(strList.begin(), strList.end(), str);
        int idx = iter - strList.begin();
        if (idx == strList.size()){
                strList.push_back(str);
                ticList.push_back(curTime);
                tocList.push_back(curTime);
        }
        tocList[idx] = curTime;
        double diff = (double (tocList[idx].tv_sec - ticList[idx].tv_sec ) + double (tocList[idx].tv_usec - ticList[idx].tv_usec ) * 1.e-6 );
        diff =  ceilf(diff * 10000) / 10000;
        return diff;

    }

}
