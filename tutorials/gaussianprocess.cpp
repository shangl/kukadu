#include <vector>
#include <memory>
#include <armadillo>
#include <kukadu/kukadu.hpp>

#define DOSIMULATION 1

using namespace std;
using namespace arma;
using namespace kukadu;

int main(int argc, char** args) {

    vector<vec> data;

    vector<double> sampleXs;
    vector<double> sampleYs;

    cout << "producing sample data" << endl;
    vec ts(6);
    for(int i = 0; i < 6; ++i) {
        vec dat(1);
        dat(0) = i + 2;
        data.push_back(dat);

        sampleXs.push_back(i + 2);
    }

    ts(0) = -1.75145; sampleYs.push_back(-1.75145);
    ts(1) = -1.74119; sampleYs.push_back(-1.74119);
    ts(2) = -1.73314; sampleYs.push_back(-1.73314);
    ts(3) = -1.72789; sampleYs.push_back(-1.72789);
    ts(4) = -1.73026; sampleYs.push_back(-1.73026);
    ts(5) = -1.72266; sampleYs.push_back(-1.72266);

    vector<double> xs;
    vector<double> ys;
    KUKADU_SHARED_PTR<GaussianProcessRegressor> reg = make_shared<GaussianProcessRegressor>(data, ts, new GaussianKernel(0.5, 0.05), 10000);
    //	GaussianProcessRegressor* reg = new GaussianProcessRegressor(data, ts, new GaussianKernel(0.5, 1), 10000);
    //	GaussianProcessRegressor* reg = new GaussianProcessRegressor(data, ts, new TricubeKernel(), 1000);
    cout << endl << "do fitting" << endl;
    for(double i = 2.0; i < 7.0; i = i + 0.01) {
        vec query(1);
        query(0) = i;
        double res = reg->fitAtPosition(query)(0);
        xs.push_back(i);
        ys.push_back(res);
    }

}
