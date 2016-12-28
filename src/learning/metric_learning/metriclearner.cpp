#include <kukadu/storage/moduleusagesingleton.hpp>
#include <kukadu/learning/metric_learning/metriclearner.hpp>

using namespace arma;
using namespace std;

namespace kukadu {

    /****************** public functions *******************************/

    Mahalanobis::Mahalanobis() {

        int dim = 1;
        vec dia(dim);
        dia.fill(1.0);

        M = mat(dim, dim);
        M.fill(0.0);
        M.diag() = dia;

    }

    // delivers Euclidean metric
    Mahalanobis::Mahalanobis(int dim) {

        vec dia(dim);
        dia.fill(1.0);

        M = mat(dim, dim);
        M.fill(0.0);
        M.diag() = dia;

    }

    Mahalanobis::Mahalanobis(arma::mat M) {
        setM(M / M(0,0));
    }

    Mahalanobis::Mahalanobis(const Mahalanobis& maha) {
        this->M = maha.M;
    }

    double Mahalanobis::computeSquaredDistance(arma::vec vec1, arma::vec vec2) {

        vec res = (vec1 - vec2).t() * M * (vec1 - vec2);
        return res(0);

    }

    arma::vec Mahalanobis::getCoefficients() {

        vec ret(M.n_cols * M.n_rows);

        // vectorise function is not there as documented, so i have to implement it by myself
        for(int i = 0, k = 0; i < M.n_cols; ++i) {
            for(int j = 0; j < M.n_rows; ++j, ++k) {
                ret(k) = M(i, j);
            }
        }

        return ret;

    }

    void Mahalanobis::setM(arma::mat M) {

        if(M.n_cols != M.n_rows)
            throw KukaduException("(Mahalanobis) not a squared matrix");

        this->M = M;

    }

    arma::mat Mahalanobis::getM() const {
        return M;
    }

    arma::mat Mahalanobis::getDecomposition() {

        KUKADU_MODULE_START_USAGE();

        mat Z;
        mat U, V;
        vec s;

        svd_econ(U, s, V, M);

        for(int i = 0; i < s.n_elem; ++i)
            s(i) = sqrt(s(i));

        mat matS = diagmat(s);

        Z = U * matS;

        KUKADU_MODULE_END_USAGE();

        return Z;

    }

    InfTheoConstraints::InfTheoConstraints() {
    }

    void InfTheoConstraints::addConstraint(arma::vec x1, arma::vec x2, double slack) {

        x1s.push_back(x1);
        x2s.push_back(x2);

        slacks.push_back(slack);
        lambdas.push_back(0.0);

    }

    void InfTheoConstraints::flush() {
        x1s.clear();
        x2s.clear();
        slacks.clear();
        lambdas.clear();
    }

    int InfTheoConstraints::getConstraintCount() {
        return x1s.size();
    }

    arma::vec InfTheoConstraints::getX1(int idx) {
        return x1s.at(idx);
    }

    arma::vec InfTheoConstraints::getX2(int idx) {
        return x2s.at(idx);
    }

    double InfTheoConstraints::getSlack(int idx) {
        return slacks.at(idx);
    }

    double InfTheoConstraints::getLambda(int idx) {
        return lambdas.at(idx);
    }

    void InfTheoConstraints::setSlack(int idx, double slack) {
        slacks.at(idx) = slack;
    }

    void InfTheoConstraints::setLambda(int idx, double lambda) {
        lambdas.at(idx) = lambda;
    }

    InfTheoMetricLearner::InfTheoMetricLearner(std::vector<arma::vec> x1s, std::vector<arma::vec> x2s, std::vector<double> distances, double simBorder, double disSimBorder, double gamma,
                           double divTol, int checkForConCount) : MahalanobisLearner(x1s, x2s, distances) {

        this->simBorder = simBorder;
        this->disSimBorder = disSimBorder;
        this->gamma = gamma;
        this->divTol = divTol;
        this->checkForConCount = checkForConCount;

        int dim = getVectorDim();

        vec dia(dim);
        dia.fill(1.0);

        // euclidean distance is standard matrix
        M0 = mat(dim, dim);
        M0.fill(0.0);
        M0.diag() = dia;

    }

    Mahalanobis InfTheoMetricLearner::learnMetric() {

        KUKADU_MODULE_START_USAGE();

        computeConstraints();

        Mahalanobis metric = getMetric();
        metric.setM(M0);

        int simConCount = simConstraints.getConstraintCount();
        int disSimConCount = disSimConstraints.getConstraintCount();

        vec x1;
        vec x2;
        double slack = 0.0;
        double lambda = 0.0;

        InfTheoConstraints* currentConstSet = NULL;
        int currentIdx = 0;
        int currentSimIdx = 0;
        int currentDisSimIdx = 0;

        double p = 0.0;
        double delta = 0.0;
        double alpha = 0.0;
        double beta = 0.0;

        int i = 0;

        for(i = 0; !checkConvergence(metric.getM()) && (simConCount || disSimConCount); ++i) {

            // choose sim constraint
            if(i % 2 && simConCount) {

                currentConstSet = &simConstraints;
                currentIdx = currentSimIdx = (currentSimIdx + 1) % simConCount;

                // line 3.3
                delta = 1;

            }
            // choose dissim constraint
            else if(!(i %2) && disSimConCount) {
                currentConstSet = &disSimConstraints;
                currentIdx = currentDisSimIdx = (currentDisSimIdx + 1) % disSimConCount;

                // line 3.3
                delta = -1;

            }

            // pick some constraint (line 3.1)
            x1 = currentConstSet->getX1(currentIdx);
            x2 = currentConstSet->getX2(currentIdx);
            slack = currentConstSet->getSlack(currentIdx);
            lambda = currentConstSet->getLambda(currentIdx);

            // line 3.2
            p = metric.computeSquaredDistance(x1, x2);

            // ignore pairs with x1 == x2
            if(p > 0) {

                // line 3.4
                alpha = min(lambda, delta / 2.0 * (1.0 / p - gamma / slack));

                // line 3.5
                beta = delta * alpha / (1.0 - delta * alpha * p);

                // line 3.6
                double nextSlack = gamma * slack / (gamma + delta * alpha * slack);
                currentConstSet->setSlack(currentIdx, nextSlack);

                // line 3.7
                double nextLambda = lambda - alpha;
                currentConstSet->setLambda(currentIdx, nextLambda);

                // line 3.8
                mat currentM = metric.getM();
                currentM = currentM + beta * currentM * (x1 - x2) * (x1 - x2).t() * currentM;
                metric.setM(currentM);

            }

        }

        KUKADU_MODULE_END_USAGE();

        return metric;

    }

    MahalanobisLearner::MahalanobisLearner(std::vector<arma::vec> x1s, std::vector<arma::vec> x2s, std::vector<double> distances) : metric(x1s.at(0).n_elem) {

        int vecSize = x1s.size();

        if(x1s.size() != x2s.size() || distances.size() != vecSize) {
            string error = "(MahalanobisLearner) sizes of x vector collections do not match";
            cerr << error << endl;
            throw KukaduException(error.c_str());
        }

        int xDimension = x1s.at(0).n_elem;
        for(int i = 1; i < x1s.size(); ++i) {
            if(x1s.at(i).n_elem != xDimension || x2s.at(i).n_elem != xDimension) {
                string error = "(MahalanobisLearner) sizes of x vectors do not match";
                cerr << error << endl;
                throw KukaduException(error.c_str());
            }
        }

        this->x1s = x1s;
        this->x2s = x2s;
        this->distances = distances;

    }

    void MahalanobisLearner::addSample(arma::vec x1, arma::vec x2, double distance) {

        if(x1s.size() > 0 && x1s.at(0).n_elem == x1.n_elem && x2s.at(0).n_elem == x2.n_elem) {
            string error = "(MahalanobisLearner) size of x vector does not match";
            cerr << error << endl;
            throw KukaduException(error.c_str());
        }

        distances.push_back(distance);

        x1s.push_back(x1);
        x2s.push_back(x2);

    }

    Mahalanobis MahalanobisLearner::getMetric() {
        return metric;
    }

    int MahalanobisLearner::getVectorDim() {
        return x1s.at(0).n_elem;
    }

    int MahalanobisLearner::getSampleCount() {
        return x1s.size();
    }

    double MahalanobisLearner::getSampleDistance(int idx) {
        return distances.at(idx);
    }

    arma::vec MahalanobisLearner::getX1(int idx) {
        return x1s.at(idx);
    }

    arma::vec MahalanobisLearner::getX2(int idx) {
        return x2s.at(idx);
    }

    TogersonMetricLearner::TogersonMetricLearner(std::vector<arma::vec> x1s, std::vector<arma::vec> x2s, std::vector<double> distances) : MahalanobisLearner(x1s, x2s, distances) {
    }

    Mahalanobis TogersonMetricLearner::learnMetric() {

        KUKADU_MODULE_START_USAGE();

        dim = getVectorDim();
        sampleCount = getSampleCount();


        int iIdx = selectI();

        mat B = generateB(iIdx);
        mat X = generateX();
        mat Y = generateY(B);
        mat Z = generateZ(X, Y);
        mat A = Z.t() * Z;

        // normalize it for convenience
        A = 1 / A(0, 0) * A;
        auto mA = Mahalanobis(A);

        KUKADU_MODULE_END_USAGE();

        return mA;

    }

    /****************** private functions ******************************/

    int InfTheoMetricLearner::checkConvergence(arma::mat currentM) {

        int simConCount = simConstraints.getConstraintCount();
        int disSimConCount = disSimConstraints.getConstraintCount();

        if(oldMetrics.size() < (simConCount + disSimConCount))
            oldMetrics.push_back(currentM);

        lastMetrics.push_back(currentM);

        int metricsCount = oldMetrics.size();
        int lastMetricsCount = lastMetrics.size();

        // use whole date before checking convergence
        if(metricsCount < (simConCount + disSimConCount))
            return 0;

        // use at least checkForConCount metrics to check for convergence
        if(lastMetricsCount <= checkForConCount)
            return 0;

        lastMetrics.erase(lastMetrics.begin());
        for(int i = 0; (i + 1) < checkForConCount; ++i) {

            mat M0 = lastMetrics.at(i);
            mat M1 = lastMetrics.at(i + 1);

            mat diffM = M0 - M1;
            double convDiv = sum(sum(diffM % diffM));

            if(convDiv > divTol)
                return 0;

        }

        return 1;

    }

    void InfTheoMetricLearner::computeConstraints() {

        simConstraints.flush();
        disSimConstraints.flush();

        int sampleCount = getSampleCount();

        for(int i = 0; i < sampleCount; ++i) {

            vec x1 = getX1(i);
            vec x2 = getX2(i);

            double dist = getSampleDistance(i);
            if(dist <= simBorder) {
                simConstraints.addConstraint(x1, x2, simBorder);
            } else if(dist >= disSimBorder) {
                disSimConstraints.addConstraint(x1, x2, disSimBorder);
            } else {
                // no additional constraint
            }

        }

    }

    int TogersonMetricLearner::selectI() {

        // if you change this, generateB and generateX have to be adjusted
        return 0;

    }

    std::vector<int> TogersonMetricLearner::getDRowIdxs(arma::vec x) {

        CustomSet foundXs;
        vector<int> idxs;

        for(int i = 0; i < expandedX1s.size(); ++i) {

            if(compareArmadilloVec(expandedX1s.at(i), x)) {

                if(foundXs.insert(expandedX2s.at(i)).second)
                    idxs.push_back(i);

            }
        }

        return idxs;
    }

    int TogersonMetricLearner::compareArmadilloVec(arma::vec vec1, arma::vec vec2) {

        int retVal = 1;

        if(vec1.n_elem != vec2.n_elem)
            retVal = 0;

        for(int i = 0; i < vec1.n_elem; ++i) {
            if(vec1(i) != vec2(i))
                retVal = 0;
        }

        return retVal;

    }

    void TogersonMetricLearner::expandConstraints() {

        expandedX1s.clear();
        expandedX2s.clear();
        expandedDistances.clear();

        expandedX1s.insert(expandedX1s.end(), x1s.begin(), x1s.end());
        expandedX1s.insert(expandedX1s.end(), x2s.begin(), x2s.end());

        expandedX2s.insert(expandedX2s.end(), x2s.begin(), x2s.end());
        expandedX2s.insert(expandedX2s.end(), x1s.begin(), x1s.end());

        expandedDistances.insert(expandedDistances.end(), distances.begin(), distances.end());

        for(int i = 0; i < expandedX1s.size(); ++i) {
            xsSet.insert(expandedX1s.at(i));
        }

        D = mat(xsSet.size(), xsSet.size());
        D.fill(0.0);

    }

    arma::mat TogersonMetricLearner::generateX() {

        mat X(dim, xsSet.size() - 1);

        for(int i = 1; i < xsSet.size(); ++i) {
            for(int j = 0; j < dim; ++j) {
                vec currentX = xsSet.at(i);
                X(j, i - 1) = currentX(j);
            }
        }

        return X;
    }

    void TogersonMetricLearner::generateD() {

        int xsCount = xsSet.size();
        D = mat(xsCount, xsCount);

        for(int i = 0; i < xsSet.size(); ++i) {

            vec x1 = xsSet.at(i);
            vector<int> idxs = getDRowIdxs(x1);

            if(xsCount != idxs.size()) {

                string msg = "(TogersonMetricLearner) provided distances not complete";
                cerr << msg << endl;
                throw KukaduException(msg.c_str());

            }

            int x1Idx = xsSet.find(x1).second - 1;

            for(int j = 0; j < idxs.size(); ++j) {

                int x2Idx = xsSet.find(expandedX1s.at(idxs.at(j))).second - 1;
                D(x1Idx, j) = distances.at(idxs.at(j));

            }

        }

    }

    arma::mat TogersonMetricLearner::generateB(int iIdx) {

        vec iVec = getX1(iIdx);

        expandConstraints();
        generateD();

        int i = iIdx;

        mat B(D.n_rows - 1, D.n_cols - 1);
        for(int j = 1; j < D.n_rows; ++j) {
            for(int k = 1; k < D.n_cols; ++k) {

                double costheta = ( pow(D(i, j), 2) + pow(D(i, k), 2) - pow(D(j, k), 2) ) / (2 * D(i, j) * D(i, k));
                B(j - 1, k - 1) = D(i, j) * D(i, k) * costheta;

            }
        }

        return B;

    }

    arma::mat TogersonMetricLearner::generateY(arma::mat B) {

        mat Y;
        mat U, V;
        vec s;

        U.fill(0.0);
        V.fill(0.0);
        s.fill(0.0);

        svd(U, s, V, B);

        for(int i = 0; i < s.n_elem; ++i)
            s(i) = sqrt(s(i));

        mat matS = diagmat(s);

        Y = U * matS;

        return Y;

    }

    arma::mat TogersonMetricLearner::generateZ(arma::mat X, arma::mat Y) {

        mat Z = Y.t() * X.t() * inv(X * X.t());
        return Z;

    }

    arma::mat TogersonMetricLearner::generateA() {
    }

    /****************** end ********************************************/

}
