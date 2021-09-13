#ifndef KUKADU_METRICLEARNER_H
#define KUKADU_METRICLEARNER_H

#include <armadillo>
#include <kukadu/utils/customset.hpp>
#include <kukadu/types/kukadutypes.hpp>

namespace kukadu {

    class Mahalanobis {

    private:

        arma::mat M;

    public:

        Mahalanobis();
        Mahalanobis(int dim);
        Mahalanobis(arma::mat M);
        Mahalanobis(const Mahalanobis& maha);

        double computeSquaredDistance(arma::vec vec1, arma::vec vec2);

        arma::vec getCoefficients();

        void setM(arma::mat M);
        arma::mat getM() const;
        arma::mat getDecomposition();

    };

    class MahalanobisLearner {

    private:

        Mahalanobis metric;

    protected:

        std::vector<arma::vec> x1s;
        std::vector<arma::vec> x2s;
        std::vector<double> distances;

    public:

        MahalanobisLearner(std::vector<arma::vec> x1s, std::vector<arma::vec> x2s, std::vector<double> distances);

        void addSample(arma::vec x1, arma::vec x2, double distance);

        int getSampleCount();
        int getVectorDim();

        double getSampleDistance(int idx);
        arma::vec getX1(int idx);
        arma::vec getX2(int idx);

        Mahalanobis getMetric();

        virtual Mahalanobis learnMetric() = 0;

    };

    class InfTheoConstraints {

    private:

        std::vector<arma::vec> x1s;
        std::vector<arma::vec> x2s;
        std::vector<double> slacks;
        std::vector<double> lambdas;

    public:

        InfTheoConstraints();

        void addConstraint(arma::vec x1, arma::vec x2, double slack);
        void flush();

        int getConstraintCount();

        arma::vec getX1(int idx);
        arma::vec getX2(int idx);
        double getSlack(int idx);
        double getLambda(int idx);

        void setSlack(int idx, double slack);
        void setLambda(int idx, double lambda);
    };

    // this class implements the method from paper "Information-Theoretic Metric Learning" from davis, kulis, jain, sra and dhillon
    class InfTheoMetricLearner : public MahalanobisLearner {

    private:

        int checkForConCount;

        double divTol;
        double simBorder;
        double disSimBorder;
        double gamma;

        std::vector<arma::mat> lastMetrics;
        std::vector<arma::mat> oldMetrics;

        arma::mat M0;

        InfTheoConstraints simConstraints;
        InfTheoConstraints disSimConstraints;

        void computeConstraints();
        int checkConvergence(arma::mat currentM);

    public:

        InfTheoMetricLearner(std::vector<arma::vec> x1s, std::vector<arma::vec> x2s, std::vector<double> distances, double simBorder, double disSimBorder, double gamma, double divTol, int checkForConCount);

        Mahalanobis learnMetric();

    };

    struct armacomp {

        int operator() (const arma::vec& vec1, const arma::vec& vec2) const {

            if(vec1.n_elem != vec2.n_elem)
                return vec1.n_elem - vec2.n_elem;

            // check equality
            for(int i = 0; i < vec1.n_elem; ++i) {
                if(vec1(i) == vec2(i)) {

                } else if (vec1(i) > vec2(i)) {
                    std::cout << vec1.t() << " " << vec2.t() << "===============" << 1 << "==============" << std::endl;
                    return 1;
                } else {
                    std::cout << vec1.t() << " " << vec2.t() << "===============" << -1 << "==============" << std::endl;
                    return -1;
                }
            }

            std::cout << vec1.t() << " " << vec2.t() << "===============" << 0 << "==============" << std::endl;

            return 0;

        }

    };

    // method according to http://forrest.psych.unc.edu/teaching/p230/Torgerson.pdf
    class TogersonMetricLearner : public MahalanobisLearner {

    private:

        int dim;
        int sampleCount;

        arma::mat D;

        CustomSet xsSet;

        std::vector<arma::vec> expandedX1s;
        std::vector<arma::vec> expandedX2s;
        std::vector<double> expandedDistances;

        void generateD();
        void expandConstraints();

        int selectI();
        int compareArmadilloVec(arma::vec vec1, arma::vec vec2);

        std::vector<int> getDRowIdxs(arma::vec x);

        arma::mat generateA();
        arma::mat generateX();
        arma::mat generateB(int iIdx);
        arma::mat generateY(arma::mat B);
        arma::mat generateZ(arma::mat X, arma::mat Y);

    public:

        TogersonMetricLearner(std::vector<arma::vec> x1s, std::vector<arma::vec> x2s, std::vector<double> distances);

        Mahalanobis learnMetric();

    };

}

#endif
