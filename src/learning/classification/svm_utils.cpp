/*
Author: Andrej Karpathy (http://cs.stanford.edu/~karpathy/)
1 May 2012
BSD licence
*/

#include <set>
#include <iostream>
#include <kukadu/types/kukadutypes.hpp>
#include <kukadu/learning/classification/eigenlibsvm/svm_utils.h>

#define Malloc(type,n) (type *)malloc((n)*sizeof(type))

using namespace std;
using namespace kukadu;

namespace esvm {

    SVMClassifier::SVMClassifier(int svmType, int kernelType) {

        // Initialize parameters (defaults taken from libsvm code)
        param_= new svm_parameter();
        //param_->svm_type = svmType;
        param_->kernel_type = kernelType;
        param_->svm_type = svmType;
        param_->degree = 3;
        param_->gamma = 0;
        param_->coef0 = 0;
        param_->nu = 0.5;
        param_->cache_size = 100;
        param_->C = 5.0;
        param_->eps = 0.1;
        param_->p = 0.1;
        param_->shrinking = 1;
        param_->probability = 0;
        param_->nr_weight = 0;
        param_->weight_label = NULL;
        param_->weight = NULL;

        model_ = NULL;
        problem_ = NULL;
        x_space_ = NULL;

        D_ = 0;

    }

    SVMClassifier::~SVMClassifier() {

        svm_destroy_param(param_); //frees weight, weight_label
        free(param_);
        if(x_space_ != NULL) free(x_space_);

        // free model, if it exists
        if(model_ != NULL) svm_free_model_content(model_);

    }

    void SVMClassifier::train(const arma::mat& X, const arma::mat& y) {

        // convert to vector and call real train
        int N = y.size();
        vector<int> yc(N, 0);
        for(int i = 0; i < N; i++)
            yc[i]= (int) y(i);

        train(X, yc);

    }

    void SVMClassifier::train(const arma::mat& X, const vector<int>& y) {

        // find the number of classes
        std::set<int> s;
        for(auto& cl : y)
            s.insert(cl);
        classCount = s.size();

        int Ntr = X.n_rows;
        int D = X.n_cols;

        // Clean up memory allocated in the previous calls to train
        if(model_ != NULL) {
            svm_free_model_content(model_);
            model_ = NULL;
        }

        if(x_space_ != NULL) {
            free(x_space_);
            x_space_ = NULL;
        }

        // Preallocate structures for transformation
        problem_ = new svm_problem();
        problem_->l = Ntr;
        problem_->y = Malloc(double, Ntr);
        problem_->x = Malloc(struct svm_node*, Ntr);
        x_space_= Malloc(struct svm_node, Ntr * (D + 1)); // D + 1 because of libsvm packing conventions
        D_= D; //store dimension of data

        // Process data from X matrix appropriately
        int j = 0;
        for(int i = 0; i < Ntr; i++) {

            problem_->x[i] = &x_space_[j];

            int k = 0;
            while(k < D){

                x_space_[j].index= k + 1; //features start at 1
                x_space_[j].value= X(i, k);

                k++;
                j++;

            }

            x_space_[j].index = -1; // Dark magic of inserting a -1 after every instance. Not sure what this is
            j++;

        }

        // Copy over data from label vector y
        for(int i = 0; i < Ntr; i++)
            problem_->y[i]= (double) y[i];

        // Make sure everything went ok in conversion
        const char *error_msg;
        error_msg = svm_check_parameter(problem_, param_);
        if(error_msg)
            throw KukaduException("(SVMClassifier) parameters are wrong");
        else
            // Train model!
            model_ = svm_train(problem_, param_);

        // free memory for the problem data
        free(problem_->y);
        free(problem_->x);

    }

    void SVMClassifier::test(const arma::mat& X, vector<int>& yhat) {

        // Do simple error checking
        int n = X.n_rows;
        int dim = X.n_cols;

        if(model_ == NULL)
            throw KukaduException("(SVMClassifier) svm not trained yet");

        // Carry out the classification
        yhat.resize(n);

        struct svm_node* node = (struct svm_node*) malloc(dim * sizeof(struct svm_node));
        double* prob_estimates = (double*) malloc(classCount * sizeof(double));

        for(int i = 0; i < n; i++) {

            for(int j = 0; j < dim; j++) {

                node[j].index = j + 1;
                node[j].value = X(i, j);

            }

            double predict_label;

            if(param_->probability == 1)
                predict_label = svm_predict_probability(model_, node, prob_estimates);
            else
                predict_label = svm_predict(model_, node);

            yhat.at(i) = (int) predict_label;

        }

        free(node);
        free(prob_estimates);

    }
  
    void SVMClassifier::getw(arma::mat &w, float &b) {

        if(model_ == NULL)
            throw KukaduException("(SVMClassifier) svm not trained yet");

        b = model_->rho[0]; //libsvm stores -b in rho

        // w is just linear combination of the support vectors when kernel is linear
        w.resize(D_, 1);
        for(int j=0;j<D_;j++) {
            double acc = 0;
            for(int i = 0; i < model_->l; i++) {
                acc += model_->SV[i][j].value * model_->sv_coef[0][i];
            }
            w(j) = -acc;
        }

    }

    int SVMClassifier::saveModel(const char *filename) {
        throw KukaduException("(SVMClassifier) not supported yet");
    }

    void SVMClassifier::loadModel(const char *filename) {
        throw KukaduException("(SVMClassifier) not supported yet");
    }

    void SVMClassifier::setC(double Cnew) {
        param_->C= Cnew;
    }

}
