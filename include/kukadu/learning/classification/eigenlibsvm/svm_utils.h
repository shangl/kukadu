/*
Author: Andrej Karpathy (http://cs.stanford.edu/~karpathy/)
1 May 2012
BSD licence
*/

#ifndef __EIGEN_SVM_UTILS_H__
#define __EIGEN_SVM_UTILS_H__

#include <string>
#include <vector>
#include <armadillo>
#include <libsvm/svm.h>

namespace esvm {
  
  /* 
  Trains a binary SVM on dense data with linear kernel using libsvm. 
  Usage:
  
  vector<int> yhat;
  SVMClassifier svm;
  svm.train(X, y);
  svm.test(X, yhat);

  where X is an arma::mat that is NxD array. (N D-dimensional data),
  y is a vector<int> or an arma::mat Nx1 vector. The labels are assumed
  to be -1 and 1. This version doesn't play nice if your dataset is 
  too unbalanced.
  */
  class SVMClassifier {

  private:

      svm_model *model_;
      svm_problem *problem_;
      svm_parameter *param_;
      svm_node *x_space_;

      int D_;

      int classCount;

    public:

        SVMClassifier(int svmType = C_SVC, int kernelType = RBF);
        ~SVMClassifier();

        // train the svm
        void train(const arma::mat &X, const std::vector<int> &y);
        void train(const arma::mat &X, const arma::mat &y);

        // test on new data
        void test(const arma::mat &X, std::vector<int> &yhat);

        // libsvm does not directly calculate the w and b, but a set of support
        // vectors. This function will use them to compute w and b, as currenly
        // we assume linear kernel only
        // yhat = sign( X * w + b )
        void getw(arma::mat &w, float &b);

        // I/O
        int saveModel(const char *filename);
        void loadModel(const char *filename);

        // options
        void setC(double Cnew); //default is 1.0

        //TODO: add cross validation support
        //TODO: add probability support?



  };

}


#endif //__EIGEN_SVM_UTILS_H__
