//////////////////////////////////////////////////////////////////////////////////////////
//																						//
//	The MIT License (MIT)																//
//																						//
//	Copyright (c) 2015 Pablo Ramon Soria												//
//																						//
//	Permission is hereby granted, free of charge, to any person obtaining a copy		//
//	of this software and associated documentation files (the "Software"), to deal		//
//	in the Software without restriction, including without limitation the rights		//
//	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell			//
//	copies of the Software, and to permit persons to whom the Software is				//
//	furnished to do so, subject to the following conditions:							//
//																						//
//	The above copyright notice and this permission notice shall be included in all		//
//	copies or substantial portions of the Software.										//
//																						//
//	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR			//
//	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,			//
//	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE			//
//	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER				//
//	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,		//
//	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE		//
//	SOFTWARE.																			//
//																						//
//////////////////////////////////////////////////////////////////////////////////////////

#include <cassert>
#include <algorithm>
#include <functional>
#include <fstream>
#include <kukadu/learning/classification/svmpp.hpp>

namespace svmpp {
	//-----------------------------------------------------------------------------------------------------------------
	// Train struct
	//-----------------------------------------------------------------------------------------------------------------
	void TrainSet::addEntry(double * _x, unsigned _dimension, double _y) {
		addEntry(std::vector<double>(_x, _x+_dimension), _y);
	}

	//-----------------------------------------------------------------------------------------------------------------
	void TrainSet::addEntry(const std::vector<double>& _x, double _y) {
		mX.push_back(_x);
		mY.push_back(_y);
	}

	//-----------------------------------------------------------------------------------------------------------------
	void TrainSet::addEntries(const std::vector<std::vector<double>>& _X, const std::vector<double>& _Y){
		mX.insert(mX.end(), _X.begin(), _X.end());
		mY.insert(mY.end(), _Y.begin(), _Y.end());
	}

	//-----------------------------------------------------------------------------------------------------------------
	std::vector<double> TrainSet::labels() const {
		return mY;
	}

	//-----------------------------------------------------------------------------------------------------------------
	TrainSet::Problem TrainSet::problem() const {
		Problem problem;
		
		assert(mX.size() == mY.size());

		// Set size of train set.
		problem.l = mX.size();

		// Set labels of data
		problem.y = new double[problem.l];
		for (unsigned i = 0; i < mY.size(); i++) {
			problem.y[i] = mY[i];
		}

		// Set data
		unsigned dims = mX[0].size();
		problem.x = new svm_node*[problem.l];
		for (unsigned i = 0; i < mX.size(); i++) {
			problem.x[i] = new svm_node[dims+1];
			int filledIndex = 0;
			for (unsigned j = 0; j < dims; j++) {
				if (mX[i][j] != 0) {
					problem.x[i][filledIndex].index = j;
					problem.x[i][filledIndex].value = mX[i][j];
					filledIndex++;
				}
			}
			problem.x[i][filledIndex].index = -1;
		}

		return problem;
	}


	//-----------------------------------------------------------------------------------------------------------------
	// Query struct
	//-----------------------------------------------------------------------------------------------------------------
	Query::Query(double * _x, unsigned _dimension): Query(std::vector<double>(_x, _x + _dimension)) {
	}

	//-----------------------------------------------------------------------------------------------------------------
	Query::Query(const std::vector<double>& _x) {
		unsigned dims = _x.size();
		mNode = new svm_node[dims+1];
		int filledIndex=0;
		for (unsigned i = 0; i < dims; i++) {
			if (_x[i] != 0) {
				mNode[filledIndex].index = i;
				mNode[filledIndex].value = _x[i];
				filledIndex++;
			}
		}
		mNode[filledIndex].index = -1;
	}

	//-----------------------------------------------------------------------------------------------------------------
	Query::Node Query::node() const {
		return mNode;
	}

	//-----------------------------------------------------------------------------------------------------------------
	// Svm class
	//-----------------------------------------------------------------------------------------------------------------

    void print_null(const char *s) {}

    Svm::Svm() {
        // force libsvm to shut up ;)
        svm_set_print_string_function(&print_null);
    }

	bool Svm::save(std::string _file) const {
		return svm_save_model(_file.c_str(), mModel) != -1;
	}

	//-----------------------------------------------------------------------------------------------------------------
	bool Svm::load(std::string _file) {
		mModel = svm_load_model(_file.c_str());
		return mModel != nullptr;
    }

	//-----------------------------------------------------------------------------------------------------------------
	void Svm::train(const Params & _params, const TrainSet & _trainSet) {
		mParams = _params;
		auto problem = _trainSet.problem();
		mModel = svm_train(&problem, &_params);
	}

	static std::ofstream sAutoTrainFile;
	//-----------------------------------------------------------------------------------------------------------------
	void Svm::trainAuto(const TrainSet & _trainSet, const Params & _initialParams, const std::vector<ParamGrid> &_paramGrids) {
		Params best;
		sAutoTrainFile.open("svmpp_autotrain_log.txt");
		recursiveTrain(_trainSet, _paramGrids, _initialParams, best);

		train(best, _trainSet);
	}

	//-----------------------------------------------------------------------------------------------------------------
	double Svm::crossValidation(const Params & _params, const TrainSet & _trainSet, int _nFolds) {
		double *labels = new double[_trainSet.labels().size()];
		auto problem = _trainSet.problem();
		svm_cross_validation(&problem, &_params, _nFolds, labels);

		double successRate = 0;
		auto groundTruth = _trainSet.labels();
		for (unsigned i = 0; i < groundTruth.size(); i++) {
			if(labels[i] == groundTruth[i])
				successRate++;
		}
		
		return successRate/groundTruth.size();
	}

	//-----------------------------------------------------------------------------------------------------------------
	double Svm::predict(const Query & _query) const {
		return svm_predict(mModel, _query.node());
	}

	//-----------------------------------------------------------------------------------------------------------------
	double Svm::predict(const Query & _query, std::vector<double> &_probs) const {
		assert(hasProbabilities());
		double *probs = new double[mModel->nr_class];
		svm_predict_probability(mModel, _query.node(), probs);
		
		int maxIndex;
		double maxProb=0;
		for (int i = 0; i < mModel->nr_class;i++) {
			_probs.push_back(probs[i]);
			if (maxProb < probs[i]) {
				maxProb = probs[i];
				maxIndex = i;
			}
		}

		return maxIndex;
	}

	//-----------------------------------------------------------------------------------------------------------------
	bool Svm::hasProbabilities() const {
		return svm_check_probability_model(mModel) == 1;
	}

	//-----------------------------------------------------------------------------------------------------------------
	Svm::Params Svm::params() const {
		return mParams;
	}

	//-----------------------------------------------------------------------------------------------------------------
	// Private Interface
	double Svm::recursiveTrain(const TrainSet &_trainSet, std::vector<ParamGrid> _grids, Params _init, Params & _best) {
		// Get first grid to this level loop.
		ParamGrid grid = _grids[0];

		// Init variables 
		Params bestParams;
		double bestScore = 0;

		// go over params of this grid
		for (double param = grid.min(); param < grid.max();param *= grid.step()) {
			// Set params of this step of the grid
			Params init = _init;
			setParam(init, grid.type(), param);
			Params currentParams;
			double score;

			// If it is not last grid on the list, go one step deeper
			if (_grids.size() != 1) {
				score = recursiveTrain(_trainSet, std::vector<ParamGrid>(_grids.begin()+1, _grids.end()), init, currentParams);
			}
			else {	// Else, train with this parameters
				score = crossValidation(init, _trainSet);
				sAutoTrainFile << "Score: " << score << "\t C: " << init.C << "\t gamma: " << init.gamma << "\t degree: " << init.degree << "\t coef0: " << init.coef0 << std::endl;
				sAutoTrainFile.flush();
				currentParams = init;
			}

			// Get best score and params
			if (score > bestScore) {
				bestScore = score;
				bestParams = currentParams;
			}
		}
		// Save best param on argument and return score.
		_best = bestParams;
		return bestScore;
	}

	//-----------------------------------------------------------------------------------------------------------------
	void Svm::setParam(Params & _params, ParamGrid::Type _type, double _value) {
		switch (_type) {
		case ParamGrid::Type::C:
			_params.C = _value;
			break;
		case ParamGrid::Type::Gamma:
			_params.gamma = _value;
			break;
		case ParamGrid::Type::Degree:
			_params.degree = _value;
			break;
		case ParamGrid::Type::Coeff0:
			_params.coef0 = _value;
			break;
		}
	}

}	// namespace svmpp

