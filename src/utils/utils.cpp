#include <math.h>
#include <istream>
#include <fstream>
#include <iostream>
#include <iterator>
#include <sys/stat.h>
#include <sys/types.h>
#include <boost/filesystem.hpp>

#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>

#include <kukadu/utils/utils.hpp>
#include <kukadu/types/kukadutypes.hpp>

using namespace std;
using namespace arma;

namespace kukadu {

    double sigmoid(double x) {
        return (1.0 + tanh(x / 2.0)) / 2.0;
    }

    void printPose(const geometry_msgs::Pose& p) {
        cout << p.position.x << " " << p.position.y << " " << p.position.z << " " << p.orientation.x << " " << p.orientation.y << " " << p.orientation.z << " " << p.orientation.w << endl;
    }

    std::pair<int, double> computeMaxJointDistance(arma::vec joints1, arma::vec joints2) {

        int maxIdx = 0;
        double maxDist = 0.0;
        for(int i = 0; i < joints1.n_elem; ++i) {
            double currDist = abs(joints1(i) - joints2(i));
            if(currDist > maxDist) {
                maxDist = currDist;
                maxIdx = i;
            }
        }
        return {maxIdx, maxDist};

    }

    void preparePathString(std::string& s) {
        if(*(s.end()) != '/')
            s.append("/");
    }

    int createDirectory(std::string path) {

        struct stat st = {0};

        if (stat(path.c_str(), &st) == -1) {
            mkdir(path.c_str(), 0700);
            return true;
        } else return false;

    }

    std::string resolvePath(std::string path) {

        wordexp_t p;
        wordexp(path.c_str(), &p, 0 );
        char** w = p.we_wordv;
        string ret = string(*w);
        wordfree( &p );

        return ret;

    }

    arma::vec createArmaVecFromDoubleArray(double* data, int n) {
        arma::vec ret(n);
        for(int i = 0; i < n; ++i)
            ret(i) = data[i];
        return ret;
    }

    /* reimplements the getch() function available on windows
     *
     * returns: sign take from console
     * input: -
    */
    int getch() {
        struct termios oldt, newt;
        int ch;
        tcgetattr( STDIN_FILENO, &oldt );
        newt = oldt;
        newt.c_lflag &= ~( ICANON | ECHO );
        tcsetattr( STDIN_FILENO, TCSANOW, &newt );
        ch = getchar();
        tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
        return ch;
    }

    float* copyJoints(const float* arr, int arrSize) {
        float* ret = new float[arrSize];
        for(int i = 0; i < arrSize; ++i) ret[i] = arr[i];
        return ret;
    }

    void printDoubleVector(vector<double>* data) {
        int dataSize = data->size();
        for(int i = 0; i < dataSize; ++i) {
            cout << data->at(i) << ", ";
        }
        cout << endl;
    }

    std::vector<double>* getDoubleVectorFromArray(double* arr, int size) {

        vector<double>* v = new vector<double>(arr, arr + size);
        return v;

    }

    vec computeDiscreteDerivatives(vec x, vec y) {

        if(x.n_elem != y.n_elem)
            throw KukaduException("(utils.computeDiscreteDerivatives) vector dimensions do not match");

        vec ret(x.n_elem);
        double y0, y1, x0, x1, k;
        for(int i = 1; i < x.n_elem; ++i) {

            y0 = y(i - 1);
            y1 = y(i);
            x0 = x(i - 1);
            x1 = x(i);

            k = (y1 - y0) / (x1 - x0);
            if(k != k)
                throw KukaduException("(utils.computeDiscreteDerivatives) two consecutive data lines have the same time (would resoult in division by 0)");

            ret(i - 1) = k;

        }

        ret(ret.n_elem - 1) = k;

        return ret;

    }

    arma::mat computeDiscreteDerivatives(arma::vec t, arma::mat ys) {

        mat derivatives(ys.n_rows, 1);

        for(int i = 0; i < ys.n_cols; ++i) {
            vec colI = ys.col(i);
            vec vel = computeDiscreteDerivatives(t, colI);
            if(!i)
                derivatives = vel;
            else
                derivatives = join_rows(derivatives, vel);
        }

        return derivatives;

    }

    string buildPolynomialEquation(double* w, int paramCount) {
        string ret = "0";
        for(int i = 0; i < paramCount; ++i) {
            std::ostringstream s;
            s << " + " << w[i] << " * x**" << i;
            ret += s.str();
        }
        return ret;
    }

    double* poly_eval_multiple(double* data, int data_len, double* c, int c_len) {
        double* evals = new double[data_len];
        for(int i = 0; i < data_len; ++i) {
            evals[i] = gsl_poly_eval(c, c_len, data[i]);
        }
        return evals;
    }

    double* polyder(double* c, int len) {
        double* dc = new double[len];
        for(int i = 1; i < len; ++i) {
            dc[i - 1] = c[i] * i;
        }
        dc[len - 1] = 0;
        return dc;
    }

    double* polynomialfit(int obs, int degree, double *dx, double *dy) {

        gsl_multifit_linear_workspace *ws;
        gsl_matrix *cov, *X;
        gsl_vector *y, *c;
        double chisq;
        double* store = new double[degree];

        int i, j;

        X = gsl_matrix_alloc(obs, degree);
        y = gsl_vector_alloc(obs);
        c = gsl_vector_alloc(degree);
        cov = gsl_matrix_alloc(degree, degree);

        // computation of design matrix according to bishop equation 3.16 (page 142)
        for(i = 0; i < obs; ++i) {
            gsl_matrix_set(X, i, 0, 1.0);
            for(j=0; j < degree; j++) {
                gsl_matrix_set(X, i, j, pow(dx[i], j));
            }
            gsl_vector_set(y, i, dy[i]);
        }

        ws = gsl_multifit_linear_alloc(obs, degree);
        gsl_multifit_linear(X, y, c, cov, &chisq, ws);

        for(i = 0; i < degree; ++i) {
            store[i] = gsl_vector_get(c, i);
        }

        gsl_multifit_linear_free(ws);
        gsl_matrix_free(X);
        gsl_matrix_free(cov);
        gsl_vector_free(y);
        gsl_vector_free(c);

        return store;

    }

    float* createFloatArrayFromStdVector(vector<float>* data) {
        int size = data->size();
        float* retArr = new float[size];
        for(int i = 0; i < size; ++i) retArr[i] = data->at(i);
        return retArr;
    }

    double* createDoubleArrayFromStdVector(vector<double>* data) {
        int size = data->size();
        double* retArr = new double[size];
        for(int i = 0; i < size; ++i) retArr[i] = data->at(i);
        return retArr;
    }

    double* createDoubleArrayFromArmaVector(vec data) {
        int size = data.n_elem;
        double* retArr = new double[size];
        for(int i = 0; i < size; ++i) retArr[i] = data(i);
        return retArr;
    }

    double* createDoubleArrayFromVector(gsl_vector* data) {
        int size = data->size;
        double* retArr = new double[size];
        for(int i = 0; i < size; ++i) retArr[i] = gsl_vector_get(data, i);
        return retArr;
    }

    /*
     * transforms gsl_matrix to double** matrix
     * returns: double** representation of matrix
     * input:
     *	gsl_matrix* data:	data in gsl_matrix format
     *
    */
    double** createDoubleArrayFromMatrix(gsl_matrix* data) {
        int rows = data->size1;
        int columns = data->size2;
        double** retArr = new double*[rows];
        for(int i = 0; i < rows; ++i) {
            retArr[i] = new double[columns];
            for(int j = 0; j < columns; ++j) {
                retArr[i][j] = gsl_matrix_get(data, i, j);
            }
        }
        return retArr;
    }

    /*
     * releases allocated memory for double array
     * returns: -
     * input:
     * 	double** data:		double array
     * 	int columns:		number of columns in array
     *
    */
    void freeDoubleArray(double** data, int columns) {
        for(int i = 0; i < columns; ++i) {
            free(data[i]);
        }
        free(data);
    }

    void printDoubleVector(double* data, int size) {
        for(int i = 0; i < size; ++i) {
            cout << data[i] << ",";
        }
        cout << endl;
    }

    /*
     * prints a double matrix to console
     * returens: -
     * input:
     * 	double** data:		matrix
     * 	int rows:		number of rows in matrix
     * 	int columns:		number of columns in matrix
    */
    void printDoubleMatrix(double** data, int rows, int columns) {
        for(int i = 0; i < rows; ++i) {
            for(int j = 0; j < columns; ++j) {
                cout << data[i][j] << ",  ";
            }
            cout << endl;
        }
        fflush(stdout);
    }

    gsl_vector* createGslVectorFromStdVector(std::vector<double>* data) {
        gsl_vector* ret = gsl_vector_alloc(data->size());
        for(int i = 0; i < data->size(); ++i) gsl_vector_set(ret, i, data->at(i));
        return ret;
    }

    std::vector<double>* createStdVectorFromGslVector(gsl_vector* vec) {
        std::vector<double>* ret = new std::vector<double>();
        for(int i = 0; i < vec->size; ++i) {
            ret->push_back(gsl_vector_get(vec, i));
        }
        return ret;
    }

    /*
     * converts a queue array representation to the gsl_matrix format
     * returns: gsl_matrix version of data
     * input:
     * 	std:queue<double>** data:	queue array containing data
     * 	int columns:			number of columns (length of data array)
    */
    gsl_matrix* createMatrixFromQueueArray(std::queue<double>** data, int columns) {
        gsl_matrix* retMatrix = gsl_matrix_alloc(data[0]->size(), columns);
        int numRows = data[0]->size();
        for(int i = 0; i < numRows; ++i) {
            for(int j = 0; j < columns; ++j) {
                double val = data[j]->front();
                data[j]->pop();
                gsl_matrix_set(retMatrix, i, j, val);
            }
        }
        return retMatrix;
    }

    gsl_matrix* invertSquareMatrix(gsl_matrix* mat) {

        //	gsl_linalg_SV_decomp(

        int s;
        int size = mat->size1;
        gsl_matrix* retMatrix = gsl_matrix_alloc(size, size);
        gsl_permutation* p = gsl_permutation_alloc(size);
        gsl_linalg_LU_decomp (mat, p, &s);
        gsl_linalg_LU_invert(mat, p, retMatrix);
        return retMatrix;

    }

    /*
     * reads double numbers from file
     *
     * returns: array of queues, where each queue stores a column
     * input:
     *	char* file:		complete path to input file
     *	int fileColumns:	count of columns to import
    */
    std::pair<std::vector<long long int>, mat> readDmpData(string file, bool removeDuplicates) {

        ifstream inFile;
        inFile.open(file.c_str(), ios::in | ios::app | ios::binary);
        auto retMat = readDmpData(inFile, removeDuplicates);
        inFile.close();

        return retMat;

    }

    bool fileIsEmpty(std::string& filePath) {
        ifstream inFile;
        inFile.open(filePath.c_str(), ios::in | ios::app | ios::binary);
        auto retVal = fileIsEmpty(inFile);
        inFile.close();
        return retVal;
    }

    std::pair<std::vector<long long int>, mat> readDmpData(std::ifstream& inFile, bool removeDuplicates) {

        mat joints;
        vector<long long int> time;

        string line;
        double dn = 0.0;
        long long int t0 = 0;
        long long int currentTime = 0;
        double prevTime = DBL_MIN;
        int fileColumns = 0;

        bool firstIteration = true;

        if (inFile.is_open()) {

            int j = 0;
            while(inFile.good()) {

                // find out how many columns there are
                if(firstIteration) {
                    getline(inFile, line);
                    KukaduTokenizer tok(line);
                    int i = 0;
                    for(i = 0; tok.next() != ""; ++i);
                    fileColumns = i;
                    firstIteration = false;
                    joints = mat(1, fileColumns - 1);
                }

                bool ignoreLine = false;

                // iterate over all columns
                for(int i = 0; i < fileColumns; ++i) {

                    // if its the first column - it is the time, otherwise its a measurement
                    if(i > 0)
                        inFile >> dn;
                    else
                        inFile >> currentTime;

                    // only store the new data, if the time has changed
                    if(i == 0 && prevTime == currentTime && removeDuplicates) {
                        ignoreLine = true;
                        // remove the rest of the line from the buffer
                        getline(inFile, line);
                        break;
                    } else if(i == 0)
                        prevTime = currentTime;

                    // normalization
                    if(j == 0 && i == 0) { t0 = currentTime; currentTime = 0; }
                    else if(i == 0) currentTime -= t0;

                    // if i > 0 it is joint data
                    if(i > 0)
                        joints(j, i - 1) = dn;
                    else
                        time.push_back(currentTime);

                }

                if(!ignoreLine) {

                    j++;
                    joints.resize(j + 1, fileColumns);

                }

            }

            joints.resize(j - 1, fileColumns);
            time.resize(j - 1);

        }

        return {time, joints};

    }

    std::pair<std::vector<std::string>, std::pair<std::vector<long long int>, arma::mat> > readSensorStorage(std::string file) {

        vector<string> labels;
        string token;
        string line;
        ifstream inFile;
        inFile.open(file.c_str(), ios::in | ios::app | ios::binary);
        getline(inFile, line);

        KukaduTokenizer tok(line);
        while((token = tok.next()) != "")
            labels.push_back(token);

        auto data = readDmpData(inFile);
        pair<std::vector<std::string>, std::pair<std::vector<long long int>, arma::mat> > retPair(labels, data);

        inFile.close();
        return retPair;

    }

    vec readQuery(string file) {
        vec ret(1);
        string line;
        string token;
        double dn = 0.0;
        int i = 0;
        ifstream inFile;
        inFile.open(file.c_str(), ios::in | ios::app | ios::binary);
        if (inFile.is_open()) {
            cout << string("(utils) query file ") + file + " opened" << endl;
            if(inFile.good()) {
                getline(inFile, line);
                KukaduTokenizer tok(line);
                while((token = tok.next()) != "") {
                    dn = stod(token);
                    ret(i) = dn;
                    ++i;
                    ret.resize(i + 1);
                }
            }
        }
        ret.resize(i);

        inFile.close();
        return ret;
    }

    vector<string> getFilesInDirectory(string folderPath) {

        DIR *dir;
        struct dirent *ent;
        vector<string> ret;

        if((dir = opendir(folderPath.c_str())) != NULL) {

            while((ent = readdir(dir)) != NULL) {
                string tmp(ent->d_name);
                ret.push_back(tmp);
            }

            closedir (dir);
        }
        return ret;
    }

    vector<string> sortPrefix(vector<string> list, string prefix) {

        vector<string> ret;
        int listSize = list.size();

        for(int i = 0; i < listSize; ++i) {
            string currentString = list.at(i);
            int pos = currentString.find(prefix);
            if(!pos) ret.push_back(currentString);
        }
        return ret;
    }

    mat gslToArmadilloMatrix(gsl_matrix* matrix) {
        int lines = matrix->size1;
        int columns = matrix->size2;
        mat ret = mat(lines, columns);
        for(int i = 0; i < lines; ++i)
            for(int j = 0; j < columns; ++j)
                ret(i, j) = gsl_matrix_get(matrix, i, j);
        return ret;
    }

    vector<double> armadilloToStdVec(vec armadilloVec) {
        vector<double> retVec;
        for(int i = 0; i < armadilloVec.n_elem; ++i)
            retVec.push_back(armadilloVec(i));
        return retVec;
    }

    vec stdToArmadilloVec(vector<double> stdVec) {
        vec ret(stdVec.size());
        for(int i = 0; i < stdVec.size(); ++i) {
            ret(i) = stdVec.at(i);
        }
        return ret;
    }

    arma::vec squareMatrixToColumn(arma::mat Z) {

        vec zCoeffs(Z.n_cols * Z.n_rows);

        int k = 0;
        for(int i = 0; i < Z.n_cols; ++i) {
            for(int j = 0; j < Z.n_rows; ++j) {
                double currVal = Z(i, j);
                zCoeffs(k) = currVal;
                ++k;
            }
        }

        return zCoeffs;

    }

    arma::mat columnToSquareMatrix(arma::vec c) {

        int dim = sqrt(c.n_elem);
        arma::mat newM(dim, dim);

        int k = 0;
        for(int i = 0; i < dim; ++i) {
            for(int j = 0; j < dim; ++j) {
                newM(i, j) = c(k);
                ++k;
            }
        }

        return newM;
    }

    arma::vec symmetricMatrixToColumn(arma::mat Z) {

        vec zCoeffs(Z.n_cols * (Z.n_cols + 1) / 2);

        int k = 0;
        for(int i = 0; i < Z.n_cols; ++i) {
            for(int j = i; j < Z.n_rows; ++j) {
                double currVal = Z(i, j);
                zCoeffs(k) = currVal;
                ++k;
            }
        }

        return zCoeffs;
    }


    arma::mat columnToSymmetricMatrix(arma::vec c) {

        int n = c.n_elem;
        int dim = (sqrt(8 * n + 1) - 1) / 2;
        arma::mat newM(dim, dim);

        int k = 0;
        for(int i = 0; i < dim; ++i) {
            for(int j = i; j < dim; ++j) {
                newM(i, j) = c(k);
                if(i != j)
                    newM(j, i) = c(k);
                ++k;
            }
        }

        return newM;

    }

    std::string stringFromDouble(double d) {

        std::stringstream s;
        s << d;
        return s.str();

    }

    std::pair<arma::vec, arma::mat> fillTrajectoryMatrix(arma::vec timesInSec, arma::mat joints, double tMax) {

        int prevMaxIdx = joints.n_rows;
        double prevTMax = timesInSec(timesInSec.n_rows - 1, 0);

        if(tMax > prevTMax) {
            double tDiff = prevTMax - timesInSec(timesInSec.n_rows - 2, 0);

            int insertSteps = (int) ((tMax - prevTMax) / tDiff);
            joints.resize(joints.n_rows + insertSteps + 1, joints.n_cols);
            timesInSec.resize(joints.n_rows + insertSteps + 1, joints.n_cols);
            for(int i = 0; i < insertSteps; ++i) {
                for(int j = 0; j < joints.n_cols; ++j) {
                    joints(prevMaxIdx + i, j) = joints(prevMaxIdx - 1, j);
                }
                timesInSec(prevMaxIdx + i, 0) = prevTMax + (i + 1) * tDiff;
            }

            if(joints(prevMaxIdx + insertSteps - 1, 0) != tMax) {
                for(int j = 1; j < joints.n_cols; ++j) {
                    joints(prevMaxIdx + insertSteps, j) = joints(prevMaxIdx - 1, j);
                }
                timesInSec(prevMaxIdx + insertSteps, 0) = tMax;
            } else {
                joints.resize(joints.n_rows - 1, joints.n_cols);
                timesInSec.resize(joints.n_rows - 1, joints.n_cols);
            }

        }

        return {timesInSec, joints};

    }

    arma::mat armaJoinRows(arma::vec v1, arma::mat m2) {

        if(v1.n_elem != m2.n_rows)
            throw KukaduException("(armaJoinRows) matrix dimensions do not match");

        arma::mat retMat(m2.n_rows, 1 + m2.n_cols);
        for(int i = 0; i < m2.n_rows; ++i) {
            retMat(i, 0) = v1(i);
            for(int j = 0; j < m2.n_cols; ++j)
                retMat(i, 1 + j) = m2(i, j);
        }

        return retMat;

    }

    arma::mat armaJoinRows(arma::mat m1, arma::mat m2) {

        if(m1.n_rows != m2.n_rows)
            throw KukaduException("(armaJoinRows) matrix dimensions do not match");

        arma::mat retMat(m1.n_rows, m1.n_cols + m2.n_cols);
        for(int i = 0; i < m1.n_rows; ++i) {
            for(int j = 0; j < m1.n_cols; ++j)
                retMat(i, j) = m1(i, j);
            for(int j = 0; j < m2.n_cols; ++j)
                retMat(i, m1.n_cols + j) = m2(i, j);
        }

        return retMat;

    }

    double absolute(double val) {
        return (val >= 0) ? val : -val;
    }

    void set_ctrlc_exit_handler() {
        struct sigaction sigIntHandler;
        sigIntHandler.sa_handler = exit_handler;
        sigemptyset(&sigIntHandler.sa_mask);
        sigIntHandler.sa_flags = 0;
        sigaction(SIGINT, &sigIntHandler, NULL);
    }

    bool fileIsEmpty(std::ifstream& pFile) {
        return pFile.peek() == std::ifstream::traits_type::eof();
    }

    void exit_handler(int s) {

        exit(1);

    }


    geometry_msgs::Pose vectorarma2pose(arma::vec* vectorpose) {

        geometry_msgs:: Pose posepose;
        posepose.position.x = vectorpose->at(0);
        posepose.position.y = vectorpose->at(1);
        posepose.position.z = vectorpose->at(2);
        posepose.orientation.x = vectorpose->at(3);
        posepose.orientation.y = vectorpose->at(4);
        posepose.orientation.z = vectorpose->at(5);
        posepose.orientation.w = vectorpose->at(6);

        return posepose;

    }

    arma::vec pose2vectorarma(geometry_msgs::Pose posepose) {

        vec armapose(7);
        armapose(0) = posepose.position.x;
        armapose(1) = posepose.position.y;
        armapose(2) = posepose.position.z;
        armapose(3) = posepose.orientation.x;
        armapose(4) = posepose.orientation.y;
        armapose(5) = posepose.orientation.z;
        armapose(6) = posepose.orientation.w;
        return armapose;

    }

    arma::vec log(tf::Quaternion quat) {

        vec logQuat(3);
        double modU = sqrt(quat.x() * quat.x()  + quat.y() * quat.y() + quat.z() * quat.z());
        double acosw;

        acosw = acos(quat.w());
         if(std::isnan(acosw)) acosw = 0.0;

        if (modU > 0.0) {

            logQuat(0) = acosw * quat.x() / modU;
            logQuat(1) = acosw * quat.y() / modU;
            logQuat(2) = acosw * quat.z() / modU;

        } else {

            logQuat(0) = 0.0;
            logQuat(1) = 0.0;
            logQuat(2) = 0.0;

        }

        return logQuat;

    }

    tf::Quaternion vecExp(arma::vec logQuat) {

        double x, y, z, w;
        double modR = sqrt(logQuat(0) * logQuat(0) + logQuat(1) * logQuat(1) + logQuat(2) * logQuat(2));
        if (modR > M_PI / 2) cout<< "(utils) mod out of limits "<< modR << endl;

        if (modR > 0) {

            w = cos(modR);
            x = sin(modR) * logQuat(0) / modR;
            y = sin(modR) * logQuat(1) / modR;
            z = sin(modR) * logQuat(2) / modR;

        } else {

            w = 1.0;
            x = 0.0;
            y = 0.0;
            z = 0.0;

        }

        return tf::Quaternion(x, y, z, w);

    }


    double distQuat(const tf::Quaternion& q1, const tf::Quaternion& q2) {

        double d;
        const tf::Quaternion q = q1 * q2.inverse();
        vec logQuat = log(q);

        if ((q.x() == 0) && (q.y() == 0) && (q.z() == 0) && (q.w() == -1)) d = 2 * M_PI;

        else {

            d = 2 * sqrt(logQuat(0) * logQuat(0) + logQuat(1) * logQuat(1) + logQuat(2) * logQuat(2));

        }

        return d;
    }

    tf::Transform Matrix4f2Transform(Eigen::Matrix4f Tm) {

        tf::Vector3 origin;
        origin.setValue(static_cast<double>(Tm(0,3)),static_cast<double>(Tm(1,3)),static_cast<double>(Tm(2,3)));

        tf::Matrix3x3 tf3d;
        tf3d.setValue(static_cast<double>(Tm(0,0)), static_cast<double>(Tm(0,1)), static_cast<double>(Tm(0,2)),
                      static_cast<double>(Tm(1,0)), static_cast<double>(Tm(1,1)), static_cast<double>(Tm(1,2)),
                      static_cast<double>(Tm(2,0)), static_cast<double>(Tm(2,1)), static_cast<double>(Tm(2,2)));

        tf::Quaternion tfqt;
        tf3d.getRotation(tfqt);

        tf::Transform transform;
        transform.setOrigin(origin);
        transform.setRotation(tfqt);

        return transform;

    }

    /*
     * provides rotation quaterion with angle a around the vector that is defined by xx, yy, zz
     */
    tf::Quaternion axisAngle2Quat(const double &xx, const double &yy, const double &zz, const double &a) {

        double result = sin( a / 2.0 );

        double x = xx * result;
        double y = yy * result;
        double z = zz * result;

        double w = cos( a / 2.0 );

        return tf::Quaternion(x, y, z, w).normalize();

    }

    double distancePoint2Line(double xp,double yp,double x1,double y1,double x2,double y2) {
        return abs((y2 - y1) * xp - (x2 -x1) * yp + x2 * y1 - x1 * y2) / sqrt((y2-y1) * (y2-y1) + (x2 -x1) * (x2 -x1));
    }

    arma::vec pointOnLine2Point(double xp,double yp,double x1,double y1,double x2,double y2) {

        double a = x2 - x1;
        double b = y1 - y2;
        double c = -y1 * a - x1 * b;
        double x = (b * (b * xp - a * yp) - a * c) / sqrt(a * a + b * b);
        double y = (a * (a * yp - b * xp) - b * c) / sqrt(a * a + b * b);

        arma::vec p(2);
        p(0) = x * 2;
        p(1) = y * 2;

        return p;

    }

    bool fileExists(const std::string filePath) {
        struct stat info;
        if(stat(filePath.c_str(), &info) != 0 )
            return false;
        return true;
    }

    bool isDirectory(const std::string dirPath) {
        boost::filesystem::path p = dirPath.c_str();
        return boost::filesystem::is_directory(p);
    }

    void copyFile(const std::string source, const std::string destination) {

        fstream f(source.c_str(), fstream::in|fstream::binary);
        f << noskipws;
        istream_iterator<unsigned char> begin(f);
        istream_iterator<unsigned char> end;

        fstream f2(destination.c_str(), fstream::out | fstream::trunc | fstream::binary);
        ostream_iterator<char> begin2(f2);

        copy(begin, end, begin2);

        f.flush();
        f2.flush();

        f.close();
        f2.close();

    }

    void deleteDirectory(std::string path) {
        if(isDirectory(path)) {
            boost::filesystem::path p = path.c_str();
            boost::filesystem::remove_all(p);
        }
    }

    void deleteFile(std::string path) {
        if(!isDirectory(path)) {
            boost::filesystem::path p = path.c_str();
            boost::filesystem::remove_all(p);
        }
    }

    sensor_msgs::PointCloud2 pclPcToSensorMsgsPc(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc) {
        sensor_msgs::PointCloud2 output;
        pcl::PCLPointCloud2 intermediate;
        pcl::toPCLPointCloud2(*pc, intermediate);
        pcl_conversions::fromPCL(intermediate, output);
        return output;
    }

    sensor_msgs::PointCloud2 pclPcToSensorMsgsPc(pcl::PointCloud<pcl::PointXYZ> pc) {
        sensor_msgs::PointCloud2 output;
        pcl::PCLPointCloud2 intermediate;
        pcl::toPCLPointCloud2(pc, intermediate);
        pcl_conversions::fromPCL(intermediate, output);
        return output;
    }

    pcl::PCLPointCloud2 sensorMsgsPcToPclPc2(sensor_msgs::PointCloud2 pc) {
        pcl::PCLPointCloud2 tmp;
        pcl_conversions::toPCL(pc, tmp);
        return tmp;
    }

    std::vector<double> createJointsVector(int n_args, ...) {
        va_list ap;
        va_start(ap, n_args);
        vector<double> retVec;
        for(int i = 1; i < (n_args + 1); ++i) {
            double curVal = va_arg(ap, double);
            retVec.push_back(curVal);
        }
        return retVec;
    }

    arma::vec quatToRpy(tf::Quaternion quat) {
        tf::Matrix3x3 rot(quat);
        arma::vec retRpy(3);
        rot.getRPY(retRpy(0), retRpy(1), retRpy(2));
        return retRpy;
    }

    arma::vec quatToRpy(geometry_msgs::Quaternion quat) {
        tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
        return quatToRpy(q);
    }

    tf::Quaternion rpyToQuat(const double roll, const double pitch, const double yaw) {
        tf::Quaternion quat(yaw, pitch, roll);
        return quat;
    }

    long getFileSize(std::string filename) {
        struct stat stat_buf;
        int rc = stat(filename.c_str(), &stat_buf);
        return rc == 0 ? stat_buf.st_size : -1;
    }

    double roundByDigits(double number, int numDigitsBehindComma) {
        return ceilf(number * pow(10.0, (double) numDigitsBehindComma)) / 100;
    }
    std::wstring stringToWString(const std::string& s) {

        std::wstring wsTmp(s.begin(), s.end());
        return wsTmp;

    }

    arma::vec convertAndRemoveOffset(std::vector<long long int>& supervisedTs) {
        int sampleCount = supervisedTs.size();
        arma::vec convertedVec(sampleCount ? sampleCount : 1);
        if(sampleCount) {
            long long int tOff = supervisedTs.at(0);
            for(int i = 0; i < sampleCount; ++i)
                convertedVec(i) = (double) (supervisedTs.at(i) - tOff) / 1e3;
        }
        return convertedVec;
    }

    std::vector<long long int> convertTimesInSecondsToTimeInMilliseconds(arma::vec& timesInSeconds) {
        std::vector<long long int> retVec;
        for(int i = 0; i < timesInSeconds.n_elem; ++i) {
            long long int newTime = round((double) timesInSeconds(i) * 1000.0);
            retVec.push_back(newTime);
        }
        return retVec;
    }

    arma::vec convertTimesInMillisecondsToTimeInSeconds(std::vector<long long int>& timesInSeconds) {
        int sampleCount = timesInSeconds.size();
        arma::vec convertedVec(sampleCount ? sampleCount : 1);
        if(sampleCount) {
            for(int i = 0; i < sampleCount; ++i)
                convertedVec(i) = (double) timesInSeconds.at(i) / 1e3;
        }
        return convertedVec;
    }

    bool isRotationMatrix(arma::mat rotMatrix) {

        if(rotMatrix.n_rows != 3 || rotMatrix.n_cols != 3)
            return false;

        mat supposedToBeIdentity = rotMatrix.t() * rotMatrix;
        if(!isEqualThresh(supposedToBeIdentity, eye(3, 3), 1e-3))
            return false;

        if(!isEqualThresh(det(rotMatrix), 1.0, 1e-3))
            return false;

        return true;

    }

    bool isEqualThresh(double value1, double value2, double threshold) {
        if(abs(value1 - value2) < threshold)
            return true;
        return false;
    }

    bool isEqualThresh(arma::mat mat1, arma::mat mat2, double threshold) {
        if(mat1.n_rows != mat2.n_rows || mat1.n_cols != mat2.n_cols)
            throw KukaduException("(isEqualThres) matrix dimensions do not fit");
        for(int i = 0; i < mat1.n_rows; ++i)
            for(int j = 0; j < mat1.n_cols; ++j)
                if(!isEqualThresh(mat1(i, j), mat2(i, j), threshold))
                    return false;
        return true;
    }

    arma::vec rotationMatrixToRpy(arma::mat rotMatrix) {

        arma::vec rpy(3);

        double alpha = 0.0;
        double gamma = 0.0;

        if(!isRotationMatrix(rotMatrix))
            throw KukaduException("(rotationMatrixToRpy) passed argument is not a rotation matrix");

        double beta = atan2(-rotMatrix(2, 0), sqrt(pow(rotMatrix(0, 0), 2.0) + pow(rotMatrix(1, 0), 2.0)));
        if(isEqualThresh(beta, M_PI / 2.0, 1e-3)) {
            alpha = 0.0;
            gamma = atan2(rotMatrix(0, 1), rotMatrix(1, 1));
        } else if(isEqualThresh(beta, -M_PI / 2.0, 1e-3)) {
            alpha = 0.0;
            gamma = -atan2(rotMatrix(0, 1), rotMatrix(1, 1));
        } else {
            alpha = atan2(rotMatrix(1, 0) / cos(beta), rotMatrix(0, 0) / cos(beta));
            gamma = atan2(rotMatrix(2, 1) / cos(beta), rotMatrix(2, 2) / cos(beta));
        }

        rpy(0) = gamma; rpy(1) = beta; rpy(2) = alpha;
        return rpy;

    }

    tf::Transform affineTransMatrixToTf(arma::mat transMatrix) {

        if(transMatrix.n_cols != 4 || transMatrix.n_rows != 4)
            throw KukaduException("(affineTransMatrixToTf) transformation matrix has wrong dimensionality (not 4x4)");

          tf::Vector3 origin;
          origin.setValue(transMatrix(0, 3), transMatrix(1, 3), transMatrix(2, 3));

          tf::Matrix3x3 tf3d;
          tf3d.setValue(
                      transMatrix(0, 0), transMatrix(0, 1), transMatrix(0, 2),
                      transMatrix(1, 0), transMatrix(1, 1), transMatrix(1 ,2),
                      transMatrix(2, 0), transMatrix(2, 1), transMatrix(2, 2)
                );

          tf::Quaternion tfqt;
          tf3d.getRotation(tfqt);

          tf::Transform transform;
          transform.setOrigin(origin);
          transform.setRotation(tfqt);

          return transform;

    }

    std::vector<std::string> readFunctionSignature(std::string prettyFunc) {

        std::string funcNamespace;
        std::string funcClass;
        std::string funcName;

        // this function assumes a name according to the following conventions:
        // arbitrary number of namespaces with lower case names
        // followed by one class with upper case name (no nested classes)
        // followed by the function name
        string signat = prettyFunc;
        kukadu::KukaduTokenizer signTok(signat, "(");
        // extract the return type and  complete function name
        auto retAndName = signTok.next();

        // kick out the return type
        signTok = kukadu::KukaduTokenizer(retAndName, " ");
        auto signSplit = signTok.split();

        if(signSplit.size() > 0) {

            auto complFuncName = signSplit.back();
            signTok = kukadu::KukaduTokenizer(complFuncName, ":");
            auto signParts = signTok.split();
            if(signParts.size() >= 2) {

                int i = 0;
                for(; i < (signParts.size() - 1) && std::islower(signParts.at(i)[0]); ++i)
                    funcNamespace += signParts.at(i);

                // if after the namespace, there are 2 more tokens --> its a class member
                if(i == (signParts.size() - 2))
                    funcClass = signParts.at(i++);

                // the last token must be the function name
                funcName = signParts.at(i);

            }

        }

        return {funcNamespace, funcClass, funcName};

    }

    arma::vec loadJointPosByTimestamp(StorageSingleton& storage, int robotId, std::vector<int> jointIds, long long int timeStamp) {

        stringstream s;
        s << "select joint_id, position from joint_mes where robot_id = " << robotId << " and joint_id in (";

        bool firstJoint = true;
        for(auto& jointId : jointIds) {
            if(!firstJoint)
                s << ", ";
            else
                firstJoint = false;
            s << jointId;
        }
        s << ")";
        s << " and time_stamp = " << timeStamp;

        vector<bool> jointIdSet(jointIds.size());
        for(int i = 0; i < jointIdSet.size(); ++i)
            jointIdSet.at(i) = false;

        vec retVec(jointIds.size());

        auto jointPosRes = storage.executeQuery(s.str());
        while(jointPosRes->next()) {
            auto currentJointId = jointPosRes->getInt("joint_id");
            auto currentPos = jointPosRes->getDouble("position");
            auto currentJointIt = std::find(jointIds.begin(), jointIds.end(), currentJointId);
            if(currentJointIt != jointIds.end()) {
                int idIdx = currentJointIt - jointIds.begin();
                jointIdSet.at(idIdx) = true;
                retVec(idIdx) = currentPos;
            } else
                throw KukaduException("(loadJointPosByTimestamp) sql error");
        }

        for(int i = 0; i < jointIdSet.size(); ++i)
            if(!jointIdSet.at(i))
                throw KukaduException("(loadJointPosByTimestamp) not all requested joint positions are in the database for the requested timestamp");

        return retVec;

    }

    std::vector<int> loadJointIdsFromName(StorageSingleton& storage, int robotId, std::vector<std::string> jointNames) {

        vector<int> idVec(jointNames.size());

        stringstream s;
        s << "select joint_id, joint_name from hardware_joints where hardware_instance_id = " << robotId << " and joint_name in (";
        bool firstJoint = true;
        for(auto& jointName : jointNames) {
            if(!firstJoint)
                s << ", ";
            else
                firstJoint = false;
            s << "'" << jointName << "'";
        }
        s << ")";

        vector<bool> jointIdSet(jointNames.size());
        for(int i = 0; i < jointIdSet.size(); ++i)
            jointIdSet.at(i) = false;

        auto jointRes = storage.executeQuery(s.str());
        while(jointRes->next()) {
            auto currentJointName = jointRes->getString("joint_name");
            auto currentJointId = jointRes->getInt("joint_id");
            auto currentJointIt = std::find(jointNames.begin(), jointNames.end(), currentJointName);
            if(currentJointIt != jointNames.end()) {
                int idIdx = currentJointIt - jointNames.begin();
                jointIdSet.at(idIdx) = true;
                idVec.at(idIdx) = currentJointId;
            } else
                throw KukaduException("(loadJointIdsFromName) sql error");
        }

        for(int i = 0; i < jointIdSet.size(); ++i)
            if(!jointIdSet.at(i))
                throw KukaduException("(loadJointIdsFromName) robot does not contain all requested joint names");

        return idVec;


    }

    double armadilloMin(arma::vec& v) {

        double min = std::numeric_limits<double>::max();
        for(int i = 0; i < v.n_elem; ++i)
            if(min > v(i))
                min = v(i);
        return min;

    }

    double armadilloMax(arma::vec& v) {

        double max = std::numeric_limits<double>::min();
        for(int i = 0; i < v.n_elem; ++i)
            if(max < v(i))
                max = v(i);
        return max;

    }

    double armadilloMin(arma::mat& m) {

        double min = std::numeric_limits<double>::max();
        for(int i = 0; i < m.n_rows; ++i)
            for(int j = 0; j < m.n_cols; ++j)
                if(min > m(i, j))
                    min = m(i, j);
        return min;

    }

    double armadilloMax(arma::mat& m) {

        double max = std::numeric_limits<double>::min();
        for(int i = 0; i < m.n_rows; ++i)
            for(int j = 0; j < m.n_cols; ++j)
                if(max < m(i, j))
                    max = m(i, j);
        return max;

    }

    bool armadilloHasNan(arma::mat& m) {
        for(int i = 0; i < m.n_rows; ++i)
            for(int j = 0; j < m.n_cols; ++j)
                if(m(i, j) != m(i, j))
                    return true;
        return false;
    }

    bool armadilloHasNan(arma::vec& v) {

        for(int i = 0; i < v.n_elem; ++i)
            if(v(i) != v(i))
                return true;
        return false;

    }
    
    bool armadilloHasInf(arma::mat& m) {
        for(int i = 0; i < m.n_rows; ++i)
            for(int j = 0; j < m.n_cols; ++j)
                if(std::isinf(m(i, j)))
                    return true;
        return false;
    }

	bool armadilloHasInf(arma::vec& v) {
		for(int i = 0; i < v.n_elem; ++i)
            if(std::isinf(v(i)))
                return true;
        return false;
	}

    arma::mat computeRotFromCorrespondences(const std::vector<vec>& cs1, const std::vector<vec>& cs2) {

        auto normalizedCameraCentroid = zeros(3);
        auto normalizedRobotCentroid = zeros(3);

        mat h(3, 3); h.fill(0.0);
        for(int i = 0; i < cs1.size(); ++i)
            h += (cs1.at(i) - normalizedCameraCentroid) * (cs2.at(i) - normalizedRobotCentroid).t();

        mat u(3, 3); mat v(3, 3); vec s(3);
        svd(u, s, v, h);

        if(det(v) < 0.0) {

            // if reflection case --> mirror the 3rd column
            for(int i = 0; i < v.n_rows; ++i)
                v(2, i) = -v(2, i);

        }

        mat r = v * u.t();

        return r;

    }

}
