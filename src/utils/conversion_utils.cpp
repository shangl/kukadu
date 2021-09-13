#include <kukadu/utils/conversion_utils.hpp>

using namespace std;
using namespace arma;

namespace kukadu {

    int compareVectorOfArmadillos(std::vector<arma::vec> vec1, std::vector<arma::vec> vec2) {
        if(vec1.size() != vec2.size()) return 0;
        for(int i = 0; i < vec1.size(); ++i)
            if(!compareArmadilloVec(vec1.at(i), vec2.at(i))) return 0;
        return 1;
    }

    int compareArmadilloVec(arma::vec vec1, arma::vec vec2) {
        if(vec1.n_elem != vec2.n_elem) return 0;
        for(int i = 0; i < vec1.n_elem; ++i)
            if(vec1(i) != vec2(i)) return 0;
        return 1;
    }

    int compareVectorOfDoubles(std::vector<double> vec1, std::vector<double> vec2) {
        if(vec1.size() != vec2.size()) return 0;
        for(int i = 0; i < vec1.size(); ++i)
            if(vec1.at(i) != vec2.at(i)) return 0;
        return 1;
    }

    /*
     * parses a string and returns the floating point value of the represented number
     *
     * returns: number parsed from the input string s, or 0 if parsing was not successful
     * input:
     * 	const std::string& s:	string to parse
    */
    double string_to_double(const std::string& s) {
        std::istringstream i(s);
        double x;
        if (!(i >> x))
            return 0;
        return x;
    }

    mat readMat(std::ifstream& inStream) {

        mat retMatrix(1, 1);
        string line;
        double dn = 0.0;
        string token;
        bool firstTime;

        if(inStream.is_open()) {
            int j = 0;
            while(inStream.good()) {
                getline(inStream, line);
                if(line != "" && line != "=") {
                    KukaduTokenizer tok(line);
                    int i = 0;
                    for(i = 0; (token = tok.next()) != ""; ++i) {
                        dn = string_to_double(token);
                        if((i + 1) >= retMatrix.n_cols)
                            retMatrix.resize(j + 1, i + 1);
                        retMatrix(j, i) = dn;

                    }
                    j++;
                    if(firstTime) {
                        retMatrix.resize(j + 1, i);
                        firstTime = false;
                    } else
                        retMatrix.resize(j + 1, retMatrix.n_cols);
                } else if(line == "=") {
                    return retMatrix;
                } else if (line == "") {
                    retMatrix.resize(retMatrix.n_rows - 1, retMatrix.n_cols);
                //    cout << "found an empty line" << endl;
                }
            }
        }

        return retMatrix;

    }

    std::string double_to_string(const double d) {
        std::ostringstream strs;
        strs << d;
        std::string str = strs.str();
        return str;
    }

}
