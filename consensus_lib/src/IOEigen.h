#include <iostream>
#include <algorithm>
#include <math.h>
#include <fstream>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace IOeigen
{
    class IOEigen
    {
    public:
        static bool readMatrix(std::string filename, Eigen::MatrixXd &M);
        static void writeMatrix(std::string filename, Eigen::MatrixXd const M, bool header = false);
        static Eigen::MatrixXd vector2matrix(Eigen::VectorXd v, int dim = 3);
        static Eigen::VectorXd matrix2vector(Eigen::MatrixXd const M);
    };

} // namespace Ioeigen