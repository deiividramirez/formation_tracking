#include "IOEigen.h"
using namespace IOeigen;

bool IOEigen::readMatrix(std::string filename, Eigen::MatrixXd &M)
{
    std::ifstream fin(filename.c_str());
    if (!fin.good())
    {
        return false;
    }
    int rows, cols;
    // read header matrix Name rows cols
    fin >> rows;
    fin >> cols;
    M.resize(rows, cols);
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            fin >> M(i, j);
        }
    }

    return true;
}

void IOEigen::writeMatrix(std::string filename, Eigen::MatrixXd const M, bool header)
{
    std::string str_header = "";
    std::ofstream myfile;
    myfile.open(filename.c_str());
    if (header)
    {
        str_header + std::to_string(M.rows()) + " " + std::to_string(M.cols()) + "\n";
        myfile << header;
    }
    myfile << M;
    myfile.close();
}