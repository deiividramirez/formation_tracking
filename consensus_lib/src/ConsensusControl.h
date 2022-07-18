#include <iostream>
#include <algorithm>
#include <math.h>
#include <fstream>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <unsupported/Eigen/KroneckerProduct>

struct TBG
{
    Eigen::VectorXd a_gain;
    Eigen::VectorXd xi;
    Eigen::VectorXd xip;
    Eigen::RowVectorXd t0_o; // start time of Xi function
    Eigen::RowVectorXd tf_o; // final time of XI function
    Eigen::MatrixXd J2N1_p;
    Eigen::VectorXd q_12;
    double t_up;   // up time of Xi function (0,1)
    double t_down; // down time of Xi function (0,1)
    double t_a;    // task active time
    double a_0;
};

struct OBS_TASK
{
    Eigen::VectorXd center;
    Eigen::VectorXd e;
    Eigen::MatrixXd J;
    double rad;
};

/*
Control formation functions
*/
Eigen::MatrixXd laplacian(Eigen::MatrixXd A);
Eigen::VectorXd consensusError(Eigen::MatrixXd L, Eigen::MatrixXd qz);
Eigen::VectorXd consensusError(Eigen::MatrixXd L, Eigen::VectorXd qz);
Eigen::MatrixXd jacobian(Eigen::VectorXd q_i, Eigen::VectorXd qo_i);
Eigen::MatrixXd pseudoinverse(Eigen::MatrixXd J, double lambda = 0);
Eigen::MatrixXd nullSpace(Eigen::MatrixXd J, double lambda = 0);
Eigen::VectorXd agentsDistances(Eigen::MatrixXd q, int current);
Eigen::VectorXd obstaclesDistances(Eigen::VectorXd q_i, Eigen::MatrixXd qobs);
Eigen::VectorXd TBGgain_cos(double d, double t0, double tf, double a0, double t);
Eigen::VectorXd TBGgain(double d, double t0, double tb, int t, int opt);
double nearestAgent(Eigen::MatrixXd q, Eigen::VectorXd &nearest, int current);
double nearestObstacle(Eigen::VectorXd q_i, Eigen::MatrixXd qobs, Eigen::VectorXd &nearest);
double obstacleError(Eigen::MatrixXd N, Eigen::VectorXd distances, Eigen::VectorXd ratios, Eigen::VectorXd &nearest);
Eigen::VectorXd velDesDelSmooth(Eigen::MatrixXd q, Eigen::MatrixXd qobs, Eigen::VectorXd ratios,
                                Eigen::VectorXd p_ia, TBG &tbg, OBS_TASK p_c,
                                double t, double d, double dt, int i_agent);
void softTransition(TBG &tbg, double eo, double t, double d, double dt, int i_agent);
Eigen::MatrixXd displacementVector(double theta, double rz, int n, int dim = 3);
Eigen::VectorXd formationCentroid(Eigen::MatrixXd formation);
Eigen::VectorXd conFixed(Eigen::MatrixXd L, Eigen::VectorXd x, double k1, double k2, double p, double q);
Eigen::VectorXd conFixed(Eigen::VectorXd e_i, double k1, double k2, double p, double q);

/*
Auxiliar functions
*/
Eigen::VectorXd getSingleAgent(Eigen::VectorXd e_q, int n_agents, int current);
Eigen::VectorXd centroidError(Eigen::MatrixXd qz, Eigen::VectorXd qz_centroid);
Eigen::VectorXd consensusErrorL2(Eigen::VectorXd e, int n_agents, int dim = 3);
Eigen::MatrixXd vector2matrix(Eigen::VectorXd v, int dim = 3);
Eigen::VectorXd matrix2vector(Eigen::MatrixXd M);
Eigen::MatrixXd transposeMatrix(Eigen::MatrixXd M);
bool readInput(std::string filename, Eigen::MatrixXd &A, Eigen::MatrixXd &q, Eigen::MatrixXd &qobs);
bool readAdjancencyMat(std::string filename, Eigen::MatrixXd &A);
void printMatrixData(Eigen::MatrixXd M, int page_cols = 13);
void saveMatrixData(Eigen::MatrixXd M, std::string filename);
