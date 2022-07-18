#include <iostream>
#include <algorithm>
#include <math.h>
#include <fstream>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <unsupported/Eigen/KroneckerProduct>

namespace Foap
{

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

                void initTBG(const int n_robots, const int dim, Eigen::VectorXd const params);
        };

        class FOAP
        {
        public:
                static Eigen::MatrixXd laplacian(Eigen::MatrixXd const A);
                static Eigen::MatrixXd jacobian(Eigen::VectorXd q_i, Eigen::VectorXd qo_i, int type);
                static Eigen::MatrixXd jacobian2(Eigen::VectorXd q_i, Eigen::VectorXd qo_i, int type);
                static Eigen::MatrixXd pseudoinverse(Eigen::MatrixXd J, double lambda = 0);
                static Eigen::MatrixXd nullSpace(Eigen::MatrixXd const J, double lambda = 0);
                static Eigen::MatrixXd displacementVector(double theta, double rz, int n, int dim = 3);
                static Eigen::MatrixXd displacementVector2(double theta, double rz, int n, int dim = 3);
                static Eigen::VectorXd formationCentroid(Eigen::MatrixXd const formation);
                static Eigen::VectorXd centroidError(Eigen::MatrixXd const qz, Eigen::VectorXd const qz_centroid);
                static Eigen::VectorXd consensusError(Eigen::MatrixXd const L, Eigen::MatrixXd const qz);
                static Eigen::VectorXd consensusError(Eigen::MatrixXd const L, Eigen::VectorXd const qz);
                static Eigen::VectorXd normconsensusError(Eigen::VectorXd const e, int n_agents);
                static Eigen::VectorXd conFixed(Eigen::VectorXd e_i, double k1, double p = 0.5);
                static Eigen::VectorXd conPredefined(double h, double hp, double x_b, double kf, Eigen::VectorXd e_0, Eigen::VectorXd e_i);
                static Eigen::VectorXd convexCombination(Eigen::MatrixXd const q, Eigen::MatrixXd const qobs, Eigen::VectorXd const ratios,
                                                         Eigen::VectorXd const p_ia, Eigen::MatrixXd const J_2, TBG &tbg,
                                                         double t, double d, double dt, int i_agent);
                static Eigen::VectorXd getSingleAgent(Eigen::VectorXd const e_q, int n_agents, int current);
                static Eigen::VectorXd TBGgain_consensus(double tf, double t, double kf);

                // static void initTBG(TBG &tbg, Eigen::VectorXd const params);
                static bool readAdjancencyMat(std::string filename, Eigen::MatrixXd &A);
                static Eigen::MatrixXd vector2matrix(Eigen::VectorXd v, int dim = 3);
                static Eigen::VectorXd matrix2vector(Eigen::MatrixXd const M);
                static void saveMatrixData(Eigen::MatrixXd M, std::string filename);

        private:
                static Eigen::MatrixXd transpose(Eigen::MatrixXd const M);
                static Eigen::VectorXd agentsDistances(Eigen::MatrixXd const q, int current);
                static Eigen::VectorXd agentsDistances2(Eigen::MatrixXd const q, int current);
                static Eigen::VectorXd obstaclesDistances(Eigen::VectorXd const q_i, Eigen::MatrixXd const qobs);
                static double nearestAgent(Eigen::MatrixXd const q, Eigen::VectorXd &nearest, int current);
                static double nearestObstacle(Eigen::VectorXd const q_i, Eigen::MatrixXd const qobs, Eigen::VectorXd &nearest);
                static double obstacleError(Eigen::MatrixXd const N, Eigen::VectorXd const distances, Eigen::VectorXd const ratios, Eigen::VectorXd &nearest);
                static int obstacleType(Eigen::MatrixXd const N, Eigen::VectorXd const distances, Eigen::VectorXd const ratios);
                static Eigen::VectorXd TBGgain(double d, double t0, double tf, double a0, double t);
                // static Eigen::VectorXd TBGgain_consensus(double tf,double t,double kf);
                static void xi(TBG &tbg, double eo, double t, double d, double dt, int i_agent);
        };
} // namespace Foap
