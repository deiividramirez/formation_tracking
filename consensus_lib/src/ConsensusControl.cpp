#include "ConsensusControl.h"

Eigen::IOFormat OctaveFmtD(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
std::string sepD = "\n----------------------------------------\n";

double INF = 1e10;

/*
Consensus error calculation
ez = -Lqz(t)
*/
Eigen::VectorXd consensusError(Eigen::MatrixXd L, Eigen::MatrixXd qz)
{
    int rows, dim;
    rows = L.rows();        // number of agents
    dim = qz.size() / rows; // dimension of the system
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(dim, dim);
    Eigen::VectorXd qz_vector = matrix2vector(qz);
    Eigen::VectorXd ez = -Eigen::kroneckerProduct(I, L).eval() * qz_vector;

    return ez;
}

// @override method for Vector parameter
Eigen::VectorXd consensusError(Eigen::MatrixXd L, Eigen::VectorXd qz)
{
    int rows, dim;
    rows = L.rows();        // number of agents
    dim = qz.size() / rows; // dimension of the system
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(dim, dim);
    Eigen::VectorXd ez = -Eigen::kroneckerProduct(I, L).eval() * qz;

    return ez;
}

/*
Laplacian matrix calculation from a given Adjancency matrix
L = D - A
*/
Eigen::MatrixXd laplacian(Eigen::MatrixXd A)
{
    int rows = A.rows();
    Eigen::VectorXd Ones = Eigen::VectorXd::Ones(rows);
    Eigen::MatrixXd D = (A * Ones).asDiagonal();
    Eigen::MatrixXd L = D - A;

    return L;
}

/*
Displacement vector function
dim = 3D dimension by default
z_i = [rz*cos(theta*i), rz*sin(theta*i)], i in 1, ..., n-agents
theta will be transform in radians. (theta*pi/180)
*/
Eigen::MatrixXd displacementVector(double theta, double rz, int n, int dim)
{
    double theta_z = theta * M_PI / 180.0;
    Eigen::MatrixXd z(dim, n);
    Eigen::RowVectorXd z_x(n);
    Eigen::RowVectorXd z_y(n);
    Eigen::RowVectorXd z_z(n);

    for (int i = 0; i < n; i++)
    {
        z_x(i) = rz * cos(theta_z * (i + 1));
        z_y(i) = rz * sin(theta_z * (i + 1));
        z_z(i) = 0;
    }
    z.row(0) = z_x;
    z.row(1) = z_y;
    z.row(2) = z_z;

    return z;
}

/*
Jacobian calculus
J = (q_i - qo_i) / ||q_i - qo_i||
where q_i is the i-agent and qo_i is the nearest obstacle/agent related to q_i
*/
Eigen::MatrixXd jacobian(Eigen::VectorXd q_i, Eigen::VectorXd qo_i)
{
    Eigen::VectorXd q_diff = q_i - qo_i;
    double q_diff_norm = q_diff.norm();
    Eigen::MatrixXd J = q_diff / q_diff_norm;

    if (J.size() == 1)
        return J;
    return J.transpose().eval();
    // return transposeMatrix(J);
}

/*
Robust Moore-Penrose pseudoinverse for J:
J^+ = J'(JJ')^(-1)
*/
Eigen::MatrixXd pseudoinverse(Eigen::MatrixXd J, double lambda)
{
    int rows, cols;
    rows = J.rows();
    cols = J.cols();
    Eigen::MatrixXd J_inv;
    Eigen::MatrixXd J_T = transposeMatrix(J);
    Eigen::MatrixXd JJ_T = J * J_T;
    if (rows == 1)
    { // case when J = 1xn
        J_inv = J_T * pow(JJ_T(0, 0) - pow(lambda, 2), -1);
    }
    else
    { // case when J = mxn
        Eigen::MatrixXd I;
        I = Eigen::MatrixXd::Identity(rows, cols);
        J_inv = J_T * ((JJ_T - pow(lambda, 2) * I).inverse());
    }

    return J_inv;
}

/*
Null space of principal task
N = I - Ji^+ (q)Ji(q)
*/
Eigen::MatrixXd nullSpace(Eigen::MatrixXd J, double lambda)
{
    int cols = J.cols();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(cols, cols);
    Eigen::MatrixXd J_inv = pseudoinverse(J, lambda);
    Eigen::MatrixXd N = I - J_inv * J;

    return N;
}

/*
Distances between agents q_i and all the others agents
*/
Eigen::VectorXd agentsDistances(Eigen::MatrixXd q, int current)
{
    int rows, cols;
    rows = q.rows();
    cols = q.cols();
    Eigen::VectorXd q_current = q.col(current);
    Eigen::VectorXd v;
    Eigen::VectorXd distances(cols);
    for (int i = 0; i < cols; i++)
    {
        v = q.col(i);
        if (i == current)
        {
            distances(i) = INF;
        }
        else
        {
            distances(i) = (q_current - v).norm();
        }
    }

    return distances;
}

/*
Distances between agent q_i and all the obstacles
*/
Eigen::VectorXd obstaclesDistances(Eigen::VectorXd q_i, Eigen::MatrixXd qobs)
{
    int cols = qobs.cols(); // number of obstacles
    Eigen::VectorXd v;
    Eigen::VectorXd distances(cols);
    for (int i = 0; i < cols; i++)
    {
        v = qobs.col(i);
        distances(i) = (q_i - v).norm();
    }

    return distances;
}

/*
Obstacle Error function: error between agent q_i and the nearest agent/obstacle
eo_i = || q_i - qo'_i || - R
The returned error is the min error between other agents and obstacles.
The agents and the obstacles have a secure radio: r_q and r_obs
*/

double obstacleError(Eigen::MatrixXd N, Eigen::VectorXd distances, Eigen::VectorXd ratios, Eigen::VectorXd &nearest)
{
    Eigen::VectorXd eo = distances - ratios;
    int minIdx;
    double min;
    min = eo.minCoeff(&minIdx);
    nearest = N.col(minIdx);

    return min;
}

/*
Nearest agent to q_i agent.
Return q_near = [x, y, z]
*/
double nearestAgent(Eigen::MatrixXd q, Eigen::VectorXd &nearest, int current)
{
    int cols = q.cols(); // number of agents
    Eigen::VectorXd distances = agentsDistances(q, current);
    int minIdx;
    double min = distances.minCoeff(&minIdx);
    nearest = q.col(minIdx);

    return min;
}

/*
Nearest obstacle to q_i agent
*/
double nearestObstacle(Eigen::VectorXd q_i, Eigen::MatrixXd qobs, Eigen::VectorXd &nearest)
{
    int cols = qobs.cols(); // number of obstacles
    Eigen::VectorXd distances = obstaclesDistances(q_i, qobs);
    int minIdx;
    double min = distances.minCoeff(&minIdx);
    nearest = qobs.col(minIdx);

    return min;
}

/*
Formation centroid
Formation = [q1, ... , qn], q_i = [x,y,z], 3xn
return centroid
*/
Eigen::VectorXd formationCentroid(Eigen::MatrixXd formation)
{
    int dim, n;
    dim = formation.rows(); // get the dimension
    n = formation.cols();   // number of agents/obstacles
    Eigen::VectorXd centroid(dim);

    for (int i = 0; i < dim; i++)
    {
        centroid(i) = formation.row(i).mean();
    }

    return centroid;
}

/*
TBG gain cos:
return [a, Xi, Xip]
*/
Eigen::VectorXd TBGgain_cos(double d, double t0, double tf, double a0, double t)
{
    Eigen::VectorXd ans(3); // [a, Xi, Xip]
    if (t < t0)
    {
        ans(1) = 0;
        ans(2) = 0;
    }
    else if ((t <= tf) && (t >= t0))
    {
        ans(1) = 0.5 * (1 - cos((M_PI * (t - t0)) / (tf - t0)));
        ans(2) = (M_PI / (2.0 * (tf - t0))) * sin((M_PI * (t - t0)) / (tf - t0));
    }
    else if (t > tf)
    {
        ans(1) = 1;
        ans(2) = 0;
    }
    ans(0) = a0 + (ans(2) / (1 - ans(1) + d));

    return ans;
}

/*
TBG gain:
return [a,b,Xi,dXi,ddXi]
*/
Eigen::VectorXd TBGgain(double d, double t0, double tb, int t, int opt)
{
    Eigen::VectorXd ans(5); // [a,b,Xi,dXi,ddXi]
    if (opt)
    {
        if (t < t0)
        {
            ans(2) = 0;
            ans(3) = 0;
            ans(4) = 0;
        }
        else if ((t <= tb) && (t >= t0))
        {
            ans(2) = 10 * ((pow((t - t0), 3)) / (pow((tb - t0), 3))) - 15 * ((pow((t - t0), 4)) / (pow((tb - t0), 4))) + 6 * ((pow((t - t0), 5)) / (pow((tb - t0), 5)));
            ans(3) = 1.0 * (30 * ((pow((t - t0), 2)) / (pow((tb - t0), 3))) - 60 * ((pow((t - t0), 3)) / (pow((tb - t0), 4))) + 30 * ((pow((t - t0), 4)) / (pow((tb - t0), 5))));
            ans(4) = 60 * ((pow((t - t0), 1)) / (pow((tb - t0), 3))) - 180 * ((pow((t - t0), 2)) / (pow((tb - t0), 4))) + 120 * ((pow((t - t0), 3)) / (pow((tb - t0), 5)));
        }
        else if (t > tb)
        {
            ans(2) = 1;
            ans(3) = 0;
            ans(4) = 0;
        }
    }
    else
    {
        if (t < t0)
        {
            ans(2) = 0;
            ans(3) = 0;
            ans(4) = 0;
        }
        else if ((t <= tb) && (t >= t0))
        {
            ans(2) = 0.5 * (1 - cos(M_PI * (t - t0) / (tb - t0)));
            ans(3) = 0.5 * (M_PI / (tb - t0)) * sin(M_PI * (t - t0) / (tb - t0));
            ans(4) = 0.5 * pow((M_PI / (tb - t0)), 2) * cos(M_PI * (t - t0) / (tb - t0));
        }
        else if (t > tb)
        {
            ans(2) = 1;
            ans(3) = 0;
            ans(4) = 0;
        }
    }

    double f = 3;
    double tmp = 1 - ans(2) + d;
    ans(0) = f * ans(3) / tmp;
    ans(1) = (ans(4) * tmp + f * pow(ans(3), 2)) / pow(tmp, 2);

    return ans;
}

/*
Compute the desired velocity with obstacle avoidance and consensus
Agents: q = [q_1, ... , q_n], q_i = [x, y, z]
Obstacles: qobs = [qobs_1, ..., qobs_o], qobs_i = [x, y, z]
Secure Ratios: ratios = [r1, r2]
consensus speed = p_ia = [vx, vy, vz]

return q_c = consensus speed, tbg_n
*/
Eigen::VectorXd velDesDelSmooth(Eigen::MatrixXd q, Eigen::MatrixXd qobs, Eigen::VectorXd ratios,
                                Eigen::VectorXd p_ia, TBG &tbg, OBS_TASK p_c,
                                double t, double d, double dt, int i_agent)
{

    int rows, cols, ratios_size;
    double lambda_error = 15;
    rows = q.rows();
    cols = q.cols();
    ratios_size = ratios.size(); // rows = pose 3D, cols = number of agents/robots. ratios_size = number of ratios

    Eigen::MatrixXd J_2 = p_c.J;
    Eigen::VectorXd q_n_i;    // nearest agent from q_i
    double distance_q_n_i;    // distance to the nearest agent form q_i
    Eigen::VectorXd qobs_n_i; // nearest obstacle from q_i
    double distance_qobs_n_i; // distance to the nearest obstacle from q_i

    // find the nearest agent/obstacle from the current agent
    Eigen::VectorXd q_i = q.col(i_agent);
    distance_q_n_i = nearestAgent(q, q_n_i, i_agent);
    distance_qobs_n_i = nearestObstacle(q_i, qobs, qobs_n_i);

    // keep the nearest agent & obstacle
    Eigen::MatrixXd nearest(rows, ratios_size); // assume 3D pose and 2 distances
    nearest << q_n_i, qobs_n_i;
    Eigen::VectorXd distances(ratios_size); // assumme 2 distances
    distances << distance_q_n_i, distance_qobs_n_i;

    // get the smallest error
    double eo;
    Eigen::VectorXd eo_nearest; // nearest obstacle or agent
    eo = obstacleError(nearest, distances, ratios, eo_nearest);
    Eigen::VectorXd q_c;
    // If no obstacle is detected, the speed is maintained
    if (eo > 0 && tbg.t0_o(i_agent) == 0 && tbg.xi(i_agent) == 0)
    {
        q_c = p_ia;
        return q_c;
    }
    else
    { // the soft transition are activated for the obstacle avoidance
        softTransition(tbg, eo, t, d, dt, i_agent);
    }

    Eigen::MatrixXd J_o;
    // Since only one obstacle is avoid at a time, the parameters of the nearest obstacle are used, priorities are assigned
    // Compute the Jacobian
    J_o = jacobian(q_i, eo_nearest);

    // Compute the null space & get the control law
    double lambda = 0.01;
    Eigen::MatrixXd N_1 = nullSpace(J_o);
    Eigen::MatrixXd J2N1 = J_2 * N_1;
    Eigen::MatrixXd J2N1_p = pseudoinverse(J2N1, lambda);
    tbg.J2N1_p = J2N1_p;

    // ê1_t = h(t)ê1 + (1- h(t))J2J1⁺ê2
    Eigen::MatrixXd Jp_2 = pseudoinverse(J_2);
    Eigen::MatrixXd Jp_o = pseudoinverse(J_o);
    double e1_t = -lambda_error * tbg.xi(i_agent) * eo + ((1.0 - tbg.xi(i_agent)) * (J_o * Jp_2 * p_ia)(0, 0));

    // q_12 = (J2N1)⁺(ê2 - J2J1⁺ ê1_t)
    Eigen::VectorXd q_12 = J2N1_p * (p_ia - J_2 * Jp_o * e1_t);
    tbg.q_12 = q_12;

    // Control law: q = J1⁺ê1_t + q_12
    q_c = Jp_o * e1_t + q_12;

    return q_c;
}

/*
Soft transition for obstacle avoidance function
return tbg updated
*/
void softTransition(TBG &tbg, double eo, double t, double d, double dt, int i_agent)
{
    Eigen::VectorXd ansTBG; // TBGgain_cos return [a, Xi, Xip]
    // negative error and the times are equal to zero, this indicates that the obstacle was detected for the first time
    if (tbg.t0_o(i_agent) == 0 && tbg.tf_o(i_agent) == 0 && eo < 0)
    {
        // The start and end times of the function are assigned
        tbg.t0_o(i_agent) = t;
        tbg.tf_o(i_agent) = t + tbg.t_up;
        ansTBG = TBGgain_cos(d, tbg.t0_o(i_agent), tbg.tf_o(i_agent), tbg.a_0, t);
        tbg.a_gain(i_agent) = ansTBG(0);
        tbg.xi(i_agent) = ansTBG(1);
        tbg.xip(i_agent) = ansTBG(2);
    } // if the function is active (t0_o!= 0 && tf_o != 0) the values ​​of the initial activation time and the final time (t0_o, tf_o) are preserved
    else if (tbg.t0_o(i_agent) != 0 && tbg.tf_o(i_agent) != 0)
    {
        // If the time is within the limit of the final time it continues to grow smoothly
        if (t < (tbg.tf_o(i_agent) + tbg.t_a))
        {
            ansTBG = TBGgain_cos(d, tbg.t0_o(i_agent), tbg.tf_o(i_agent), tbg.a_0, t);
            tbg.a_gain(i_agent) = ansTBG(0);
            tbg.xi(i_agent) = ansTBG(1);
            tbg.xip(i_agent) = ansTBG(2);
        } // If the time is no longer within the final time and activation time, the function (xi1) will gradually decrease from 1 to 0
        else if (t < (tbg.tf_o(i_agent) + tbg.t_a + tbg.t_down + dt))
        {
            ansTBG = TBGgain_cos(d, tbg.tf_o(i_agent) + tbg.t_a, tbg.tf_o(i_agent) + tbg.t_a + tbg.t_down + dt, tbg.a_0, t);
            tbg.xi(i_agent) = 1.0 - ansTBG(1);
            // If during the transition from 1 to 0 the obstacle is invaded (e_o <0),
            // the time of t0_o and tf_o is reassigned so that the function xi
            // takes the value where it left off and begins its transition from the current value to 1 smoothly.
            if (eo < 0)
            {
                tbg.tf_o(i_agent) = 2 * t - tbg.t_a - tbg.t_up - tbg.t0_o(i_agent);
                tbg.t0_o(i_agent) = tbg.tf_o(i_agent) - tbg.t_up;
            }
        }
        else
        { // If it does not enter the conditions, the initial values ​​are assigned until another obstacle is found
            tbg.xi(i_agent) = 0;
            tbg.tf_o(i_agent) = 0;
            tbg.t0_o(i_agent) = 0;
        }
    }
}

/*
Finite time consensus
L = Laplacian matrix
x = system state
k1, k2 = gains
p, q = consensus pows
u = system output
u_i=k1*[e_i]^p+k2*[e_i]^q
*/
Eigen::VectorXd conFixed(Eigen::MatrixXd L, Eigen::VectorXd x, double k1, double k2, double p, double q)
{
    Eigen::VectorXd u, e_i, a, b;
    // use consensusError = -Lx
    e_i = consensusError(L, x);
    a = e_i.cwiseSign();
    b = k1 * ((e_i.cwiseAbs()).array().pow(p)); // + k2*((e_1.cwiseAbs()).array().pow(q));
    u = a.cwiseProduct(b);

    return u;
}

Eigen::VectorXd conFixed(Eigen::VectorXd e_i, double k1, double k2, double p, double q)
{
    Eigen::VectorXd u, a, b;
    // use consensusError = -Lx
    a = e_i.cwiseSign();
    b = k1 * ((e_i.cwiseAbs()).array().pow(p)); // + k2*((e_1.cwiseAbs()).array().pow(q));
    u = a.cwiseProduct(b);

    return u;
}

/*
Helper functions
*/

/*
Get each agent error
*/
Eigen::VectorXd getSingleAgent(Eigen::VectorXd e_q, int n_agents, int current)
{
    int dim = e_q.size() / n_agents;
    Eigen::VectorXd e(dim);
    for (int i = 0; i < dim; i++)
    {
        e(i) = e_q(i * n_agents + current);
    }

    return e;
}

/*
Consensus error norm
return e = [e_x, e_y, e_z]
*/
Eigen::VectorXd consensusErrorL2(Eigen::VectorXd e, int n_agents, int dim)
{
    Eigen::VectorXd e_L2(dim);
    for (int i = 0; i < dim; i++)
    {
        e_L2(i) = e.segment(i * n_agents, n_agents).norm();
    }

    return e_L2;
}

/*
Consensus error between virtual agents and the centroid
*/
Eigen::VectorXd centroidError(Eigen::MatrixXd qz, Eigen::VectorXd qz_centroid)
{
    int rows, cols;
    rows = qz.rows();
    cols = qz.cols(); // dim, n_agents
    Eigen::VectorXd e;
    Eigen::MatrixXd diff(rows, cols);
    for (int i = 0; i < cols; i++)
    {
        diff.col(i) = qz.col(i) - qz_centroid;
    }
    e = matrix2vector(diff);

    return e;
}

/*
Parser for matrix to vector
M =
1 2
3 4
v = [1, 2, 3, 4]
*/
Eigen::VectorXd matrix2vector(Eigen::MatrixXd M)
{
    int rows, cols;
    rows = M.rows();
    cols = M.cols();
    Eigen::VectorXd v(rows * cols);
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
            v(i * cols + j) = M(i, j);

    return v;
}

/*
Parser for vector to matrix
v = [1, 2, 3, 4]
M =
1 2
3 4
*/
Eigen::MatrixXd vector2matrix(Eigen::VectorXd v, int dim)
{
    int cols = v.size() / dim;
    Eigen::MatrixXd M(dim, cols);
    for (int i = 0; i < dim; i++)
    {
        M.row(i) = v.segment(i * cols, cols);
    }

    return M;
}

/*
Auxiliar function to transpose a MatrixXd with dynamic size
*/
Eigen::MatrixXd transposeMatrix(Eigen::MatrixXd M)
{
    int rows, cols;
    rows = M.rows();
    cols = M.cols();
    Eigen::MatrixXd M_inv(rows, cols);
    M_inv << M;
    M_inv = M_inv.transpose().eval();

    return M_inv;
}

/*
Read from file adjancency matrix, agents and obstacles positions.
*/

bool readInput(std::string filename, Eigen::MatrixXd &A, Eigen::MatrixXd &q, Eigen::MatrixXd &qobs)
{
    std::ifstream fin(filename.c_str());
    if (!fin.good())
    {
        return false;
    }
    int n;
    // read Adjancency matrix
    fin >> n;
    A.resize(n, n);
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            fin >> A(i, j);
        }
    }
    // read robots positions
    int n_agents, dim;
    fin >> n_agents >> dim;
    q.resize(dim, n_agents);
    for (int i = 0; i < n_agents; i++)
    {
        for (int j = 0; j < dim; j++)
        {
            fin >> q(j, i);
        }
    }
    // read obstacles
    int n_obs, dim_obs;
    fin >> n_obs >> dim_obs;
    qobs.resize(dim_obs, n_obs);
    for (int i = 0; i < n_obs; i++)
    {
        for (int j = 0; j < dim_obs; j++)
        {
            fin >> qobs(j, i);
        }
    }

    return true;
}

/*
Read adjancency matrix
n -> n robots
M -> matrix nxn
*/
bool readAdjancencyMat(std::string filename, Eigen::MatrixXd &A)
{
    std::ifstream fin(filename.c_str());
    if (!fin.good())
    {
        return false;
    }
    int n;
    // read Adjancency matrix
    fin >> n;
    A.resize(n, n);
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            fin >> A(i, j);
        }
    }

    return true;
}

/*
Print data to std out in octave format
*/
void printMatrixData(Eigen::MatrixXd M, int page_cols)
{
    int rows, cols;
    rows = M.rows();
    cols = M.cols();
    Eigen::MatrixXd tmp;
    int max_cols;
    for (int i = 0; i < cols; i = i + page_cols)
    {
        if ((i + page_cols) < cols)
        {
            max_cols = page_cols;
            tmp.resize(rows, max_cols);
            std::cout << "\nColumns " << i + 1 << " through " << i + page_cols << ":\n\n";
        }
        else
        {
            max_cols = cols - i;
            tmp.resize(rows, max_cols);
            std::cout << "\nColumns " << i + 1 << " through " << cols << ":\n\n";
        }
        for (int j = 0; j < max_cols; j++)
        {
            tmp.col(j) = M.col(i + j);
        }
        std::cout << tmp << "\n";
    }
}

/*
Save data to file
*/
void saveMatrixData(Eigen::MatrixXd M, std::string filename)
{
    std::ofstream myfile;
    myfile.open(filename.c_str());
    myfile << M;
    myfile.close();
}
