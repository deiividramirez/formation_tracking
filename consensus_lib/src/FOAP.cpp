#include "FOAP.h"
using namespace Foap;
#include <Eigen/Dense>
double MAX_DIST = 1e10;
double a_axis = 0.5, b_axis = 0.5, c_axis = 1;
Eigen::MatrixXd elipse_axis = Eigen::MatrixXd::Identity(3, 3);
// elipse_axis (2,2)= double 3;
// Eigen::elipse_axis (1,1) = pow(1/a_axis,2);
// elipse_axis(0,0)=2;
// elipse_axis ={ 1, 0, 0, 0, 0, 0, 0, 0, 0};
// Eigen::MatrixXd elipse_axis(3,3)= MatrixXd::Identity(3,3);//=pow(1/a_axis,2),0,0,0,pow(1/b_axis,2),0,0,0,pow(1/c_axis,2);
// elipse_axis <<pow(1/a_axis,2),0,0,0,pow(1/b_axis,2),0,0,0,pow(1/c_axis,2);
// elipse_axis <<1,0,0,0,1,0,0,0,1;

/*
Laplacian matrix calculation from a given Adjancency matrix
L = D - A
*/
Eigen::MatrixXd FOAP::laplacian(Eigen::MatrixXd const A)
{
    int rows = A.rows();
    Eigen::VectorXd Ones = Eigen::VectorXd::Ones(rows);
    Eigen::MatrixXd D = (A * Ones).asDiagonal();
    Eigen::MatrixXd L = D - A;

    return L;
}

/*
Jacobian calculus
J = (q_i - qo_i) / ||q_i - qo_i||
where q_i is the i-agent and qo_i is the nearest obstacle/agent related to q_i
*/
Eigen::MatrixXd FOAP::jacobian(Eigen::VectorXd const q_i, Eigen::VectorXd const qo_i, int type)
{

    Eigen::VectorXd temp = qo_i;
    if (type == 1)
    {
        /* code */
        // std::cout << "obstacle detected" << std::endl;
        // std::cout << "elipse_axis" << elipse_axis << std::endl;
        temp[2] = q_i[2];
    }
    Eigen::VectorXd q_diff = q_i - temp;
    double q_diff_norm = q_diff.norm();
    Eigen::MatrixXd J = q_diff / q_diff_norm;

    return FOAP::transpose(J);
}
/*
Jacobian calculos for elipsoid
*/
Eigen::MatrixXd FOAP::jacobian2(Eigen::VectorXd const q_i, Eigen::VectorXd const qo_i, int type)
{

    /*Se asigna la misma altura de los agentes al obstaculo*/
    // Eigen::MatrixXd elipse_axis = Eigen::MatrixXd::Identity(3,3);
    elipse_axis(0, 0) = pow(1 / a_axis, 2);
    elipse_axis(1, 1) = pow(1 / b_axis, 2);
    elipse_axis(2, 2) = pow(1 / c_axis, 2);
    // elipse_axis(0,0)=1
    double arg = 0, r_k = 0;
    Eigen::MatrixXd J;

    Eigen::VectorXd temp = qo_i;
    // elipse_axis (1,1) =2;
    // Obstacle detected
    if (type == 1)
    {
        /* code */
        // std::cout << "elipse_axis" << a_axis << std::endl;
        temp[2] = q_i[2];
        Eigen::VectorXd q_diff = q_i - temp;
        double q_diff_norm = q_diff.norm();
        // Eigen::MatrixXd
        J = q_diff / q_diff_norm;
    }
    // Agent detected
    else
    {

        Eigen::VectorXd q_diff = q_i - temp;
        arg = pow(1 / a_axis, 2) * pow(q_diff[0], 2) + pow(1 / b_axis, 2) * pow(q_diff[1], 2) + pow(1 / c_axis, 2) * pow(q_diff[2], 2);
        r_k = sqrt(arg - 1);

        // double q_diff_norm = q_diff.norm();
        // Eigen::MatrixXd
        // std::cout <<elipse_axis*q_diff<<std::endl;
        J = elipse_axis * q_diff / r_k;
    }

    return FOAP::transpose(J);
}
/*
Robust Moore-Penrose pseudoinverse for J:
J^+ = J'(JJ')^(-1)
*/
Eigen::MatrixXd FOAP::pseudoinverse(Eigen::MatrixXd J, double lambda)
{
    int rows, cols;
    rows = J.rows();
    cols = J.cols();
    Eigen::MatrixXd J_inv;
    Eigen::MatrixXd J_T = FOAP::transpose(J);
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
Null space
N = I - Ji^+ (q)Ji(q)
*/
Eigen::MatrixXd FOAP::nullSpace(Eigen::MatrixXd const J, double lambda)
{
    int cols = J.cols();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(cols, cols);
    Eigen::MatrixXd J_inv = pseudoinverse(J, lambda);
    Eigen::MatrixXd N = I - J_inv * J;

    return N;
}

/*
Displacement vector function
dim = 3D dimension by default
z_i = [rz*cos(theta*i), rz*sin(theta*i)], i in 1, ..., n-agents
theta will be transform in radians. (theta*pi/180)
*/
Eigen::MatrixXd FOAP::displacementVector(double theta, double rz, int n, int dim)
{
    double theta_z = theta * M_PI / 180.0;
    Eigen::VectorXd zeros = Eigen::VectorXd::Zero(n);
    Eigen::MatrixXd z(dim, n);
    Eigen::RowVectorXd z_x = zeros;
    Eigen::RowVectorXd z_y = zeros;
    Eigen::RowVectorXd z_z = zeros;

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

Eigen::MatrixXd FOAP::displacementVector2(double theta, double rz, int n, int dim)
{
    double theta_z = theta * M_PI / 180.0;
    Eigen::VectorXd zeros = Eigen::VectorXd::Zero(n);
    Eigen::MatrixXd z(dim, n);
    Eigen::RowVectorXd z_x = zeros;
    Eigen::RowVectorXd z_y = zeros;
    Eigen::RowVectorXd z_z = zeros;

    for (int i = 0; i < n - 2; i++)
    {
        z_x(i) = rz * cos(theta_z * (i + 1));
        z_y(i) = rz * sin(theta_z * (i + 1));
        z_z(i) = 0; // rz*cos(theta_z*(i+1));
    }
    z_z(n - 2) = 1;
    z_z(n - 1) = -1;

    z.row(0) = z_x;
    z.row(1) = z_y;
    z.row(2) = z_z;

    return z;
}

/*
Formation centroid
Formation = [q1, ... , qn], q_i = [x,y,z], 3xn
return centroid
*/
Eigen::VectorXd FOAP::formationCentroid(Eigen::MatrixXd const formation)
{
    int dim, n;
    dim = formation.rows(); // get the dimension
    n = formation.cols();   // number of agents/

    Eigen::VectorXd centroid(dim);

    for (int i = 0; i < dim; i++)
    {
        centroid(i) = formation.row(i).mean();
    }

    return centroid;
}

/*
Consensus error between virtual agents and the centroid
*/
Eigen::VectorXd FOAP::centroidError(Eigen::MatrixXd const qz, Eigen::VectorXd const qz_centroid)
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
Consensus error calculation
ez = -Lqz(t)
*/
Eigen::VectorXd FOAP::consensusError(Eigen::MatrixXd const L, Eigen::MatrixXd const qz)
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
Eigen::VectorXd FOAP::consensusError(Eigen::MatrixXd const L, Eigen::VectorXd const qz)
{
    int rows, dim;
    rows = L.rows();        // number of agents
    dim = qz.size() / rows; // dimension of the system
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(dim, dim);
    Eigen::VectorXd ez = -Eigen::kroneckerProduct(I, L).eval() * qz;

    return ez;
}

/*
Consensus error norm
return e = [e_x, e_y, e_z]
*/
Eigen::VectorXd FOAP::normconsensusError(Eigen::VectorXd const e, int n_agents)
{
    int dim = e.size() / n_agents;
    Eigen::VectorXd e_L2(dim);
    for (int i = 0; i < dim; i++)
    {
        e_L2(i) = e.segment(i * n_agents, n_agents).norm();
    }

    return e_L2;
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
Eigen::VectorXd FOAP::conFixed(Eigen::VectorXd e_i, double k1, double p)
{
    Eigen::VectorXd u, a, b;
    a = e_i.cwiseSign();
    b = k1 * ((e_i.cwiseAbs()).array().pow(p));
    u = a.cwiseProduct(b);

    return u;
}
/*
Predefined time consensus
L=Laplacian matrix
x=system state
kf=gain of control
u=system input
x_b=average of initial conditions
x(0)
u=hp(e_i(0))+kf(e_i(t)-h*e_i(0)
*/
Eigen::VectorXd FOAP::conPredefined(double h, double hp, double x_b, double kf, Eigen::VectorXd e_0, Eigen::VectorXd e_i)
{
    Eigen::VectorXd u;
    u = -hp * e_0 + kf * (e_i - h * e_0);
    return u;
}
/*
Global and local task combination
Lee et al. 2012
q = q_1 + q_1|2
*/

Eigen::VectorXd FOAP::convexCombination(Eigen::MatrixXd const q, Eigen::MatrixXd const qobs, Eigen::VectorXd const ratios,
                                        Eigen::VectorXd const p_ia, Eigen::MatrixXd const J_2, TBG &tbg,
                                        double t, double d, double dt, int i_agent)
{

    int rows, cols, ratios_size;
    double lambda_error = 20;
    rows = q.rows();
    cols = q.cols();
    ratios_size = ratios.size(); // rows = pose 3D, cols = number of agents/robots. ratios_size = number of ratios

    Eigen::VectorXd q_n_i;    // nearest agent from q_i
    double distance_q_n_i;    // distance to the nearest agent form q_i
    Eigen::VectorXd qobs_n_i; // nearest obstacle from q_i
    double distance_qobs_n_i; // distance to the nearest obstacle from q_i
    // Eigen::VectorXd q_floor_i; // floor of q_i
    // double distance_q_f; // distance of agent i from the floor

    // find the nearest agent/obstacle from the current agent
    Eigen::VectorXd q_i = q.col(i_agent);
    distance_q_n_i = nearestAgent(q, q_n_i, i_agent);

    distance_qobs_n_i = nearestObstacle(q_i, qobs, qobs_n_i);

    // distance_q_f = q_i(rows-1);
    // q_floor_i = q_i;
    // q_floor_i(rows-1) = 0;

    // keep the nearest agent & obstacle
    Eigen::MatrixXd nearest(rows, ratios_size); // assume 3D pose and 2 distances + floor distance
    nearest << q_n_i, qobs_n_i;                 //, q_floor_i;

    Eigen::VectorXd distances(ratios_size);         // assumme 2 distances + floor distance
    distances << distance_q_n_i, distance_qobs_n_i; //, distance_q_f;

    // get the smallest error
    double eo;
    int type = 3;
    Eigen::VectorXd eo_nearest; // nearest obstacle or agent

    // int Idx // to distinguish between agent and obstacle

    eo = obstacleError(nearest, distances, ratios, eo_nearest);
    type = obstacleType(nearest, distances, ratios);
    /* if (type==1)
     {
      std::cout<<"Obstaculo fijo"<< std::endl;
     }*/
    Eigen::VectorXd q_c;

    // If no obstacle is detected, the speed is maintained
    if (eo > 0 && tbg.t0_o(i_agent) == 0 && tbg.xi(i_agent) == 0)
    {
        q_c = p_ia;
        return q_c;
    }
    else
    { // the soft transition are activated for the obstacle avoidance
        xi(tbg, eo, t, d, dt, i_agent);
    }

    Eigen::MatrixXd J_o;
    // Since only one obstacle is avoid at a time, the parameters of the nearest obstacle are used, priorities are assigned
    // Compute the Jacobian
    J_o = jacobian2(q_i, eo_nearest, type);

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
TBG gain function
*/
Eigen::VectorXd FOAP::TBGgain(double d, double t0, double tf, double a0, double t)
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
TBG gain prefedined-time consensus
*/

Eigen::VectorXd FOAP::TBGgain_consensus(double tf, double t, double kf)
{
    Eigen::VectorXd ans(2); //[h,hp]
    if (t <= tf)
    {
        // Function h(t)
        ans(0) = 2 * pow((t / tf), 3) - 3 * pow((t / tf), 2) + 1;
        // Function hp(t)
        ans(1) = 6 * pow(1 / tf, 3) * pow(t, 2) - 6 * pow(1 / tf, 2) * t;
    }
    else
    {
        ans(0) = 0;
        ans(1) = 0;
    }

    return ans;
}
/*
xi smoothing function
*/
void FOAP::xi(TBG &tbg, double eo, double t, double d, double dt, int i_agent)
{
    Eigen::VectorXd ansTBG; // TBGgain_cos return [a, Xi, Xip]
    // negative error and the times are equal to zero, this indicates that the obstacle was detected for the first time
    if (tbg.t0_o(i_agent) == 0 && tbg.tf_o(i_agent) == 0 && eo < 0)
    {
        // The start and end times of the function are assigned
        tbg.t0_o(i_agent) = t;
        tbg.tf_o(i_agent) = t + tbg.t_up;
        ansTBG = TBGgain(d, tbg.t0_o(i_agent), tbg.tf_o(i_agent), tbg.a_0, t);
        tbg.a_gain(i_agent) = ansTBG(0);
        tbg.xi(i_agent) = ansTBG(1);
        tbg.xip(i_agent) = ansTBG(2);
    } // if the function is active (t0_o!= 0 && tf_o != 0) the values ​​of the initial activation time and the final time (t0_o, tf_o) are preserved
    else if (tbg.t0_o(i_agent) != 0 && tbg.tf_o(i_agent) != 0)
    {
        // If the time is within the limit of the final time it continues to grow smoothly
        if (t < (tbg.tf_o(i_agent) + tbg.t_a))
        {
            ansTBG = TBGgain(d, tbg.t0_o(i_agent), tbg.tf_o(i_agent), tbg.a_0, t);
            tbg.a_gain(i_agent) = ansTBG(0);
            tbg.xi(i_agent) = ansTBG(1);
            tbg.xip(i_agent) = ansTBG(2);
        } // If the time is no longer within the final time and activation time, the function (xi1) will gradually decrease from 1 to 0
        else if (t < (tbg.tf_o(i_agent) + tbg.t_a + tbg.t_down + dt))
        {
            ansTBG = TBGgain(d, tbg.tf_o(i_agent) + tbg.t_a, tbg.tf_o(i_agent) + tbg.t_a + tbg.t_down + dt, tbg.a_0, t);
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

/*****************Auxiliar methods*****************/
/*
Get each agent error
*/
Eigen::VectorXd FOAP::getSingleAgent(Eigen::VectorXd const e_q, int n_agents, int current)
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
TBG Function inicialization
*/
void TBG::initTBG(const int n_robots, const int dim, Eigen::VectorXd const params)
{
    if (params.size() < 5)
    {
        std::cout << "Default TBG inicialization" << std::endl;
        a_gain = 3 * Eigen::VectorXd::Ones(n_robots);
        t_up = 0.2;
        t_down = 0.1;
        t_a = 0;
        a_0 = 0.01;
    }
    else
    {
        a_gain = params(0) * Eigen::VectorXd::Ones(n_robots);
        t_up = params(1);
        t_down = params(2);
        t_a = params(3);
        a_0 = params(4);
    }
    xi = Eigen::VectorXd::Zero(n_robots);
    xip = Eigen::VectorXd::Zero(n_robots);
    t0_o = Eigen::RowVectorXd::Zero(n_robots);
    tf_o = Eigen::RowVectorXd::Zero(n_robots);
    J2N1_p = Eigen::MatrixXd::Identity(dim, dim);
}

/*
Save data to file
*/
void FOAP::saveMatrixData(Eigen::MatrixXd M, std::string filename)
{
    std::ofstream myfile;
    myfile.open(filename.c_str());
    myfile << M;
    myfile.close();
}

/*
Read adjancency matrix
n -> n robots
M -> matrix nxn
*/
bool FOAP::readAdjancencyMat(std::string filename, Eigen::MatrixXd &A)
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
Distances between agents q_i and all the others agents
*/
Eigen::VectorXd FOAP::agentsDistances(Eigen::MatrixXd const q, int current)
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
            distances(i) = MAX_DIST;
        }
        else
        {
            distances(i) = (q_current - v).norm();
        }
    }

    return distances;
}

/*
Distances between agents q_i and all the others agents, assuming that the agents are allipsoids
*/
Eigen::VectorXd FOAP::agentsDistances2(Eigen::MatrixXd const q, int current)
{
    int rows, cols;
    double argumento;
    rows = q.rows();
    cols = q.cols();
    Eigen::VectorXd q_current = q.col(current); // posicion del agente q
    Eigen::VectorXd v;
    Eigen::VectorXd distances(cols);
    for (int i = 0; i < cols; i++)
    {
        v = q.col(i);
        if (i == current)
        {
            distances(i) = MAX_DIST;
        }
        else
        {
            // distances(i) = (q_current - v).norm();
            /*
            ditancia elipsoidal del agente hacia algún obstáculo
            rk=(A²(x-xc)²+B²(y-yc)²+C²(z-zc)²-1)^(1/2);
            */
            argumento = pow(1 / a_axis, 2) * pow(q_current[0] - v[0], 2) + pow(1 / b_axis, 2) * pow(q_current[1] - v[1], 2) + pow(1 / c_axis, 2) * pow(q_current[2] - v[2], 2);
            if (argumento < 1)
            {
                // std::cout<<distances(i)<<std::endl<<"error"<<std::endl;
                distances(i) = 0;
                break;
            }
            distances(i) = sqrt(argumento - 1);
        }
    }
    // if (current==0)
    //   std::cout<<distances<<std::endl<<"nueva"<<std::endl;

    return distances;
}
/*
Distances between agent q_i and all the obstacles
*/
Eigen::VectorXd FOAP::obstaclesDistances(Eigen::VectorXd const q_i, Eigen::MatrixXd const qobs)
{
    int cols = qobs.cols(); // number of obstacles
    Eigen::VectorXd v;
    Eigen::VectorXd distances(cols);
    for (int i = 0; i < cols; i++)
    {
        /*
        Modificar esta parte, toma la altura que se le da al obstaculo, no toma el cilindro como tal
        */
        v = qobs.col(i);
        // std::cout<<"error "<<v<<"\n";
        v[2] = q_i[2];
        /*v[0]=qobs[0][i];
        v[1]=qobs[1][i];
            v[2]=q_i[2];
    */
        distances(i) = (q_i - v).norm();
        // std::cout<<"error"<<(q_i - v)<<"\n";
    }

    return distances;
}

/*
Obstacle Error function: error between agent q_i and the nearest agent/obstacle
eo_i = || q_i - qo'_i || - R
The returned error is the min error between other agents and obstacles.
The agents and the obstacles have a secure radio: r_q and r_obs
*/

double FOAP::obstacleError(Eigen::MatrixXd const N, Eigen::VectorXd const distances,
                           Eigen::VectorXd const ratios, Eigen::VectorXd &nearest)
{
    Eigen::VectorXd eo = distances - ratios;
    int minIdx;
    double min;
    min = eo.minCoeff(&minIdx);
    nearest = N.col(minIdx);
    return min;
}

int FOAP::obstacleType(Eigen::MatrixXd const N, Eigen::VectorXd const distances,
                       Eigen::VectorXd const ratios)
{
    Eigen::VectorXd eo = distances - ratios;
    int minIdx;
    double min;
    min = eo.minCoeff(&minIdx);
    return minIdx;
}

/*double FOAP::obstacleError2(Eigen::MatrixXd const N, Eigen::VectorXd const distances,
                            Eigen::VectorXd const ratios, Eigen::VectorXd &nearest, int &Idx){
    Eigen::VectorXd eo = distances- ratios;
    //int minIdx;
    double min;
    min = eo.minCoeff(&Idx);
    nearest = N.col(Idx);

    return min;
}*/

/*
Nearest agent to q_i agent.
Return q_near = [x, y, z]
*/
double FOAP::nearestAgent(Eigen::MatrixXd const q, Eigen::VectorXd &nearest, int current)
{
    int cols = q.cols(); // number of agents
    // Eigen::VectorXd distances = agentsDistances(q, current);
    Eigen::VectorXd distances = agentsDistances2(q, current);
    int minIdx;
    // distances para esferas y distances2 para elipses
    double min = distances.minCoeff(&minIdx);
    nearest = q.col(minIdx);

    return min;
}

/*
Nearest obstacle to q_i agent
*/
double FOAP::nearestObstacle(Eigen::VectorXd const q_i, Eigen::MatrixXd const qobs, Eigen::VectorXd &nearest)
{
    int cols = qobs.cols(); // number of obstacles
    int dim = q_i.size();   // dimensionnearestObstacle
    if (cols == 0)
    { // no fixed obstacles, we crete an imaginary obstacle
        nearest = MAX_DIST * Eigen::VectorXd::Ones(dim);
        return MAX_DIST;
    }

    Eigen::VectorXd distances = obstaclesDistances(q_i, qobs);

    int minIdx;
    double min = distances.minCoeff(&minIdx);
    nearest = qobs.col(minIdx);

    return min;
}

Eigen::MatrixXd FOAP::transpose(Eigen::MatrixXd const M)
{
    // M is a scalar
    if (M.size() == 1)
        return M;

    return M.transpose().eval();
}

/*
Parser for matrix to vector
M =
1 2
3 4
v = [1, 2, 3, 4]
*/
Eigen::VectorXd FOAP::matrix2vector(Eigen::MatrixXd const M)
{
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> M2(M);
    Eigen::Map<Eigen::VectorXd> v(M2.data(), M2.size());

    return v;
}

/*
Parser for vector to matrix
v = [1, 2, 3, 4]
M =
1 2
3 4
*/
Eigen::MatrixXd FOAP::vector2matrix(Eigen::VectorXd v, int dim)
{
    int n_robots = v.size() / dim;
    Eigen::Map<Eigen::MatrixXd> MT(v.data(), n_robots, dim);
    Eigen::MatrixXd M = MT.transpose().eval();

    return M;
}
