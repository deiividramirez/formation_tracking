/**************************** IOEigen.h ****************************/
#include "../../consensus_lib/src/IOEigen.h"

/**************************** ROS ****************************/
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <mav_msgs/conversions.h>
#include <geometry_msgs/PointStamped.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/GetWorldProperties.h>

/************************** C++ libraries *****************************/
#include <iostream>
#include <algorithm>
#include <vector>
#include <string>

using namespace IOeigen;

/************************** Declaración de funciones *********************/
void positionCallback(const geometry_msgs::PointStamped::ConstPtr &msg, int index);
void getCurrentPose(Eigen::MatrixXd &q);
Eigen::MatrixXd ortProj(Eigen::MatrixXd &x);

/*************************** Variables globales *************************/
#define DEBUG 0

Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
std::string sep = "\n----------------------------------------\n";
bool status_file;

/****************************************************************************/
/*
    Si se ejecutará en un nuevo computador, se debe cambiar
    el valor de la variable "path" a la ruta del archivo de
    configuración segun el archivo README.md

    path = "${HOME}/catkin_ws/src/formation_tracking/src/"
*/

std::string path = "/home/leonardo/catkin_ws/src/formation_tracking/src/";

/*
    Se deben tener en cuenta 5 archivos necesarios para la ejecución
    de la simulación:

        1. basic_wall.world en ${HOME}/catkin_ws/src/rotors_simulator/rotors_gazebo/worlds
        2. multiple_hummingbirds_example.launch en ${HOME}/catkin_ws/src/rotors_simulator/rotors_gazebo/launch/
        3. A.dat en ${HOME}/catkin_ws/src/formation_tracking/src/
        4. bearing.dat en ${HOME}/catkin_ws/src/formation_tracking/src/
        5. conf.dat en ${HOME}/catkin_ws/src/formation_tracking/src/
        6. lider1_ace.dat en ${HOME}/catkin_ws/src/formation_tracking/src/
        7. lider2_ace.dat en ${HOME}/catkin_ws/src/formation_tracking/src/
*/

std::string A_filename = path + "A.dat";
std::string bearing_filename = path + "bearing.dat";
std::string conf_filename = path + "conf.dat";
std::string lider1_filename = path + "lider1_ace.dat";
std::string lider2_filename = path + "lider2_ace.dat";

std::string dron_nombre = "hummingbird";
std::string relativeEntityName = "world";

// pub & suv variables name
std::string slash("/");
std::string publisher_name = "/command/trajectory";
std::string subscriber_name = "/ground_truth/position/";

// ROS msg's
std::vector<geometry_msgs::PointStamped> pos_msg;

/****************************************************************************/

// VARIABLES PARA LAS DIMENSIONES DEL SISTEMA
/*
    En este ejemplo habrán 8 dromes, en 3 dimensiones,
    10 modelos (contando ground y la pared)
*/
int n_drones = 0, dim = 3, n_modelos = 0;

/********************************* MAIN ************************************/

int main(int argc, char **argv)
{
    /************* LECTURA DE MATRICES ADYACENCIA Y BEARINGS *************/
    std::map<std::string, Eigen::MatrixXd> params;
    std::string A_name;
    Eigen::MatrixXd A, bearing, configuraciones, lider1, lider2;

    // Lectura de conf.dat
    status_file = IOEigen::readMatrix(conf_filename, configuraciones);
    if (!status_file)
    {
        std::cout << "Ruta seleccionada: " << path << std::endl;
        std::cout << "No se encuentra la matriz de adyacencia \"conf.dat\"" << std::endl;
        std::cout << "De ser necesario cambie la ruta en ~/catkin_ws/src/formation_tracking/src/formation_tracking.cpp" << std::endl;
        return 1;
    }
    else
    {
        std::cout << "Configuraciones conf leida correctamente" << std::endl
                  << std::endl;
    }

    // Lectura de A.dat
    status_file = IOEigen::readMatrix(A_filename, A);
    if (!status_file)
    {
        std::cout << "Ruta seleccionada: " << path << std::endl;
        std::cout << "No se encuentra la matriz de adyacencia \"A.dat\"" << std::endl;
        std::cout << "De ser necesario cambie la ruta en ~/catkin_ws/src/formation_tracking/src/formation_tracking.cpp" << std::endl;
        return 1;
    }
    else
    {
        std::cout << "Matriz de adyacencia A leida correctamente:" << std::endl;
        std::cout << A << std::endl
                  << std::endl;
    }

    // Lectura de bearing.dat
    status_file = IOEigen::readMatrix(bearing_filename, bearing);
    if (!status_file)
    {
        std::cout << "Ruta seleccionada: " << path << std::endl;
        std::cout << "No se encuentra la matriz de adyacencia \"bearing.dat\"" << std::endl;
        std::cout << "De ser necesario cambie la ruta en ~/catkin_ws/src/formation_tracking/src/formation_tracking.cpp" << std::endl;
        return 1;
    }
    else
    {
        std::cout << "\nMatriz de bearings leida correctamente:" << std::endl;
        std::cout << bearing << std::endl
                  << std::endl;
    }

    // Lectura de lider1_ace.dat
    status_file = IOEigen::readMatrix(lider1_filename, lider1);
    if (!status_file)
    {
        std::cout << "Ruta seleccionada: " << path << std::endl;
        std::cout << "No se encuentra la matriz de adyacencia \"lider1_ace.dat\"" << std::endl;
        std::cout << "De ser necesario cambie la ruta en ~/catkin_ws/src/formation_tracking/src/formation_tracking.cpp" << std::endl;
        return 1;
    }
    else
    {
        std::cout << "Matriz de aceleraciones del lider1 leida correctamente" << std::endl;
    }

    // Lectura de lider2_ace.dat
    status_file = IOEigen::readMatrix(lider2_filename, lider2);
    if (!status_file)
    {
        std::cout << "Ruta seleccionada: " << path << std::endl;
        std::cout << "No se encuentra la matriz de adyacencia \"lider2_ace.dat\"" << std::endl;
        std::cout << "De ser necesario cambie la ruta en ~/catkin_ws/src/formation_tracking/src/formation_tracking.cpp" << std::endl;
        return 1;
    }
    else
    {
        std::cout << "Matriz de aceleraciones del lider2 leida correctamente\n"
                  << std::endl;
    }

    /************************* COMPROBACIÓN **************************/

    // Comprobación de las dimensiones de A y bearing
    if (A.cols() != (int)sqrt(bearing.rows()))
    {
        std::cout << "Dimensiones de A.dat y de bearing.dat no concuerdan :C" << std::endl;
        return 1;
    }
    else
    {
        std::cout << "Dimensiones de A.dat y de bearing.dat concuerdan :D" << std::endl
                  << std::endl;
    }

    // CONFIGURACIONES DEFINIDAS PARA EL SISTEMA
    double tmax = configuraciones(0, 0);
    double dt = configuraciones(1, 0);
    double Kp = configuraciones(2, 0);
    double Kv = configuraciones(3, 0);

    std::cout << "Configuraciones definidas:" << std::endl;
    std::cout << "Tiempo de Simulación = " << tmax << std::endl;
    std::cout << "Actualización dt = " << dt << std::endl;
    std::cout << "Ganancia Kp = " << Kp << std::endl;
    std::cout << "Ganancia Kv = " << Kv << std::endl
              << std::endl;

    /***************************** ROS init **************************/
    ros::init(argc, argv, "formation_tracking");
    ros::NodeHandle nh;

    /************************** World Properties ******************/
    /*** Obtener las posiciones iniciales de los robots desde las propiedades del mundo ***/

    Eigen::MatrixXd x, qobs, qz;
    n_drones = A.cols();
    ROS_INFO("Numero de drones segun la matriz A.dat: %i", n_drones);
    x.resize(n_drones, dim);
    qz.resize(dim, n_drones);

    gazebo_msgs::GetModelState getModelState;
    geometry_msgs::Point pp;
    ros::ServiceClient gms_c = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    ros::ServiceClient gwp_c = nh.serviceClient<gazebo_msgs::GetWorldProperties>("gazebo/get_world_properties");
    gazebo_msgs::GetWorldProperties gwp_s;
    if (!gwp_c.call(gwp_s))
    {
        ROS_ERROR("No se pudo llamar al servicio \"get_world_properties\"");
        // std::cout << "No se pudo llamar al servicio \"get_world_properties\"" << std::endl;
        return 1;
    }
    else
    {
        std::vector<std::string> models = gwp_s.response.model_names;
        n_modelos = (int)models.size();
        ROS_INFO("Numero de modelos: %i", n_modelos);
        // std::cout << "Numero de modelos: " << n_modelos << std::endl;
        if (n_modelos == 1)
        { // Si hay un mundo vacio, solo habrá 1 modelo
            ROS_ERROR("Se ha cargado un mundo vacio. No se puede continuar");
            std::cout << "Se ha cargado un mundo vacio. No se puede continuar" << std::endl;
            return 1;
        }
        // Compara si el numero de drones es igual al de la matriz A.dat
        if (n_drones > (n_modelos - 1))
        {
            ROS_ERROR("Numero de drones no coincide con el numero de modelos");
            std::cout << "Numero de drones no coincide con el numero de modelos" << std::endl;
            return 1;
        }

        // Se reordena en forma lexicográfica los modelos
        // encontrados en el mundo para que coincidan con
        // la indexación

        // models[0] = ground_plane,
        // models[1, n_drones+1] = robots,
        // models[n_drones+2, end] = obstacles
        std::sort(models.begin(), models.end());

        Eigen::VectorXd current_pose(dim);
        // Se asume en 3D, cambiar la dimensión
        // para añadir más/menos: e.x: [x, y, z, yaw]
        // Obtener las posiciones de los drones
        for (int i = 0; i < n_drones; i++)
        {
            getModelState.request.model_name = models[i + 1];
            getModelState.request.relative_entity_name = relativeEntityName;
            gms_c.call(getModelState);
            pp = getModelState.response.pose.position;
            current_pose << pp.x, pp.y, pp.z;
            x.row(i) = current_pose;
        }

        ROS_INFO("Numero de drones: %i", n_drones);

        std::cout << std::endl
                  << "Posiciones iniciales: \n"
                  << x << std::endl
                  << sep << std::endl;
    }

    std::cout << "Simulacion iniciada" << sep << std::endl;

    /*****************************************************************/

    pos_msg.resize(n_drones);
    std::vector<std::string> topic_pub(n_drones);
    std::vector<std::string> topic_sub(n_drones);
    std::vector<ros::Publisher> pose_pub(n_drones);
    std::vector<ros::Subscriber> pose_sub(n_drones);
    std::vector<trajectory_msgs::MultiDOFJointTrajectory> MultiDOF_msgs(n_drones);
    ros::Rate rate(20); // Velocidad de actualización de la simulación según GAZEBO

    for (int i = 0; i < n_drones; i++)
    {
        topic_pub[i] = slash + dron_nombre + std::to_string(i + 1) + publisher_name;
        topic_sub[i] = slash + dron_nombre + std::to_string(i + 1) + subscriber_name;
        pose_pub[i] = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(topic_pub[i], 1);
        pose_sub[i] = nh.subscribe<geometry_msgs::PointStamped>(topic_sub[i], 1, boost::bind(positionCallback, _1, i));
    }

    Eigen::MatrixXd Px, xnew, v, u, u_old, Ki, Kii, x_temp;
    Eigen::VectorXd suma;
    int steps = tmax / dt, updated = 0, counter = 0;
    double t = 0;

    ROS_INFO("Numero de pasos en la simulación: %i", steps);

    if (steps != lider1.rows())
    {
        ROS_ERROR("Numero de pasos no coincide con el numero de filas de la matriz lider1 - lider2");
        std::cout << "Numero de pasos no coincide con el numero de filas de la matriz lider1 - lider2" << std::endl;
        return 1;
    }

    // Inicializar variables
    Px = Eigen::MatrixXd::Zero(dim, dim);     // Matriz de proyección
    v = Eigen::MatrixXd::Zero(n_drones, dim); // Vector de velocidad actual
    u = Eigen::MatrixXd::Zero(n_drones, dim); // Vector de entrada de control actual
    u_old = u;                                // Vector de entrada de control anterior
    Kii = Eigen::MatrixXd::Zero(dim, dim);    // Matriz de control de estabilidad
    x_temp = Eigen::MatrixXd::Zero(dim, 1);   // Vector temporal para guardar la posición actual

    /******************************************************************/
    /************************* SIMULACION *****************************/

    // Inicializacuión de la simulación
    while (ros::ok())
    {
        if (counter > steps-1)
            break;

        // hold for updates
        if (updated < 2)
        {
            rate.sleep();
            updated += 1;
            continue;
        }

        ros::spinOnce();

        // Simulación por medio del método de Euler
        for (int i = 0; i < n_drones; i++)
        {
            if (i != 2 && i != 6)
            {
                // Se reinician las matrices Suma y Ki debido 
                // a cambio de dron
                suma = Eigen::VectorXd::Zero(dim);
                Ki = Eigen::MatrixXd::Zero(dim, dim);

                // Cálculo de la matriz de pesos Ki
                for (int j = 0; j < n_drones; j++)
                {
                    if (A(i, j) == 1)
                    {
                        x_temp.col(0) = bearing.row(i * n_drones + j);
                        Px = ortProj(x_temp);
                        Ki += Px;
                    }
                }

                // Cálculo de estados del sistema y de líderes
                for (int j = 0; j < n_drones; j++)
                {
                    if (A(i, j) == 1)
                    {
                        x_temp.col(0) = bearing.row(i * n_drones + j);
                        Px = ortProj(x_temp);
                        x_temp.col(0) = Kp * (x.row(i) - x.row(j)) + Kv * (v.row(i) - v.row(j)) - u_old.row(j);
                        Kii = Ki.inverse() * Px;
                        suma += Kii * x_temp.col(0);
                    }
                }

                u.row(i) = -suma.col(0);
            }
            else
            {
                if (i == 2)
                {
                    u.row(i) = lider2.row(counter);
                }
                else
                {
                    u.row(i) = lider1.row(counter);
                }

            }
        }

        // Actualización de la posición y velocidad
        u_old = u;
        v += dt * u;
        x += dt * v;

        // Actualización de la posición de los drones
        // en la base de datos de Gazebo
        std::cout << "Iteracion: " << counter << std::endl;
        std::cout << "Velocidad nueva: \n"
                  << u << std::endl
                  << std::endl;
        std::cout << "Posición nueva: \n"
                  << x << std::endl
                  << std::endl;

        for (int i = 0; i < n_drones; i++)
        {
            trajectory_msgs::MultiDOFJointTrajectory msg;
            msg.header.stamp = ros::Time::now();
            mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(x.row(i), 1, &msg);
            pose_pub[i].publish(msg);
        }

        // Control de avance de la simulación
        // y pequeña pausa para evitar saturación de la CPU
        counter++;
        t += dt;
        rate.sleep();

        std::cout << sep << std::endl;
    }

    // Finalización de la simulación
    ROS_INFO("Simulación finalizada.");
    return 0;
}

/*
    Función la cual calcula la proyección ortogonal de un vector

        Parametros:
            x: Eigen::MatrixXd - Vector a proyectar de tamaño dim x 1
        
        Retorno:
            Eigen::MatrixXd - Matriz de proyección ortogonal de tamaño dim x dim
*/
Eigen::MatrixXd ortProj(Eigen::MatrixXd &x)
{
    Eigen::MatrixXd Px = Eigen::MatrixXd::Identity(dim, dim) - x * x.transpose() / (x.norm() * x.norm());
    return Px;
}

/*
    Multiple pose callback.
    Return the pose of the i-robot
*/
void positionCallback(const geometry_msgs::PointStamped::ConstPtr &msg, int index)
{
    pos_msg[index].point.x = msg->point.x;
    pos_msg[index].point.y = msg->point.y;
    pos_msg[index].point.z = msg->point.z;
}

/*
    Get current pose from odometry for each robot
*/
void getCurrentPose(Eigen::MatrixXd &q)
{
    Eigen::VectorXd current_pose(3);
    for (int i = 0; i < n_drones; i++)
    {
        current_pose(0) = pos_msg[i].point.x;
        current_pose(1) = pos_msg[i].point.y;
        current_pose(2) = pos_msg[i].point.z;
        q.col(i) = current_pose;
    }
}
