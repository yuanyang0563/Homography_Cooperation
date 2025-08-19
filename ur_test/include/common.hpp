#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <termios.h>
#include <fcntl.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <csignal>

extern int x_wp;
extern int R_wp;
extern char key1;
extern char key2;
extern float lambda_u;
extern float lambda_o;
extern float upsilon_lim;
extern float omega_lim;
extern bool flag_js;
extern bool flag_data;
extern std::atomic_bool flag_stop;

void signalHandler (int signum);

Eigen::Vector3f skewVec (const Eigen::Matrix3f& R);

Eigen::Vector3f limitVelocity (Eigen::Vector3f velocity, float vel_lim);

int hitKey();
