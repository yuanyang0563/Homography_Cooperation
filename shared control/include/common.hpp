#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/vision/vpHomography.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/gui/vpDisplayX.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <csignal>

extern bool flag_js;
extern bool flag_at;
extern bool flag_init;
extern float tagSize;
extern float lambda_ibvs;
extern float lambda_pbvs;
extern float lambda_hbvs;
extern std::string mode;
extern std::atomic_bool flag_stop;

void signalHandler (int signum);

Eigen::Vector3f skewVec (const Eigen::Matrix3f& R);

Eigen::Matrix3f skewMat (const Eigen::Vector3f& v);
