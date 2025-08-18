#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <csignal>

extern std::atomic_bool flag_stop;

void signalHandler (int signum);

Eigen::Vector3f skewVec (const Eigen::Matrix3f& R);
