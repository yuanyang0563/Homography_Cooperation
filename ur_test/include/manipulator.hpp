#include "common.hpp"

class manipulator {

  public:
  
  	rclcpp::Node::SharedPtr node;  	
  	explicit manipulator (const std::string& name);  	
  	void joint_state_callback (const sensor_msgs::msg::JointState::SharedPtr msg);  	
  	void get_pose_jacobian();  	
  	void set_joint_velocity();
  	void store_data();
  	
  private:
  
  	rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_jv;
  	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js;
  	Eigen::VectorXf a, alpha, d, theta, q, dq;
  	Eigen::MatrixXf J;
  	std::vector<Eigen::Vector3f> xd;
	std::vector<Eigen::Matrix3f> Rd;
  	Eigen::Vector3f x, xr;
  	Eigen::Matrix3f R, Rr;
	std::stringstream file_name;
    	std::vector<float*> data;

};
