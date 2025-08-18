#include "common.hpp"

class manipulator {

    public:
    
    	rclcpp::Node::SharedPtr node;
    	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub;
    	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub;
    	Eigen::VectorXf a, alpha, d, theta;
    	Eigen::VectorXf q, twist;
    	Eigen::Vector3f x, xd;
    	Eigen::Matrix3f R, Rd;
    	Eigen::Vector3f upsilon, omega;
    	bool flag_init;
    	
    	manipulator (std::string name) {
    		node = std::make_shared<rclcpp::Node>(name);
    		pub = node->create_publisher<geometry_msgs::msg::Twist>("/twist_controller/commands", 10);
    		sub = node->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&manipulator::joint_state_callback, this, std::placeholders::_1));
    		a = Eigen::VectorXf(7);
    		alpha = Eigen::VectorXf(7);
    		d = Eigen::VectorXf(7);
    		theta = Eigen::VectorXf(7);
    		q = Eigen::VectorXf(7);
    		twist = Eigen::VectorXf(6);
    		xd << 0.0, 0.0, 0.35;
		Rd << 1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0;
    		flag_init = false;
    	}
    	
    	void joint_state_callback (const sensor_msgs::msg::JointState::SharedPtr msg);
    	
    	void getPose ();
    	
    	void setTwist ();
};
