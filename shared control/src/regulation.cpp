#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <csignal>

Eigen::Vector3f skewVec (const Eigen::Matrix3f& R) {
	Eigen::Matrix3f S = 0.5*(R-R.transpose());
	Eigen::Vector3f v(S(2,1),S(0,2),S(1,0));
	return v;
}

std::atomic_bool flag_stop = false;
void signalHandler (int signum) {
    	flag_stop = true;
}

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
    	
    	void joint_state_callback (const sensor_msgs::msg::JointState::SharedPtr msg) {
    		q << msg->position[0], msg->position[2], msg->position[5], msg->position[3], msg->position[4], msg->position[6], msg->position[7]+0.2;
    		flag_init = true;
    	}
    	
    	void getPose () {
    		a << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    		alpha << -M_PI/2.0, M_PI/2.0, -M_PI/2.0, M_PI/2.0, -M_PI/2.0, M_PI/2.0, 0.0;
    		d << 0.1564+0.1284, -0.0054-0.0064, 0.2104+0.2104, -0.0064-0.0064, 0.2084+0.1059, 0.0, 0.1059+0.0615;
    		theta << -q(0), q(1), -q(2), q(3), -q(4), q(5), -q(6);
    		Eigen::Matrix4f T;
    		T << cos(-M_PI/4.0), -sin(-M_PI/4.0), 0.0, 0.0,
    		     sin(-M_PI/4.0),  cos(-M_PI/4.0), 0.0, 0.44,
    		     0.0,             0.0,            1.0, 0.02,
    		     0.0,             0.0,            0.0, 1.0;
    		Eigen::Matrix4f A;
    		for (size_t i=0; i<7; ++i) {
    			A << cos(theta(i)), -sin(theta(i))*cos(alpha(i)),  sin(theta(i))*sin(alpha(i)), a(i)*cos(theta(i)),
    			     sin(theta(i)),  cos(theta(i))*cos(alpha(i)), -cos(theta(i))*sin(alpha(i)), a(i)*sin(theta(i)),
    			     0.0,            sin(alpha(i)),                cos(alpha(i)),               d(i),
    			     0.0,            0.0,                          0.0,                         1.0;
    			T *= A;
    		}
    		x = T.block(0,3,3,1);
    		R = T.block(0,0,3,3);
    	}
    	
    	void setTwist () {
    		twist.setZero();
    		if (flag_init && !flag_stop) {
    			twist.head(3) = 0.5*R.transpose()*(xd-x);
    			twist.tail(3) = 10.0*R.transpose()*skewVec(Rd*R.transpose());
    		}
    		auto msg = geometry_msgs::msg::Twist();
    		msg.linear.x = twist(0);
    		msg.linear.y = twist(1);
    		msg.linear.z = twist(2);
    		msg.angular.x = twist(3);
    		msg.angular.y = twist(4);
    		msg.angular.z = twist(5);
    		pub->publish(msg);
    	}
};



int main (int argc, char **argv) {

	rclcpp::init(argc, argv);
	rclcpp::Rate loop_rate(100);
	manipulator arm("gen3");
	
	// make the last twist message be zero before the node shuts down
	std::signal(SIGINT, signalHandler);
	
	while (rclcpp::ok() && !flag_stop) {
		arm.getPose();
		arm.setTwist();
		rclcpp::spin_some(arm.node);
		loop_rate.sleep();
	}
	arm.setTwist();
	rclcpp::shutdown();
	
	return 0;
	
}
