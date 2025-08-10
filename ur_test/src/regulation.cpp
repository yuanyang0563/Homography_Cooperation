#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

Eigen::Vector3f skewVec (const Eigen::Matrix3f& R) {
	Eigen::Matrix3f S = 0.5*(R-R.transpose());
	Eigen::Vector3f v(S(2,1),S(0,2),S(1,0));
	return v;
}

class manipulator {

  public:
  
  	rclcpp::Node::SharedPtr node;
  	rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_jv;
  	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js;
  	Eigen::VectorXf a, alpha, d, theta, q, dq;
  	Eigen::Vector3f x, xd;
  	Eigen::Matrix3f R, Rd;
  	Eigen::MatrixXf J;
  	bool flag_js;
  	
  	manipulator (std::string name) {
  		node = std::make_shared<rclcpp::Node>(name);
  		pub_jv = node->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_velocity_controller/commands", 10);
  		sub_js = node->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&manipulator::joint_state_callback, this, std::placeholders::_1));
  		a = Eigen::VectorXf(6);
  		alpha = Eigen::VectorXf(6);
  		d = Eigen::VectorXf(6);
  		theta = Eigen::VectorXf(6);
  		q = Eigen::VectorXf(6);
  		dq = Eigen::VectorXf(6);
  		J = Eigen::MatrixXf(6, 6);
  		flag_js = false;
  	}
  	
  	void joint_state_callback (const sensor_msgs::msg::JointState::SharedPtr msg) {
  		q << msg->position[5], msg->position[0], msg->position[1], msg->position[2], msg->position[3], msg->position[4];
  		flag_js = true;
  	}
  	
  	void get_pose_jacobian () {
  		a << 0.0, -0.24365, -0.21325, 0.0, 0.0, 0.0;
		alpha << M_PI/2.0, 0.0, 0.0, M_PI/2.0, -M_PI/2.0, 0.0;
		d << 0.1519, 0.0, 0.0, 0.11235, 0.08535, 0.0819;
		theta << q;
  		Eigen::Matrix4f T;
  		T << cos(3.0*M_PI/4.0), -sin(3.0*M_PI/4.0), 0.0,  0.0,
  		     sin(3.0*M_PI/4.0),  cos(3.0*M_PI/4.0), 0.0, -0.66,
  		     0.0,                0.0,               1.0,  0.0,
  		     0.0,                0.0,               0.0,  1.0;
  		Eigen::Matrix<float, 3, 7> o, z;
  		o.col(0) << T.block(0,3,3,1);
  		z.col(0) << T.block(0,2,3,1);
  		Eigen::Matrix4f A;
  		for (size_t i=0; i<6; ++i) {
  			A << cos(theta(i)), -sin(theta(i))*cos(alpha(i)),  sin(theta(i))*sin(alpha(i)), a(i)*cos(theta(i)),
  			     sin(theta(i)),  cos(theta(i))*cos(alpha(i)), -cos(theta(i))*sin(alpha(i)), a(i)*sin(theta(i)),
  			     0.0,            sin(alpha(i)),                cos(alpha(i)),               d(i),
  			     0.0,            0.0,                          0.0,                         1.0;
  			T *=A;
  			o.col(i+1) = T.block(0,3,3,1);
  			z.col(i+1) = T.block(0,2,3,1);
  		}
  		x = T.block(0,3,3,1);
  		R = T.block(0,0,3,3);
  		for (size_t i=0; i<6; ++i)
  			J.col(i) << z.col(i).cross(o.col(6)-o.col(i)), z.col(i);
  		std::cout << "position: " << x.transpose() << std::endl;
  		std::cout << "orientation: " << std::endl;
  		std::cout << R << std::endl;
  	}
  	
  	void set_joint_velocity () {
  		dq.setZero();
  		//xd << -0.3, -0.4, 0.25;
  		//xd <<  0.3, -0.4, 0.25;
  		//xd <<  0.3, -0.3, 0.25;
  		//xd <<  0.3, -0.3, 0.4;
  		//xd << -0.3, -0.3, 0.4;
  		//xd << -0.3, -0.4, 0.4;
  		//xd << -0.3, -0.4, 0.25;
  		xd <<  0.0, -0.2, 0.4;
  		Rd << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0;
  		if (flag_js) {
  			dq.head(3) = 0.1*(xd-x);
  			dq.tail(3) = 0.1*skewVec(Rd*R.transpose());
  			dq = J.transpose()*(J*J.transpose()).inverse()*dq;
  		}
  		auto msg = std_msgs::msg::Float64MultiArray();
  		msg.data.resize(6);
  		for (size_t i=0; i<6; ++i)
  			msg.data[i] = dq(i);
  		pub_jv->publish(msg);
  	}

};



int main (int argc, char **argv) {
	
	rclcpp::init(argc, argv);
	rclcpp::Rate loop_rate(125);
	manipulator arm("ur3");
	
	while (rclcpp::ok()) {
		arm.get_pose_jacobian();
		arm.set_joint_velocity();
		rclcpp::spin_some(arm.node);
		loop_rate.sleep();
	}
	rclcpp::shutdown();
	
	return 0;
}
