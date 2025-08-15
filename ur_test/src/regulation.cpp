#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <csignal>
#include <termios.h>
#include <fcntl.h>

std::atomic_bool flag_stop = false;
void signalHandler (int signum) {
    	flag_stop = true;
}

int kbhit() {
    struct termios oldt, newt;
    int ch;
    int oldf;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO); // disable buffering & echo
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    if (ch != EOF) {
        ungetc(ch, stdin);
        return 1;
    }
    return 0;
}

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
  	std::vector<Eigen::Vector3f> xd;
	std::vector<Eigen::Matrix3f> Rd;
  	Eigen::Vector3f x, xr;
  	Eigen::Matrix3f R, Rr;
  	Eigen::MatrixXf J;
  	char key1, key2;
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
  		xd.resize(7);
		xd[0] <<  0.0, -0.2, 0.4;
  		xd[1] << -0.3, -0.4, 0.25;
  		xd[2] <<  0.3, -0.4, 0.25;
  		xd[3] <<  0.3, -0.3, 0.25;
  		xd[4] <<  0.3, -0.3, 0.4;
  		xd[5] << -0.3, -0.3, 0.4;
  		xd[6] << -0.3, -0.4, 0.4;
  		Rd.resize(7);
  		Rd[0] << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0;
  		Rd[1] << 1.0, 0.0, 0.0, 0.0,  sin(M_PI/6.0), cos(M_PI/6.0), 0.0, -cos(M_PI/6.0),  sin(M_PI/6.0);
  		Rd[2] << 1.0, 0.0, 0.0, 0.0, -sin(M_PI/6.0), cos(M_PI/6.0), 0.0, -cos(M_PI/6.0), -sin(M_PI/6.0);
  		Rd[3] << cos(M_PI/4.0), 0.0,  sin(M_PI/4.0), -sin(M_PI/4.0), 0.0, cos(M_PI/4.0), 0.0, -1.0, 0.0;
  		Rd[4] << cos(M_PI/4.0), 0.0, -sin(M_PI/4.0),  sin(M_PI/4.0), 0.0, cos(M_PI/4.0), 0.0, -1.0, 0.0;
  		Rd[5] << cos(M_PI/4.0), -sin(M_PI/4.0), 0.0, 0.0, 0.0, 1.0, -sin(M_PI/4.0), -cos(M_PI/4.0), 0.0;
  		Rd[6] << cos(M_PI/4.0),  sin(M_PI/4.0), 0.0, 0.0, 0.0, 1.0,  sin(M_PI/4.0), -cos(M_PI/4.0), 0.0;
  		xr = xd[0];
  		Rr = Rd[0];
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
  	}
  	
  	void set_joint_velocity () {
  		dq.setZero();
  		if (flag_js && !flag_stop) {
  			if (key1=='x' && key2>='0' && key2<='6')
  				xr = xd[key2-'0'];
  			if (key1=='R' && key2>='0' && key2<='6')
  				Rr = Rd[key2-'0'];
  			dq.head(3) = 0.1*(xr-x);
  			dq.tail(3) = 0.1*skewVec(Rr*R.transpose());
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
	
	// make the last joint velocity message be zero before the node shuts down
	std::signal(SIGINT, signalHandler);
	
	while (rclcpp::ok() && !flag_stop) {
		if (kbhit()) {
            		arm.key1 = getchar();
            		while (rclcpp::ok() && !flag_stop) {
            			if (kbhit()) {
            				arm.key2 = getchar();
            				std::cout << arm.key2 << std::endl;
            				break;
            			}
            		}
        	}
		arm.get_pose_jacobian();
		arm.set_joint_velocity();
		rclcpp::spin_some(arm.node);
		loop_rate.sleep();
	}
	arm.set_joint_velocity();
	rclcpp::shutdown();
	
	return 0;
}
