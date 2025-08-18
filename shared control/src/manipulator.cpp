#include "manipulator.hpp"

void manipulator::joint_state_callback (const sensor_msgs::msg::JointState::SharedPtr msg) {
    	q << msg->position[0], msg->position[2], msg->position[5], msg->position[3], msg->position[4], msg->position[6], msg->position[7]+0.2;
    	flag_init = true;
}
    	
void manipulator::getPose () {
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
    	
void manipulator::setTwist () {
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
