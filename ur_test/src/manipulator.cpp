#include "manipulator.hpp"

manipulator::manipulator (const std::string& name) {
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
  	xd.resize(8);
	xd[0] <<  0.0, -0.2, 0.4;
  	xd[1] << -0.3, -0.4, 0.25;
  	xd[2] <<  0.3, -0.4, 0.25;
  	xd[3] <<  0.3, -0.3, 0.25;
  	xd[4] <<  0.3, -0.3, 0.4;
  	xd[5] << -0.3, -0.3, 0.4;
  	xd[6] << -0.3, -0.4, 0.4;
  	xd[7] <<  0.0, -0.2, 0.4;
  	Rd.resize(8);
  	Rd[0] << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0;
  	Rd[1] << 1.0, 0.0, 0.0, 0.0,  sin(M_PI/9.0), cos(M_PI/9.0), 0.0, -cos(M_PI/9.0),  sin(M_PI/9.0);
  	Rd[2] << 1.0, 0.0, 0.0, 0.0, -sin(M_PI/9.0), cos(M_PI/9.0), 0.0, -cos(M_PI/9.0), -sin(M_PI/9.0);
  	Rd[3] << cos(M_PI/4.0), 0.0,  sin(M_PI/4.0), -sin(M_PI/4.0), 0.0, cos(M_PI/4.0), 0.0, -1.0, 0.0;
  	Rd[4] << cos(M_PI/4.0), 0.0, -sin(M_PI/4.0),  sin(M_PI/4.0), 0.0, cos(M_PI/4.0), 0.0, -1.0, 0.0;
  	Rd[5] << cos(M_PI/3.0), -sin(M_PI/3.0), 0.0, 0.0, 0.0, 1.0, -sin(M_PI/3.0), -cos(M_PI/3.0), 0.0;
  	Rd[6] << cos(M_PI/3.0),  sin(M_PI/3.0), 0.0, 0.0, 0.0, 1.0,  sin(M_PI/3.0), -cos(M_PI/3.0), 0.0;
  	Rd[7] << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0;
  	xr = xd[0];
  	Rr = Rd[0];
  	data.resize(6);
  	for (size_t i=0; i<6; ++i)
  		data[i] = &q(i);
  	file_name << "../data/ur3_" <<std::fixed << std::setprecision(0) << node->now().seconds() << ".txt";
}

void manipulator::joint_state_callback (const sensor_msgs::msg::JointState::SharedPtr msg) {
	q << msg->position[5], msg->position[0], msg->position[1], msg->position[2], msg->position[3], msg->position[4];
  	flag_js = true;
}
  	
void manipulator::get_pose_jacobian () {
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
  	
void manipulator::set_joint_velocity () {
	dq.setZero();
  	if (flag_js && !flag_stop) {
  		xr = xd[x_wp];
  		Rr = Rd[R_wp];
  		if (key1=='x' && key2>='0' && key2<='7')
  			xr = xd[key2-'0'];
  		if (key1=='R' && key2>='0' && key2<='7')
  			Rr = Rd[key2-'0'];
  		dq.head(3) = limitVelocity(lambda_u*(xr-x), upsilon_lim);
  		dq.tail(3) = limitVelocity(lambda_o*skewVec(Rr*R.transpose()), omega_lim);
  		dq = J.transpose()*(J*J.transpose()).inverse()*dq;
  		if (dq.norm()<0.001) {
  			if (x_wp<7)
  				x_wp += 1;
  			else if (R_wp<7)
  				R_wp += 1;
  		}
  	}
  	auto msg = std_msgs::msg::Float64MultiArray();
  	msg.data.resize(6);
  	for (size_t i=0; i<6; ++i)
  		msg.data[i] = dq(i);
  	pub_jv->publish(msg);
}

void manipulator::store_data () {
	std::ofstream data_stream;
	if (!flag_data) {
		data_stream.open(file_name.str());
		flag_data = !flag_data;
	} else {
		data_stream.open(file_name.str(), std::ios_base::app);
	}
	data_stream << std::setiosflags(std::ios::fixed) << std::setprecision(2) << node->now().seconds();
	std::vector<float*>::iterator it;
	for (it=data.begin(); it!=data.end(); ++it)
		data_stream << ", " << std::setprecision(4) << **it;
	data_stream << std::endl;
	data_stream.close();
}
