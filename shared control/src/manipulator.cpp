#include "manipulator.hpp"

void manipulator::joint_state_callback (const sensor_msgs::msg::JointState::SharedPtr msg) {
    	q << msg->position[0], msg->position[2], msg->position[5], msg->position[3], msg->position[4], msg->position[6], msg->position[7]+0.2;
    	flag_js = true;
}

void manipulator::camera_image_callback (const sensor_msgs::msg::Image::SharedPtr msg) {
    	try {
		for (int i=0; i<msg->height; ++i) {
                	for (int j=0; j<msg->width; ++j)
                		image[i][j] = msg->data[i*msg->step+3*j];
            	}
		flag_at = detector.detect(image);
		vpDisplay::display(image);
		if (flag_init)
			vpDisplay::displayFrame(image, cMo, camera, tagSize, vpColor::none, 2);
		vpDisplay::flush(image);
    	} catch (const vpException &e) {
    		RCLCPP_ERROR(node->get_logger(), e.getMessage());
    	}
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
    	xe = T.block(0,3,3,1);
    	Re = T.block(0,0,3,3);
    	xc = Re*xec+xe;
    	Rc = Re*Rec;
}

void manipulator::getFeatures () {
    	if (flag_at) {
    		detector.getPose(0, tagSize, camera, cMo);
    		for (size_t i=0; i<3; ++i) {
    			for (size_t j=0; j<3; ++j)
    				Rco(i,j) = cMo[i][j];
    			xco(i) = cMo[i][3];
    		}
    		tagsCorners = detector.getTagsCorners()[0];
    		for (size_t i=0; i<4; ++i) {
    			s(2*i+0) = (tagsCorners[i].get_u()-camera.get_u0())*camera.get_px_inverse();
    			s(2*i+1) = (tagsCorners[i].get_v()-camera.get_v0())*camera.get_py_inverse();
    			p[0][i] = s(2*i+0);
    			p[1][i] = s(2*i+1);
    		}
    		if (!flag_init) {
    			pd = p;
    			sd = s;
    			xd = xco;
    			Rd = Rco;
    			mcd << 0.25*(p[0][0]+p[0][1]+p[0][2]+p[0][3]), 0.25*(p[1][0]+p[1][1]+p[1][2]+p[1][3]), 1.0;
    			flag_init = true;
    		}
    	}
}
    	
void manipulator::setTwist () {
    	twist.setZero();
    	if (flag_js && !flag_stop) {
    		if (mode=="home") {
    			twist.head(3) = 0.5*Re.transpose()*(x0-xe);
    			twist.tail(3) = 0.5*Re.transpose()*skewVec(R0*Re.transpose());
    		}
    		if (mode=="ibvs" && flag_init) {
    			for (size_t i=0; i<4; ++i) {
    				float x = s(2*i+0);
    				float y = s(2*i+1);
    				L.row(2*i+0) << -1.0, 0.0, x, x*y, -(1.0+x*x), y;
    				L.row(2*i+1) << 0.0, -1.0, y, 1.0+y*y, -x*y, -x;
    			}
    			twist = lambda_ibvs*Tec*(L.transpose()*L).inverse()*L.transpose()*(sd-s);
    		}
    		if (mode=="pbvs" && flag_init) {
    			Tco.block(0,0,3,3) = -Rco;
    			Tco.block(0,3,3,3) = -skewMat(xco)*Rco;
    			Tco.block(3,0,3,3) = Eigen::Matrix3f::Zero();
    			Tco.block(3,3,3,3) = -Rco;
    			twist << Rco.transpose()*(xd-xco), skewVec(Rco.transpose()*Rd);
    			twist = lambda_pbvs*Tec*Tco*twist;
    		}
    		if (mode=="hbvs" && flag_init) {
    			vpHomography H;
    			vpHomography::DLT(pd[0], pd[1], p[0], p[1], H, true);
    			H /= H[2][2];
    			for (size_t i=0; i<3; ++i) {
    				for (size_t j=0; j<3; ++j)
    					Hc(i,j) = H[i][j];
    			}
    			J.block(0,0,3,3) = ncd.transpose()*mcd*Eigen::Matrix3f::Identity();
    			J.block(0,3,3,3) = -skewMat(Hc*mcd);
    			J.block(3,0,3,3) = 0.5*skewMat(ncd);
    			J.block(3,3,3,3) = 0.5*(Hc.trace()*Eigen::Matrix3f::Identity()-Hc);
    			Eigen::MatrixXf twist_h(6, 1);
    			twist_h << (Hc-Eigen::Matrix3f::Identity())*mcd, skewVec(Hc);
    			twist += lambda_hbvs*Tec*J.transpose()*(J*J.transpose()).inverse()*twist_h;    			
    			Tco.block(0,0,3,3) = -Rco;
    			Tco.block(0,3,3,3) = -skewMat(xco)*Rco;
    			Tco.block(3,0,3,3) = Eigen::Matrix3f::Zero();
    			Tco.block(3,3,3,3) = -Rco;
    			Eigen::MatrixXf twist_p(6, 1);
    			twist_p << Rco.transpose()*(xd-xco), skewVec(Rco.transpose()*Rd);
    			twist += 2.0*lambda_hbvs*Tec*Tco*twist_p;
    		}
    	}
    	auto msg = geometry_msgs::msg::Twist();
    	msg.linear.x = twist(0);
    	msg.linear.y = twist(1);
    	msg.linear.z = twist(2);
    	msg.angular.x = 100.0*twist(3);
    	msg.angular.y = 100.0*twist(4);
    	msg.angular.z = 100.0*twist(5);
    	pub_vel->publish(msg);
}

