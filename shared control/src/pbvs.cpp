#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/gui/vpDisplayX.h>

#include <csignal>

Eigen::Vector3f skewVec (const Eigen::Matrix3f& R) {
	Eigen::Matrix3f S = 0.5*(R-R.transpose());
	Eigen::Vector3f v(S(2,1),S(0,2),S(1,0));
	return v;
}

Eigen::Matrix3f skewMat (const Eigen::Vector3f& v) {
	Eigen::Matrix3f R;
	R << 0.0, -v(2), v(1), v(2), 0.0, -v(0), -v(1), v(0), 0.0;
	return R;
}

std::atomic_bool flag_stop = false;
void signalHandler (int signum) {
    	flag_stop = true;
}

class manipulator {

    public:
    
    	rclcpp::Node::SharedPtr node;
    	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel;
    	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js;
    	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img;
    	Eigen::VectorXf a, alpha, d, theta, q;
    	Eigen::Vector3f xco, xec, xe, xc, xd;
    	Eigen::Matrix3f Rco, Rec, Re, Rc, Rd;
    	Eigen::VectorXf s, sd, twist;
    	Eigen::MatrixXf L, Tec, Tco;
    	float lambda;
    	double tagSize;
    	vpDisplayX display;
    	vpCameraParameters camera;
    	vpImage<unsigned char> image;
    	vpDetectorAprilTag detector;
    	vpHomogeneousMatrix cMo;
    	std::vector<vpImagePoint> tagsCorners;
    	bool flag_js, flag_at, flag_init;
    	
    	manipulator (std::string name) {
    		node = std::make_shared<rclcpp::Node>(name);
    		pub_vel = node->create_publisher<geometry_msgs::msg::Twist>("/twist_controller/commands", 10);
    		sub_js = node->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&manipulator::joint_state_callback, this, std::placeholders::_1));
    		sub_img = node->create_subscription<sensor_msgs::msg::Image>("/camera/color/image_raw", 10, std::bind(&manipulator::camera_image_callback, this, std::placeholders::_1));
    		a = Eigen::VectorXf(7);
    		alpha = Eigen::VectorXf(7);
    		d = Eigen::VectorXf(7);
    		theta = Eigen::VectorXf(7);
    		q = Eigen::VectorXf(7);
    		s = Eigen::VectorXf(8);
    		sd = Eigen::VectorXf(8);
    		twist = Eigen::VectorXf(6);
    		L = Eigen::MatrixXf(8, 6);
    		lambda = 1.5;
    		xec << 0.0, 0.055, 0.0;
    		Rec << -1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0;
    		Tec = Eigen::MatrixXf(6, 6);
    		Tec.block(0,0,3,3) = Rec;
    		Tec.block(0,3,3,3) = skewMat(xec)*Rec;
    		Tec.block(3,0,3,3) = Eigen::Matrix3f::Zero();
    		Tec.block(3,3,3,3) = Rec;
    		Tco = Eigen::MatrixXf(6, 6);
    		tagSize = 0.04;
    		camera.initPersProjWithoutDistortion(1297.7, 1298.6, 620.9, 238.3);
    		image.resize(720, 1280);
    		display.init(image, 0, 0, "Camera Image");
    		detector = vpDetectorAprilTag(vpDetectorAprilTag::TAG_36h11);
    		detector.setAprilTagQuadDecimate(2);
    		detector.setAprilTagNbThreads(1);
    		detector.setAprilTagPoseEstimationMethod(vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS);
    		flag_js = false;
    		flag_at = false;
    		flag_init = false;
    	}
    	
    	void joint_state_callback (const sensor_msgs::msg::JointState::SharedPtr msg) {
    		q << msg->position[0], msg->position[2], msg->position[5], msg->position[3], msg->position[4], msg->position[6], msg->position[7];
    		flag_js = true;
    	}
    	
    	void camera_image_callback (const sensor_msgs::msg::Image::SharedPtr msg) {
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
    		xe = T.block(0,3,3,1);
    		Re = T.block(0,0,3,3);
    		xc = Re*xec+xe;
    		Rc = Re*Rec;
    	}
    	
    	void getFeatures () {
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
    			}
    			if (!flag_init) {
    				xd = xco;
    				Rd = Rco;
    				sd = s;
    				flag_init = true;
    			}
    		}
    	}
    	
    	void setTwist () {
    		twist.setZero();
    		if (flag_init && !flag_stop) {
    			Tco.block(0,0,3,3) = -Rco;
    			Tco.block(0,3,3,3) = -skewMat(xco)*Rco;
    			Tco.block(3,0,3,3) = Eigen::Matrix3f::Zero();
    			Tco.block(3,3,3,3) = -Rco;
    			twist << Rco.transpose()*(xd-xco), skewVec(Rco.transpose()*Rd);
    			twist = lambda*Tec*Tco*twist;
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
    	
};



int main (int argc, char **argv) {

	rclcpp::init(argc, argv);
	rclcpp::Rate loop_rate(100);
	manipulator arm("gen3");
	
	// make the last twist message be zero before the node shuts down
	std::signal(SIGINT, signalHandler);
	
	while (rclcpp::ok() && !flag_stop) {
		arm.getFeatures();
		arm.setTwist();
		rclcpp::spin_some(arm.node);
		loop_rate.sleep();
	}
	arm.setTwist();
	rclcpp::shutdown();
	
	return 0;
	
}
