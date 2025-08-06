#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

Eigen::Vector3f skewVec (const Eigen::Matrix3f& R) {
	Eigen::Matrix3f S = 0.5*(R-R.transpose());
	Eigen::Vector3f v(S(2,1),S(0,2),S(1,0));
	return v;
}

class manipulator {

    public:
    
    	rclcpp::Node::SharedPtr node;
    	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel;
    	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js;
    	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img;
    	Eigen::VectorXf a, alpha, d, theta;
    	Eigen::VectorXf q;
    	Eigen::Vector3f x, xd;
    	Eigen::Matrix3f R, Rd;
    	Eigen::Vector3f upsilon, omega;
    	bool flag;
    	cv::aruco::Dictionary dictionary;
    	cv::aruco::DetectorParameters detectorParams;
    	cv::aruco::ArucoDetector detector;
    	
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
    		flag = false;
    		dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
    		detectorParams.cornerRefinementMethod = cv::aruco::CORNER_REFINE_APRILTAG;
    		detector = cv::aruco::ArucoDetector(dictionary, detectorParams);
    	}
    	
    	void joint_state_callback (const sensor_msgs::msg::JointState::SharedPtr msg) {
    		q << msg->position[0], msg->position[2], msg->position[5], msg->position[3], msg->position[4], msg->position[6], msg->position[7]+0.2;
    		getPose(q);
    		if (!flag)
    			flag = true;
    	}
    	
    	void camera_image_callback (const sensor_msgs::msg::Image::SharedPtr msg) {
    		try {
    			cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    			//cv::Mat image = cv_ptr->image;
    			/*
    			std::vector<int> ids;
    			std::vector<std::vector<cv::Point2f>> corners;
    			detector.detectMarkers(image, corners, ids);
    			if (!ids.empty()) {
    				cv::aruco::drawDetectedMarkers(image, corners, ids);
    				for (size_t i=0; i<4; ++i)
    					std::cout << "corner " << i << ": (" << corners[0][i].x << ", " << corners[0][i].y << ")" << std::endl;
    			}
    			*/
    			if (!cv_ptr->image.empty()) {
    			cv::imshow("Camera Image", cv_ptr->image);
    			}
    			cv::waitKey(1);
    		} catch (const cv_bridge::Exception &e) {
    			RCLCPP_ERROR(node->get_logger(), e.what());
    		}
    	}
    	
    	void getPose (const Eigen::VectorXf& q) {
    		a << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    		alpha << -M_PI/2.0, M_PI/2.0, -M_PI/2.0, M_PI/2.0, -M_PI/2.0, M_PI/2.0, 0.0;
    		d << 0.1564+0.1284, -0.0054-0.0064, 0.2104+0.2104, -0.0064-0.0064, 0.2084+0.1059, 0.0, 0.1059+0.0615;
    		theta << -q(0), q(1), -q(2), q(3), -q(4), q(5), -q(6);
    		Eigen::Matrix4f T;
    		T << cos(-M_PI/4.0), -sin(-M_PI/4.0), 0.0, 0.0,
    		     sin(-M_PI/4.0),  cos(-M_PI/4.0), 0.0, 0.0,
    		     0.0,             0.0,            1.0, 0.0,
    		     0.0,             0.0,            0.0, 1.0;
    		Eigen::Matrix<float, 3, 8> o, z;
    		o.col(0) << T.block(0,3,3,1);
    		z.col(0) << T.block(0,2,3,1);
    		Eigen::Matrix4f A;
    		for (size_t i=0; i<7; ++i) {
    			A << cos(theta(i)), -sin(theta(i))*cos(alpha(i)),  sin(theta(i))*sin(alpha(i)), a(i)*cos(theta(i)),
    			     sin(theta(i)),  cos(theta(i))*cos(alpha(i)), -cos(theta(i))*sin(alpha(i)), a(i)*sin(theta(i)),
    			     0.0,            sin(alpha(i)),                cos(alpha(i)),               d(i),
    			     0.0,            0.0,                          0.0,                         1.0;
    			T *= A;
    			o.col(i+1) << T.block(0,3,3,1);
    			z.col(i+1) << T.block(0,2,3,1);
    		}
    		x = T.block(0,3,3,1);
    		R = T.block(0,0,3,3);
    	}
    	
    	void setTwist () {
    		upsilon = 0.0*R.transpose()*(xd-x);
    		omega = 0.0*R.transpose()*skewVec(Rd*R.transpose());
    		
    		auto msg = geometry_msgs::msg::Twist();
    		msg.linear.x = upsilon(0);
    		msg.linear.y = upsilon(1);
    		msg.linear.z = upsilon(2);
    		msg.angular.x = omega(0);
    		msg.angular.y = omega(1);
    		msg.angular.z = omega(2);
    		if (flag)
    			pub_vel->publish(msg);
    	}
};



int main (int argc, char **argv) {

	rclcpp::init(argc, argv);
	rclcpp::Rate loop_rate(100);
	manipulator arm("gen3");
	
	arm.xd << 0.0, -0.5, 0.3;
	arm.Rd << 1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0;
	
	while (rclcpp::ok()) {
		arm.setTwist();
		rclcpp::spin_some(arm.node);
		loop_rate.sleep();
	}
	rclcpp::shutdown();
	
	return 0;
	
}
