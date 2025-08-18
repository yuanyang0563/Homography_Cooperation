#include "common.hpp"

class manipulator {

    public:
    
    	rclcpp::Node::SharedPtr node;
    	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel;
    	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js;
    	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img;
    	Eigen::VectorXf a, alpha, d, theta;
    	Eigen::Vector3f xco, xec, xe, xc, x0, xd;
    	Eigen::Matrix3f Rco, Rec, Re, Rc, R0, Rd;
    	Eigen::VectorXf q, s, sd, twist;
    	Eigen::MatrixXf L, J, Tec, Tco;
    	Eigen::Vector3f ncd, mcd;
    	Eigen::Matrix3f Hc;
    	vpDisplayX display;
    	vpCameraParameters camera;
    	vpImage<unsigned char> image;
    	vpDetectorAprilTag detector;
    	vpHomogeneousMatrix cMo;
    	std::vector<vpImagePoint> tagsCorners;
    	std::vector<std::vector<double>> p, pd;
    	
    	manipulator (std::string name) {
    		node = std::make_shared<rclcpp::Node>(name);
    		pub_vel = node->create_publisher<geometry_msgs::msg::Twist>("/twist_controller/commands", 10);
    		sub_js = node->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&manipulator::joint_state_callback, this, std::placeholders::_1));
    		sub_img = node->create_subscription<sensor_msgs::msg::Image>("/camera/color/image_raw", 10, std::bind(&manipulator::camera_image_callback, this, std::placeholders::_1));
    		a = Eigen::VectorXf(7);
    		alpha = Eigen::VectorXf(7);
    		d = Eigen::VectorXf(7);
    		theta = Eigen::VectorXf(7);
    		x0 << 0.0, 0.0, 0.35;
		R0 << 1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0;
    		q = Eigen::VectorXf(7);
    		s = Eigen::VectorXf(8);
    		sd = Eigen::VectorXf(8);
    		twist = Eigen::VectorXf(6);
    		L = Eigen::MatrixXf(8, 6);
    		J = Eigen::MatrixXf(6, 6);
    		xec << 0.0, 0.055, 0.0;
    		Rec << -1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0;
    		Tec = Eigen::MatrixXf(6, 6);
    		Tec.block(0,0,3,3) = Rec;
    		Tec.block(0,3,3,3) = skewMat(xec)*Rec;
    		Tec.block(3,0,3,3) = Eigen::Matrix3f::Zero();
    		Tec.block(3,3,3,3) = Rec;
    		Tco = Eigen::MatrixXf(6, 6);
    		ncd << 0.0, 0.0, 1.0;
    		camera.initPersProjWithoutDistortion(1297.7, 1298.6, 620.9, 238.3);
    		image.resize(720, 1280);
    		display.init(image, 0, 0, "Camera Image");
    		detector = vpDetectorAprilTag(vpDetectorAprilTag::TAG_36h11);
    		detector.setAprilTagQuadDecimate(2);
    		detector.setAprilTagNbThreads(1);
    		detector.setAprilTagPoseEstimationMethod(vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS);
    		p.resize(2);
    		pd.resize(2);
    		for (size_t i=0; i<2; ++i) {
    			p[i].resize(4);
    			pd[i].resize(4);
    		}
    	}
    	
    	void joint_state_callback (const sensor_msgs::msg::JointState::SharedPtr msg);
    	void camera_image_callback (const sensor_msgs::msg::Image::SharedPtr msg);
    	
    	void getPose ();
    	void getFeatures ();
    	void setTwist();
};
