#include "common.hpp"

class manipulator {

    public:
    
    	rclcpp::Node::SharedPtr node;    	    	    	
    	
    	explicit manipulator (const std::string& name);
    	
    	void getPose();
    	void getFeatures();
    	void setTwist();
    	void storeData();
    
    private:
    	    	
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
    	
    	std::vector<std::vector<vpImagePoint>> tagCorners_traj;
    	std::vector<vpImagePoint> tagCorners, tagCorners0;
    	std::vector<std::vector<double>> p, pd;
    	vpImage<unsigned char> image;
    	vpDetectorAprilTag detector;
    	vpCameraParameters camera;
    	vpHomogeneousMatrix cMo;
    	vpDisplayX display;
    	
    	std::stringstream file_name;
    	std::vector<float*> data;
    	
    	void joint_state_callback (const sensor_msgs::msg::JointState::SharedPtr msg);
    	void camera_image_callback (const sensor_msgs::msg::Image::SharedPtr msg);
};
