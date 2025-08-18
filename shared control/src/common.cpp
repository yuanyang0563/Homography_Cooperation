#include "common.hpp"


bool flag_js = false;
bool flag_at = false;
bool flag_init = false;
float tagSize = 0.04;
float lambda_ibvs = 0.35;
float lambda_pbvs = 1.50;
float lambda_hbvs = 0.40;
std::string mode;
std::atomic_bool flag_stop = false;

void signalHandler (int signum) {
    	flag_stop = true;
}

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
