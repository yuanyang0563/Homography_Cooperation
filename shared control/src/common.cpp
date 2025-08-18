#include "common.hpp"

std::atomic_bool flag_stop = false;

void signalHandler (int signum) {
    	flag_stop = true;
}

Eigen::Vector3f skewVec (const Eigen::Matrix3f& R) {
	Eigen::Matrix3f S = 0.5*(R-R.transpose());
	Eigen::Vector3f v(S(2,1),S(0,2),S(1,0));
	return v;
}
