#include "manipulator.hpp"

int main (int argc, char **argv) {

	rclcpp::init(argc, argv);
	rclcpp::Rate loop_rate(100);
	manipulator arm("gen3");

	if (argc<2) {
		std::cout << "please set control mode: home || ibvs || pbvs || hbvs" << std::endl;
		return 1;
	} else {
		mode = argv[1];
		if (mode!="home" && mode!="ibvs" && mode!="pbvs" && mode!="hbvs")
			return 1;
	}
	
	// make the last twist message be zero before the node shuts down
	std::signal(SIGINT, signalHandler);
	
	while (rclcpp::ok() && !flag_stop) {
		arm.getPose();
		arm.getFeatures();
		arm.setTwist();
		rclcpp::spin_some(arm.node);
		loop_rate.sleep();
	}
	arm.setTwist();
	rclcpp::shutdown();
	
	return 0;
	
}
