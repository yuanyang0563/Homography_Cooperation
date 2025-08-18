#include "manipulator.hpp"

int main (int argc, char **argv) {

	rclcpp::init(argc, argv);
	rclcpp::Rate loop_rate(100);
	manipulator arm("gen3");
	
	// make the last twist message be zero before the node shuts down
	std::signal(SIGINT, signalHandler);
	
	while (rclcpp::ok() && !flag_stop) {
		arm.getPose();
		arm.setTwist();
		rclcpp::spin_some(arm.node);
		loop_rate.sleep();
	}
	arm.setTwist();
	rclcpp::shutdown();
	
	return 0;
	
}
