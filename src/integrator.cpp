#include <ros/ros.h>
#include "rosneuro_integrator/Integrator.h"

int main(int argc, char** argv) {

	ros::init(argc, argv, "integrator");

	rosneuro::integrator::Integrator integrator;

	if(integrator.configure() == false) {
		ROS_ERROR("[integrator] Configuration failed");
		return -1;
	}

	integrator.run();

	ros::shutdown();

	return 0;
}
