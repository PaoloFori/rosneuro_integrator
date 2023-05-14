#ifndef ROSNEURO_INTEGRATORS_INTEGRATOR_H_
#define ROSNEURO_INTEGRATORS_INTEGRATOR_H_

#include <memory>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <pluginlib/class_loader.h>
#include <rosneuro_msgs/NeuroOutput.h>
#include "rosneuro_integrator/GenericIntegrator.h"


namespace rosneuro {

class Integrator {
	
	public:
		Integrator(void);
		~Integrator(void);

		bool configure(void);
		void run(void);

	private:
		void on_received_neurodata(const rosneuro_msgs::NeuroOutput& msg);
		bool on_reset_integrator(std_srvs::Empty::Request& req,
							 	 std_srvs::Empty::Response& res);



	private:
		Eigen::VectorXf vector_to_eigen(const std::vector<float>& in);
		std::vector<float> eigen_to_vector(const Eigen::VectorXf& in);

	private:
		ros::NodeHandle nh_;
		ros::NodeHandle p_nh_;
		ros::Subscriber	sub_;
		ros::Publisher	pub_;
		ros::ServiceServer srv_reset_;
		
		rosneuro_msgs::NeuroOutput neurooutput_;
		bool has_new_data_;
		
		std::string plugin_;
		std::string integratorname_;

		boost::shared_ptr<GenericIntegrator> 	integrator_;

		std::unique_ptr<pluginlib::ClassLoader<GenericIntegrator>> loader_;



};


}

#endif
