#ifndef ROSNEURO_INTEGRATORS_INTEGRATOR_H_
#define ROSNEURO_INTEGRATORS_INTEGRATOR_H_

#include <memory>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <pluginlib/class_loader.h>
#include <rosneuro_msgs/NeuroOutput.h>
#include <rosneuro_msgs/NeuroEvent.h>
#include "rosneuro_integrator/GenericIntegrator.h"


namespace rosneuro {
	namespace integrator {

class Integrator {
	
	public:
		Integrator(void);
		~Integrator(void);

		bool configure(void);
		void run(void);

	private:
		void on_received_data(const rosneuro_msgs::NeuroOutput& msg);
		void on_received_event(const rosneuro_msgs::NeuroEvent& msg);
		bool on_reset_integrator(std_srvs::Empty::Request& req,
							 	 std_srvs::Empty::Response& res);

		bool reset_integrator(void);

	private:
		Eigen::VectorXf vector_to_eigen(const std::vector<float>& in);
		std::vector<float> eigen_to_vector(const Eigen::VectorXf& in);
		bool is_over_threshold(const Eigen::VectorXf& values);
		void set_message(const Eigen::VectorXf& data);

	private:
		ros::NodeHandle nh_;
		ros::NodeHandle p_nh_;
		ros::Subscriber	sub_;
		ros::Publisher	pub_;
		ros::Subscriber	subevt_;
		ros::ServiceServer srv_reset_;
	
		Eigen::VectorXf input_;
		Eigen::VectorXf output_;

		rosneuro_msgs::NeuroOutput msgoutput_;

		int  reset_event_;
		const int reset_event_default_ = 781;
		bool has_new_data_;
		bool is_first_message_;
		
		std::string plugin_;
		std::string integratorname_;

		boost::shared_ptr<GenericIntegrator> 	integrator_;

		std::unique_ptr<pluginlib::ClassLoader<GenericIntegrator>> loader_;



};

	}
}

#endif
