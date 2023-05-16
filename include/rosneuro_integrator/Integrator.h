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

class Integrator {
	
	public:
		Integrator(void);
		~Integrator(void);

		bool configure(void);
		void run(void);

	private:
		void on_received_neurooutput(const rosneuro_msgs::NeuroOutput& msg);
		void on_received_neuroevent(const rosneuro_msgs::NeuroEvent& msg);
		bool on_reset_integrator(std_srvs::Empty::Request& req,
							 	 std_srvs::Empty::Response& res);

		bool reset_integrator_state(void);

	private:
		Eigen::VectorXf vector_to_eigen(const std::vector<float>& in);
		std::vector<float> eigen_to_vector(const Eigen::VectorXf& in);
		bool is_over_threshold(const Eigen::VectorXf& values);

	private:
		ros::NodeHandle nh_;
		ros::NodeHandle p_nh_;
		ros::Subscriber	subprd_;
		ros::Subscriber	subevt_;
		ros::Publisher	pubprd_;
		ros::ServiceServer srv_reset_;
		
		rosneuro_msgs::NeuroOutput neurooutput_;
		std::vector<float> thresholds_;

		int  reset_event_;
		const int reset_event_default_ = 781;
		bool has_new_data_;
		bool has_thresholds_;
		
		std::string plugin_;
		std::string integratorname_;

		boost::shared_ptr<GenericIntegrator> 	integrator_;

		std::unique_ptr<pluginlib::ClassLoader<GenericIntegrator>> loader_;



};


}

#endif
