#include "rosneuro_integrator/Integrator.h"

namespace rosneuro {
	namespace integrator {

Integrator::Integrator(void) : p_nh_("~") {

	this->has_new_data_     = false;
	this->is_first_message_ = true;

	this->loader_.reset(new pluginlib::ClassLoader<GenericIntegrator>("rosneuro_integrator", "rosneuro::integrator::GenericIntegrator"));

}

Integrator::~Integrator(void) {
	boost::shared_ptr<GenericIntegrator>().swap(this->integrator_);
	this->loader_.reset();
}

bool Integrator::configure(void) {

	std::string  plugin;

	// Getting mandatory parameters from ROS
	if(ros::param::get("~plugin", this->plugin_) == false) {
		ROS_ERROR("[integrator] Missing 'plugin' in the server. 'plugin' is a mandatory parameter");
		return false;
	}
	
	// Dynamically load the plugin
	try {
		this->integrator_ = this->loader_->createInstance(this->plugin_);
	} catch (pluginlib::PluginlibException& ex) {
		ROS_ERROR("[integrator] '%s' plugin failed to load: %s", this->plugin_.c_str(), ex.what());
	}

	this->integratorname_ = this->integrator_->name();

	if(this->integrator_->configure() == false) {
		ROS_ERROR("[%s] Cannot configure the integrator", this->integratorname_.c_str());
		return false;
	}
	ROS_INFO("[%s] Integrator correctly created and configured", this->integratorname_.c_str());

	// Getting reset event value
	this->p_nh_.param<int>("reset_event", this->reset_event_, this->reset_event_default_);
	ROS_INFO("[%s] Reset event set to: %d", this->integrator_->name().c_str(), this->reset_event_);

	// Subscribers and publishers
	this->sub_ = this->nh_.subscribe("/smr/neuroprediction", 1, &Integrator::on_received_data, this);
	this->pub_ = this->p_nh_.advertise<rosneuro_msgs::NeuroOutput>("/integrated", 1);
	this->subevt_ = this->nh_.subscribe("/events/bus", 1, &Integrator::on_received_event, this);

	// Services
	this->srv_reset_ = this->p_nh_.advertiseService("reset", &Integrator::on_reset_integrator, this);

	return true;
}

void Integrator::run(void) {

	ros::Rate r(512);

	rosneuro_msgs::NeuroOutput msg;

	while(ros::ok()) {

		if(this->has_new_data_ == true) {

			this->set_message(this->output_);
			this->pub_.publish(this->msgoutput_);
			this->has_new_data_ = false;

		}

		ros::spinOnce();
		r.sleep();

	}
}

void Integrator::on_received_data(const rosneuro_msgs::NeuroOutput& msg) {
	this->input_  = this->vector_to_eigen(msg.softpredict.data);
	this->output_ = this->integrator_->apply(this->input_);
	this->has_new_data_ = true;
	
	if(this->is_first_message_ == true) {
		this->msgoutput_.hardpredict.data = msg.hardpredict.data;
		this->msgoutput_.class_labels 	  = msg.class_labels;
		this->msgoutput_.decoder_type 	  = msg.decoder_type;
		this->msgoutput_.decoder_path 	  = msg.decoder_path;
		this->is_first_message_ = false;
	}
}

void Integrator::set_message(const Eigen::VectorXf& data) {
	this->msgoutput_.header.stamp = ros::Time::now();
	this->msgoutput_.softpredict.data = this->eigen_to_vector(data);
}

bool Integrator::reset_integrator(void) {

	if(this->integrator_->reset() == false) {
		ROS_WARN("[%s] Integrator has not been reset", this->integrator_->name().c_str());
		return false;
	}
	ROS_INFO("[%s] Integrator has been reset", this->integrator_->name().c_str());

	// Consuming old messages
	ros::spinOnce();
	
	// Publish current reset output
	this->output_ = Eigen::VectorXf::Constant(this->output_.rows(), 
											  this->output_.cols(), 
											  1.0f/this->output_.size());
	this->set_message(this->output_);
	this->pub_.publish(this->msgoutput_);
	this->has_new_data_ = false;

	return true;
}


void Integrator::on_received_event(const rosneuro_msgs::NeuroEvent& msg) {

	if(msg.event == this->reset_event_) {
		this->reset_integrator();
	}
}

bool Integrator::on_reset_integrator(std_srvs::Empty::Request& req,
									 std_srvs::Empty::Response& res) {

	return this->reset_integrator();
}

Eigen::VectorXf Integrator::vector_to_eigen(const std::vector<float>& in) {

	float* ptr_in;

	ptr_in = const_cast<float*>(in.data());

	Eigen::VectorXf out = Eigen::Map<Eigen::VectorXf>(ptr_in, in.size());

	return out;
}

std::vector<float> Integrator::eigen_to_vector(const Eigen::VectorXf& in) {

	std::vector<float> out(in.size());

	Eigen::Map<Eigen::VectorXf>(out.data(), in.size()) = in;

	return out;

}

}

}
