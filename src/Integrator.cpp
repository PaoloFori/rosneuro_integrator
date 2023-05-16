#include "rosneuro_integrator/Integrator.h"

namespace rosneuro {

Integrator::Integrator(void) : p_nh_("~") {

	this->has_new_data_   = false;
	this->has_thresholds_ = true;

	this->loader_.reset(new pluginlib::ClassLoader<GenericIntegrator>("rosneuro_integrator", "rosneuro::GenericIntegrator"));

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

	// Getting threshold parameters
	if(this->p_nh_.param<std::vector<float>>("thresholds", this->thresholds_, {}) == false) {
		ROS_WARN("[%s] Integrator working without thresholds", this->integrator_->name().c_str());
		this->has_thresholds_ = false;
	}

	// Getting reset event value
	this->p_nh_.param<int>("reset_event", this->reset_event_, this->reset_event_default_);
	ROS_INFO("[%s] Reset event set to: %d", this->integrator_->name().c_str(), this->reset_event_);

	// Subscriber and publisher
	this->subprd_ = this->nh_.subscribe("/smr/neuroprediction", 1, &Integrator::on_received_neurooutput, this);
	this->subevt_ = this->nh_.subscribe("/events/bus", 1, &Integrator::on_received_neuroevent, this);
	this->pubprd_ = this->p_nh_.advertise<rosneuro_msgs::NeuroOutput>("/integrated", 1);

	// Services
	this->srv_reset_ = this->p_nh_.advertiseService("reset", &Integrator::on_reset_integrator, this);

	return true;
}

void Integrator::run(void) {

	ros::Rate r(512);

	rosneuro_msgs::NeuroOutput msg;

	while(ros::ok()) {

		if(this->has_new_data_ == true) {

			this->pubprd_.publish(this->neurooutput_);	
			this->has_new_data_ = false;

		}

		ros::spinOnce();
		r.sleep();

	}
}

void Integrator::on_received_neurooutput(const rosneuro_msgs::NeuroOutput& msg) {

	Eigen::VectorXf input, output;

	input  = this->vector_to_eigen(msg.softpredict.data);
	output = this->integrator_->apply(input);

	this->neurooutput_.header.stamp = ros::Time::now();
	this->neurooutput_.softpredict.data = this->eigen_to_vector(output);
	this->neurooutput_.hardpredict.data = {};
	this->neurooutput_.class_labels = msg.class_labels;
	this->neurooutput_.decoder_type = msg.decoder_type;
	this->neurooutput_.decoder_path = msg.decoder_path;

	this->has_new_data_ = true;

	// Check if a reset is needed
	if(this->is_over_threshold(output) == true) {
		this->integrator_->reset();
		ROS_INFO("[%s] Value over thresholds: integrator has been reset", this->integrator_->name().c_str());
	}
}

bool Integrator::reset_integrator_state(void) {

	if(this->integrator_->reset() == false) {
		ROS_WARN("[%s] Integrator has not been reset", this->integrator_->name().c_str());
		return false;
	}
	ROS_INFO("[%s] Integrator has been reset", this->integrator_->name().c_str());

	return true;
}


void Integrator::on_received_neuroevent(const rosneuro_msgs::NeuroEvent& msg) {

	if(msg.event == this->reset_event_) {
		this->reset_integrator_state();
	}
}

bool Integrator::on_reset_integrator(std_srvs::Empty::Request& req,
									 std_srvs::Empty::Response& res) {

	return this->reset_integrator_state();
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

bool Integrator::is_over_threshold(const Eigen::VectorXf& values) {

	float maxvalue;
	bool retcod = false;

	if(this->has_thresholds_) {
		maxvalue = values.maxCoeff();	

		for(auto it=this->thresholds_.begin(); it!=this->thresholds_.end(); ++it) {
			if( maxvalue >= *it ) {
				retcod = true;
				break;
			}
		}
	}

	return retcod;
}



}
