#include "rosneuro_integrator/Integrator.h"

namespace rosneuro {

Integrator::Integrator(void) : p_nh_("~") {

	this->has_new_data_ = false;

	this->loader_.reset(new pluginlib::ClassLoader<GenericIntegrator>("rosneuro_integrators", "rosneuro::GenericIntegrator"));

}

Integrator::~Integrator(void) {
	this->loader_.reset();
}

bool Integrator::configure(void) {

	std::string  plugin;

	// Getting mandatory parameters from ROS
	if(ros::param::get("~plugin", this->plugin_) == false) {
		ROS_ERROR("Missing 'plugin' in the server. 'plugin' is a mandatory parameter");
		return false;
	}
	
	// Dynamically load the plugin
	try {
		this->integrator_ = this->loader_->createInstance(this->plugin_);
	} catch (pluginlib::PluginlibException& ex) {
		ROS_ERROR("'%s' plugin failed to load: %s", this->plugin_.c_str(), ex.what());
	}

	this->integratorname_ = this->integrator_->name();

	if(this->integrator_->configure() == false) {
		ROS_ERROR("Cannot configure the integrator \'%s\'", this->integratorname_.c_str());
		return false;
	}
	ROS_INFO("The integrator \'%s\' has been correctly created and configured", this->integratorname_.c_str());

	// Subscriber and publisher
	this->sub_ = this->nh_.subscribe("/smr/neuroprediction", 1, &Integrator::on_received_neurodata, this);
	this->pub_ = this->p_nh_.advertise<rosneuro_msgs::NeuroOutput>("/integrated", 1);

	return true;
}

void Integrator::run(void) {

	ros::Rate r(512);

	rosneuro_msgs::NeuroOutput msg;

	while(ros::ok()) {

		if(this->has_new_data_ == true) {

			this->pub_.publish(this->neurooutput_);	
			this->has_new_data_ = false;
		}

		ros::spinOnce();
		r.sleep();

	}
}

void Integrator::on_received_neurodata(const rosneuro_msgs::NeuroOutput& msg) {

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
}

bool Integrator::on_reset_integrator(std_srvs::Empty::Request& req,
									 std_srvs::Empty::Response& res) {

	if(this->integrator_->reset() == false) {
		ROS_WARN("Integrator \'%s\' has note been reset", this->integrator_->name().c_str());
		return false;
	}
	ROS_INFO("Integrator \'%s\' has been reset", this->integrator_->name().c_str());
		
	return true;
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
