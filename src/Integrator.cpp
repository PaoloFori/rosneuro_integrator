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
            if(!ros::param::get("~plugin", this->plugin_)) {
                ROS_ERROR("[integrator] Missing 'plugin' in the server. 'plugin' is a mandatory parameter");
                return false;
            }

            if(!this->loadPlugin()) return false;

            this->integrator_name_ = this->integrator_->name();

            if(!this->integrator_->configure()) {
                ROS_ERROR("[%s] Cannot configure the integrator", this->integrator_name_.c_str());
                return false;
            }

            ROS_INFO("[%s] Integrator correctly created and configured", this->integrator_name_.c_str());

            this->p_nh_.param<int>("reset_event", this->reset_event_, this->reset_event_default_);
            ROS_INFO("[%s] Reset event set to: %d", this->integrator_->name().c_str(), this->reset_event_);

            this->subscribeAdvertiseServices();

            return true;
        }

        bool Integrator::loadPlugin(void) {
            try {
                this->integrator_ = this->setIntegrator();
            } catch (pluginlib::PluginlibException& ex) {
                ROS_ERROR("[integrator] '%s' plugin failed to load: %s", this->plugin_.c_str(), ex.what());
                std::cout << ex.what() << std::endl;
                return false;
            }
            return true;
        }

        boost::shared_ptr<GenericIntegrator> Integrator::setIntegrator(void) {
            return this->loader_->createInstance(this->plugin_);
        }

        void Integrator::subscribeAdvertiseServices(void){
            this->sub_ = this->nh_.subscribe("/cvsa/neuroprediction", 1, &Integrator::onReceivedData, this);
            this->pub_ = this->nh_.advertise<rosneuro_msgs::NeuroOutput>("/cvsa/neuroprediction/integrated", 1);
            this->sub_event_ = this->nh_.subscribe("/events/bus", 1, &Integrator::onReceivedEvent, this);
            this->srv_reset_ = this->p_nh_.advertiseService("reset", &Integrator::onResetIntegrator, this);
        }

        void Integrator::run(void) {
            ros::Rate r(512);
            rosneuro_msgs::NeuroOutput msg;
            while(ros::ok()) {
                if(this->has_new_data_) {
                    this->setMessage(this->output_);
                    this->pub_.publish(this->msgoutput_);
                    this->has_new_data_ = false;
                }
                ros::spinOnce();
                r.sleep();
            }
        }

        void Integrator::onReceivedData(const rosneuro_msgs::NeuroOutput& msg) {
            this->input_  = this->vectorToEigen(msg.softpredict.data);
            this->output_ = this->integrator_->apply(this->input_);
            this->has_new_data_ = true;
            this->msgoutput_.neuroheader = msg.neuroheader;

            if(this->is_first_message_) {
                this->msgoutput_.hardpredict.data = msg.hardpredict.data;
                this->msgoutput_.decoder.classes  = msg.decoder.classes;
                this->msgoutput_.decoder.type 	  = msg.decoder.type;
                this->msgoutput_.decoder.path 	  = msg.decoder.path;
                this->is_first_message_ = false;
            }
        }

        void Integrator::setMessage(const Eigen::VectorXf& data) {
            this->msgoutput_.header.stamp = ros::Time::now();
            this->msgoutput_.softpredict.data = this->eigenToVector(data);
        }

        bool Integrator::resetIntegrator(void) {
            if(!this->integrator_->reset()) {
                ROS_WARN("[%s] Integrator has not been reset", this->integrator_->name().c_str());
                return false;
            }
            ROS_INFO("[%s] Integrator has been reset", this->integrator_->name().c_str());
            ros::spinOnce();
            std::vector<float> initial_vals = this->integrator_->getInitPrecentual(); ///////////
            this->output_ = this->vectorToEigen(initial_vals);
            this->setMessage(this->output_);
            this->pub_.publish(this->msgoutput_);
            this->has_new_data_ = false;
            return true;
        }

        void Integrator::onReceivedEvent(const rosneuro_msgs::NeuroEvent& msg) {
            if(msg.event == this->reset_event_) {
                this->resetIntegrator();
            }
        }

        bool Integrator::onResetIntegrator(std_srvs::Empty::Request& req,
                                             std_srvs::Empty::Response& res) {
            return this->resetIntegrator();
        }

        Eigen::VectorXf Integrator::vectorToEigen(const std::vector<float>& in) {
            float* ptr_in = const_cast<float*>(in.data());
            return Eigen::Map<Eigen::VectorXf>(ptr_in, in.size());
        }

        std::vector<float> Integrator::eigenToVector(const Eigen::VectorXf& in) {
            std::vector<float> out(in.size());
            Eigen::Map<Eigen::VectorXf>(out.data(), in.size()) = in;
            return out;
        }
    }
}
