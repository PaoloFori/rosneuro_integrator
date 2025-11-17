#include "rosneuro_integrator/Integrator.h"

namespace rosneuro {
	namespace integrator {
        Integrator::Integrator(void) : p_nh_("~") {
            this->loader_.reset(new pluginlib::ClassLoader<GenericIntegrator>("rosneuro_integrator", "rosneuro::integrator::GenericIntegrator"));
            prune_timer_ = nh_.createTimer(ros::Duration(0.5), &Integrator::pruneBuffer, this);
            this->max_age_ = ros::Duration(2.0); 
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

            this->p_nh_.param<int>("reset_event", this->reset_event_, this->reset_event_default_);
            ROS_INFO("[%s] Reset event set to: %d", this->integrator_->name().c_str(), this->reset_event_);

            this->p_nh_.param<int>("ic_class", this->ic_class_, this->ic_class_default_);
            ROS_INFO("[%s] ic_class set to: %d", this->integrator_->name().c_str(), this->ic_class_);

            this->p_nh_.param<float>("ic_threshold", this->ic_threshold_, 0.7);
            ROS_INFO("[%s] ic_threshold set to: %f", this->integrator_->name().c_str(), this->ic_threshold_);

            this->subscribeAdvertiseServices();

            ROS_INFO("[%s] Integrator correctly created and configured", this->integrator_name_.c_str());

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
            this->sub_icnic_ = this->nh_.subscribe("/cvsa/neuroprediction/icnic", 1, &Integrator::onReceivedData_icnic, this);
            this->sub_classifier_ = this->nh_.subscribe("/cvsa/neuroprediction/raw", 1, &Integrator::onReceivedData_classifier, this);
            this->sub_artifacts_ = this->nh_.subscribe("/cvsa/artifact_presence", 1, &Integrator::onReceivedData_artifacts, this);
            this->sub_event_ = this->nh_.subscribe("/events/bus", 1, &Integrator::onReceivedEvent, this);

            this->pub_ = this->nh_.advertise<rosneuro_msgs::NeuroOutput>("/cvsa/neuroprediction/integrated", 1);
            this->srv_reset_ = this->p_nh_.advertiseService("reset", &Integrator::onResetIntegrator, this);
        }

        void Integrator::run(void) {
            ros::Rate r(512);
            rosneuro_msgs::NeuroOutput msg;
            while(ros::ok()) {
                ros::spinOnce();
                r.sleep();
            }
        }

        void Integrator::onReceivedData_icnic(const rosneuro_msgs::NeuroOutput& msg_icnic) {
            uint32_t seq = msg_icnic.neuroheader.seq;
            ros::Time now = ros::Time::now();
        
            MessageSet set_to_process;
            bool set_is_complete = false;

            {
                std::lock_guard<std::mutex> lock(this->buffer_mutex_);
                MessageSet& entry = this->buffer_[seq];
                if (!entry.msg_icnic && !entry.msg_classifier && !entry.msg_artifact) {
                    entry.timestamp = now;
                }
                entry.msg_icnic = std::make_shared<rosneuro_msgs::NeuroOutput>(msg_icnic);

                if (entry.msg_icnic && entry.msg_classifier && entry.msg_artifact){
                    set_is_complete = true;
                    set_to_process = entry;
                    buffer_.erase(seq);
                }
            } 

            if (set_is_complete){
                this->integrateSyncData(*set_to_process.msg_icnic, 
                                        *set_to_process.msg_classifier, 
                                        *set_to_process.msg_artifact);
            }
        }

        void Integrator::onReceivedData_artifacts(const artifacts_cvsa::artifact_presence& msg_artifact) {
            uint32_t seq = msg_artifact.seq;
            ros::Time now = ros::Time::now();
        
            MessageSet set_to_process;
            bool set_is_complete = false;

            {
                std::lock_guard<std::mutex> lock(this->buffer_mutex_);
                MessageSet& entry = this->buffer_[seq];
                if (!entry.msg_icnic && !entry.msg_classifier && !entry.msg_artifact) {
                    entry.timestamp = now;
                }
                entry.msg_artifact = std::make_shared<artifacts_cvsa::artifact_presence>(msg_artifact);

                if (entry.msg_icnic && entry.msg_classifier && entry.msg_artifact){
                    set_is_complete = true;
                    set_to_process = entry;
                    buffer_.erase(seq);
                }
            } 

            if (set_is_complete){
                this->integrateSyncData(*set_to_process.msg_icnic, 
                                        *set_to_process.msg_classifier, 
                                        *set_to_process.msg_artifact);
            }
        }

        void Integrator::onReceivedData_classifier(const rosneuro_msgs::NeuroOutput& msg_classifier) {
            uint32_t seq = msg_classifier.neuroheader.seq;
            ros::Time now = ros::Time::now();
        
            MessageSet set_to_process;
            bool set_is_complete = false;

            {
                std::lock_guard<std::mutex> lock(this->buffer_mutex_);
                MessageSet& entry = this->buffer_[seq];
                if (!entry.msg_icnic && !entry.msg_classifier && !entry.msg_artifact) {
                    entry.timestamp = now;
                }
                entry.msg_classifier = std::make_shared<rosneuro_msgs::NeuroOutput>(msg_classifier);

                if (entry.msg_icnic && entry.msg_classifier && entry.msg_artifact){
                    set_is_complete = true;
                    set_to_process = entry;
                    buffer_.erase(seq);
                }
            } 

            if (set_is_complete){
                this->integrateSyncData(*set_to_process.msg_icnic, 
                                        *set_to_process.msg_classifier, 
                                        *set_to_process.msg_artifact);
            }
        }

        void Integrator::integrateSyncData(const rosneuro_msgs::NeuroOutput& msg_icnic, 
                               const rosneuro_msgs::NeuroOutput& msg_classifier, 
                               const artifacts_cvsa::artifact_presence& msg_artifact){
            uint32_t seq_num = msg_icnic.neuroheader.seq; 
            ROS_INFO("--- SET SINCRONIZZATO RICEVUTO (Seq: %u) ---", seq_num);

            // find the index of the ic_class_ in the icnic message
            int ic_index;
            auto it = std::find(msg_icnic.decoder.classes.begin(), 
                    msg_icnic.decoder.classes.end(), 
                    this->ic_class_);

            if (it != msg_icnic.decoder.classes.end()){
                ic_index = static_cast<int>(std::distance(msg_icnic.decoder.classes.begin(), it));
            }else{
                ROS_ERROR("ic_class %d not found in icnic classes", this->ic_class_);
                return;
            }

            // check if the classifier probability must be integrated or not
            Eigen::VectorXf icnic_data  = this->vectorToEigen(msg_icnic.softpredict.data);
            Eigen::VectorXf output;
            if(!msg_artifact.has_artifact && icnic_data[ic_index] >= this->ic_threshold_){
                // no EOG, artifact and in IC state
                output = this->integrator_->apply(this->vectorToEigen(msg_classifier.softpredict.data));
            }else{
                // in NIC
                output = this->integrator_->getData();
            }

            this->setMessage(output);        
        }

        void Integrator::pruneBuffer(const ros::TimerEvent& event){
            ros::Time now = ros::Time::now();
            std::lock_guard<std::mutex> lock(this->buffer_mutex_);

            for (auto it = this->buffer_.begin(); it != this->buffer_.end(); /* nothing here */){
                if ((now - it->second.timestamp) > this->max_age_){
                    ROS_WARN("Removed seq %u from the buffer (timeout).", it->first);
                    it = this->buffer_.erase(it);
                }
                else{
                    ++it;
                }
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
            std::vector<float> initial_vals = this->integrator_->getInitPrecentual(); 
            this->setMessage(this->vectorToEigen(initial_vals));
            this->pub_.publish(this->msgoutput_);
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
