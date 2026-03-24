#include "rosneuro_integrator/Integrator.h"

namespace rosneuro {
	namespace integrator {
        Integrator::Integrator(void) : p_nh_("~") {
            this->loader_.reset(new pluginlib::ClassLoader<GenericIntegrator>("rosneuro_integrator", "rosneuro::integrator::GenericIntegrator"));
            this->prune_timer_ = this->nh_.createTimer(ros::Duration(0.5), &Integrator::pruneBuffer, this);
        }

        Integrator::~Integrator(void) {
            boost::shared_ptr<GenericIntegrator>().swap(this->integrator_);
            this->loader_.reset();
        }

        bool Integrator::configure(void) {
            // load the plugin
            if(!ros::param::get("~plugin", this->plugin_)) {
                ROS_ERROR("[integrator] Missing 'plugin' in the server. 'plugin' is a mandatory parameter");
                return false;
            }
            if(!this->loadPlugin()) return false;

            // configure the plugin
            this->integrator_name_ = this->integrator_->name();

            if(!this->integrator_->configure()) {
                ROS_ERROR("[%s] Cannot configure the integrator", this->integrator_name_.c_str());
                return false;
            }

            // configure sincronization parameters
            this->max_age_ = ros::Duration(1.0);

            // paradigm organization
            if(this->p_nh_.getParam("paradigm", this->paradigm_) == false) {
                ROS_ERROR("[%s] Parameter 'paradigm' is mandatory", this->integrator_name_.c_str());
                return false;
            }

            if(this->paradigm_ == "hybrid"){
                // cvsa, mi, artifacts
                this->sub_cvsa_ = this->nh_.subscribe("/cvsa/neuroprediction/raw", 1, &Integrator::onReceivedData_cvsa, this);
                this->sub_mi_ = this->nh_.subscribe("/mi/neuroprediction/raw", 1, &Integrator::onReceivedData_mi, this);
            }else{
                // cvsa/mi and artifacts
                if(this->paradigm_ == "cvsa"){
                    this->sub_cvsa_ = this->nh_.subscribe("/cvsa/neuroprediction/raw", 1, &Integrator::onReceivedData_cvsa, this);
                }else if(this->paradigm_ == "mi"){
                    this->sub_mi_ = this->nh_.subscribe("/mi/neuroprediction/raw", 1, &Integrator::onReceivedData_mi, this);
                }else{
                    ROS_ERROR("[%s] Unknown paradigm provided", this->integrator_name_.c_str());
                    return false;
                }
            }
            
            std::string topic_pub = "/" + this->paradigm_ + "/neuroprediction/integrated/raw";
            this->pub_ = this->nh_.advertise<rosneuro_msgs::NeuroOutput>(topic_pub, 1);

            this->sub_artifacts_ = this->nh_.subscribe("/artifact_presence", 1, &Integrator::onReceivedData_artifacts, this);

            this->srv_reset_ = this->nh_.advertiseService("/integrator/reset", &Integrator::onResetIntegrator, this);


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

        void Integrator::pruneBuffer(const ros::TimerEvent& event){
            ros::Time now = ros::Time::now();
            std::lock_guard<std::mutex> lock(this->mutex_);

            for (auto it = this->sync_set_.begin(); it != this->sync_set_.end(); /* nothing here */){
                if ((now - it->second.timestamp) > this->max_age_){
                    ROS_WARN("[%s] Removed seq %u from the buffer (timeout).", this->integrator_->name().c_str(), it->first);
                    it = this->sync_set_.erase(it);
                }else{
                    ++it;
                }
            }
        }

        void Integrator::run(void) {
            ros::Rate r(512);
            while(ros::ok()) {
                ros::spinOnce();
                r.sleep();
            }
        }

        void Integrator::onReceivedData_mi(const rosneuro_msgs::NeuroOutput& msg_mi) {
            uint32_t seq = msg_mi.neuroheader.seq;
            ros::Time now = ros::Time::now();
        
            Sync_Set set_to_process;
            bool set_is_complete = false;

            {
                std::lock_guard<std::mutex> lock(this->mutex_);
                Sync_Set& entry = this->sync_set_[seq];
                if (!entry.msg_mi && !entry.msg_cvsa && !entry.msg_artifact) {
                    entry.timestamp = now;
                }
                entry.msg_mi = std::make_shared<rosneuro_msgs::NeuroOutput>(msg_mi);

                if(this->paradigm_ == "mi"){
                    if (entry.msg_mi && entry.msg_artifact){
                        set_is_complete = true;
                        set_to_process = entry;
                        sync_set_.erase(seq);
                    }
                }else if(this->paradigm_ == "hybrid"){
                    if (entry.msg_mi && entry.msg_cvsa && entry.msg_artifact){
                        set_is_complete = true;
                        set_to_process = entry;
                        sync_set_.erase(seq);
                    }
                }
            } 

            if (set_is_complete){
                this->integrateSyncData(set_to_process.msg_cvsa, 
                                        set_to_process.msg_mi, 
                                        set_to_process.msg_artifact);
            }
        }

        void Integrator::onReceivedData_cvsa(const rosneuro_msgs::NeuroOutput& msg_cvsa) {
            uint32_t seq = msg_cvsa.neuroheader.seq;
            ros::Time now = ros::Time::now();
        
            Sync_Set set_to_process;
            bool set_is_complete = false;

            {
                std::lock_guard<std::mutex> lock(this->mutex_);
                Sync_Set& entry = this->sync_set_[seq];
                if (!entry.msg_mi && !entry.msg_cvsa && !entry.msg_artifact) {
                    entry.timestamp = now;
                }
                entry.msg_cvsa = std::make_shared<rosneuro_msgs::NeuroOutput>(msg_cvsa);

                if(this->paradigm_ == "cvsa"){
                    if (entry.msg_cvsa && entry.msg_artifact){
                        set_is_complete = true;
                        set_to_process = entry;
                        sync_set_.erase(seq);
                    }
                }else if(this->paradigm_ == "hybrid"){
                    if (entry.msg_mi && entry.msg_cvsa && entry.msg_artifact){
                        set_is_complete = true;
                        set_to_process = entry;
                        sync_set_.erase(seq);
                    }
                }
            } 

            if (set_is_complete){
                this->integrateSyncData(set_to_process.msg_cvsa, 
                                        set_to_process.msg_mi, 
                                        set_to_process.msg_artifact);
            }
        }

        void Integrator::onReceivedData_artifacts(const artifacts_bci::artifact_presence& msg_artifact) {
            uint32_t seq = msg_artifact.neuroheader.seq;
            ros::Time now = ros::Time::now();
        
            Sync_Set set_to_process;
            bool set_is_complete = false;

            {
                std::lock_guard<std::mutex> lock(this->mutex_);
                Sync_Set& entry = this->sync_set_[seq];
                if (!entry.msg_mi && !entry.msg_cvsa && !entry.msg_artifact) {
                    entry.timestamp = now;
                }
                entry.msg_artifact = std::make_shared<artifacts_bci::artifact_presence>(msg_artifact);

                if(this->paradigm_ == "cvsa"){
                    if (entry.msg_cvsa && entry.msg_artifact){
                        set_is_complete = true;
                        set_to_process = entry;
                        sync_set_.erase(seq);
                    }
                }else if(this->paradigm_ == "hybrid"){
                    if (entry.msg_mi && entry.msg_cvsa && entry.msg_artifact){
                        set_is_complete = true;
                        set_to_process = entry;
                        sync_set_.erase(seq);
                    }
                }else if(this->paradigm_ == "mi"){
                    if (entry.msg_mi && entry.msg_artifact){
                        set_is_complete = true;
                        set_to_process = entry;
                        sync_set_.erase(seq);
                    }
                }
            } 

            if (set_is_complete){
                this->integrateSyncData(set_to_process.msg_cvsa, 
                                        set_to_process.msg_mi, 
                                        set_to_process.msg_artifact);
            }
        }

        void Integrator::integrateSyncData( std::shared_ptr<rosneuro_msgs::NeuroOutput> cvsa,
                                            std::shared_ptr<rosneuro_msgs::NeuroOutput> mi,
                                            std::shared_ptr<artifacts_bci::artifact_presence> artifact){
            uint32_t seq_num = msg_icnic.neuroheader.seq; 

            // find the index of the ic_class_label_ in the icnic message
            int ic_index;
            auto it = std::find(msg_icnic.decoder.classes.begin(), 
                    msg_icnic.decoder.classes.end(), 
                    this->ic_class_label_);

            if (it != msg_icnic.decoder.classes.end()){
                ic_index = static_cast<int>(std::distance(msg_icnic.decoder.classes.begin(), it));
            }else{
                ROS_ERROR("[%s] ic_class_label %d not found in icnic classes", this->integrator_->name().c_str(), this->ic_class_label_);
                return;
            }

            // check if the classifier probability must be integrated or not
            Eigen::VectorXf icnic_data  = this->vectorToEigen(msg_icnic.softpredict.data);
            Eigen::VectorXf output;
            if(!msg_artifact.has_artifact){
                // no EOG, artifact and in IC state
                std::vector<float> merged_prob = msg_classifier.softpredict.data;
                for(int i = 0; i < merged_prob.size(); i++){
                    merged_prob[i] = (1.0 - icnic_data[ic_index])*0.5 + icnic_data[ic_index]*msg_classifier.softpredict.data[i];
                }
                output = this->integrator_->apply(this->vectorToEigen(merged_prob));
            }else{
                // in NIC
                output = this->integrator_->getData();
            }

            this->setMessage(output);
            this->msgoutput_.neuroheader.seq = seq_num;
            this->msgoutput_.decoder.classes = msg_classifier.decoder.classes;
            this->pub_.publish(this->msgoutput_);
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

        bool Integrator::onResetIntegrator(std_srvs::Empty::Request& req,
                                             std_srvs::Empty::Response& res) {
            ROS_INFO("[%s] Reset integrator service called", this->integrator_->name().c_str());
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
