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
            this->start_cf_ = ros::Time::now();

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
            uint32_t seq = msg_artifact.seq;
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
                                    std::shared_ptr<artifacts_bci::artifact_presence> artifact) {

            std::vector<int> classes;
            uint32_t seq_num;
            if(this->paradigm_ == "cvsa"){
                classes = cvsa->decoder.classes;
                seq_num = cvsa->neuroheader.seq;
            }else if(this->paradigm_ == "mi"){
                classes = mi->decoder.classes;
                seq_num = mi->neuroheader.seq;
            }else if(this->paradigm_ == "hybrid"){
                classes = hybrid_classes;
                seq_num = mi->neuroheader.seq;
            }
            int num_classes = classes.size();
            Eigen::VectorXf output(num_classes);

            if (artifact->has_artifact) {
                output = this->integrator_->getData();
                /*
                Eigen::VectorXf neutral_prob = Eigen::VectorXf::Constant(num_classes, 1.0f / num_classes);
                output = this->integrator_->apply(neutral_prob);
                */
            }else{
                if(this->paradigm_ == "hybrid"){
                    double t = (cvsa->header.stamp - this->start_cf_).toSec();
                    if (t < 0.0) t = 0.0; 
                    double alpha = 0.0;
                    if(t <= 2.5){
                        alpha = 0.5 * (1.0 + cos(M_PI * t / 2.5));
                    }else{
                        alpha = 0.0;
                    }
                
                    std::vector<double> tempered_priors(num_classes, 0.0);
                    double sum_priors = 0.0;
                    for (int i = 0; i < num_classes; i++) {
                        tempered_priors[i] = std::pow(cvsa->softpredict.data[i], alpha);
                        sum_priors += tempered_priors[i];
                    }
                    for (int i = 0; i < num_classes; i++) {
                        tempered_priors[i] /= sum_priors;
                    }
                
                    // Merge with Bayes Theorem
                    double sum_final = 0.0;
                    for (int i = 0; i < num_classes; i++) {
                        output[i] = mi->softpredict.data[i] * tempered_priors[i];
                        sum_final += output[i];
                    }
                    for (int i = 0; i < num_classes; i++) {
                        output[i] /= (float)sum_final;
                    }
                }else if(this->paradigm_ == "cvsa"){
                    output = this->vectorToEigen(cvsa->softpredict.data);
                }else if(this->paradigm_ == "mi"){
                    output = this->vectorToEigen(mi->softpredict.data);
                }
                
                output = this->integrator_->apply(output);
            }
            
        
            this->msgoutput_.header.stamp = ros::Time::now();
            this->msgoutput_.softpredict.data = this->eigenToVector(output);
            this->msgoutput_.neuroheader.seq = seq_num;
            this->msgoutput_.decoder.classes = classes;

            this->pub_.publish(this->msgoutput_);
        }

        bool Integrator::resetIntegrator(void) {
            if(!this->integrator_->reset()) {
                ROS_WARN("[%s] Integrator has not been reset", this->integrator_->name().c_str());
                return false;
            }
            ROS_INFO("[%s] Integrator has been reset", this->integrator_->name().c_str());
            ros::spinOnce();
            std::vector<float> initial_vals = this->integrator_->getInitPrecentual(); 
            this->msgoutput_.header.stamp = ros::Time::now();
            this->msgoutput_.softpredict.data = initial_vals;
            std::cout << "Initial values after reset: ";
            for(size_t i = 0; i < initial_vals.size(); i++) {
                std::cout << initial_vals[i] << " ";
            }
            std::cout << std::endl;
            this->pub_.publish(this->msgoutput_);
            return true;
        }

        bool Integrator::onResetIntegrator(std_srvs::Empty::Request& req,
                                             std_srvs::Empty::Response& res) {
            ROS_INFO("[%s] Reset integrator service called", this->integrator_->name().c_str());
            this->start_cf_ = ros::Time::now();
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
