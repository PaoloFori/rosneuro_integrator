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

            this->p_nh_.param<int>("reset_event", this->reset_event_, this->reset_event_default_);
            ROS_INFO("[%s] Reset event set to: %d", this->integrator_name_.c_str(), this->reset_event_);

            // thresholds
            if(this->p_nh_.getParam("classes", this->classes_) == false) {
                ROS_ERROR("[%s] Parameter 'classes' is mandatory", this->integrator_name_.c_str());
                return false;
            }
            if(this->p_nh_.getParam("thresholds", this->thresholds_) == false) {
                ROS_ERROR("[%s] Parameter 'thresholds' is mandatory for evaluation modality", this->integrator_name_.c_str());
                return false;
            }else if(this->thresholds_.size() != this->classes_.size()) {
                ROS_ERROR("[%s] Number of thresholds must match the number of classes", this->integrator_name_.c_str());
                return false;
            }

            // paradigm organization
            if(this->p_nh_.getParam("paradigm", this->paradigm_) == false) {
                ROS_ERROR("[%s] Parameter 'paradigm' is mandatory", this->integrator_name_.c_str());
                return false;
            }

            if(this->paradigm_ == "hybrid"){
                // cvsa, mi
                this->p_nh_.param<float>("cvsa_influence", this->cvsa_influence_, this->cvsa_influence_default_);
                ROS_INFO("[%s] cvsa influence is set to %f seconds", this->integrator_name_.c_str(), this->cvsa_influence_);
                this->sub_cvsa_ = this->nh_.subscribe("/cvsa/neuroprediction/raw", 1, &Integrator::onReceivedData_cvsa, this);
                this->sub_mi_ = this->nh_.subscribe("/mi/neuroprediction/raw", 1, &Integrator::onReceivedData_mi, this);
            }else{
                // cvsa/mi
                if(this->paradigm_ == "cvsa"){
                    this->sub_cvsa_ = this->nh_.subscribe("/cvsa/neuroprediction/raw", 1, &Integrator::onReceivedData_cvsa, this);
                }else if(this->paradigm_ == "mi"){
                    this->sub_mi_ = this->nh_.subscribe("/mi/neuroprediction/raw", 1, &Integrator::onReceivedData_mi, this);
                }else{
                    ROS_ERROR("[%s] Unknown paradigm provided", this->integrator_name_.c_str());
                    return false;
                }
            }
            
            std::string topic_pub_raw = "/" + this->paradigm_ + "/neuroprediction/integrated/raw";
            this->pub_raw_ = this->nh_.advertise<rosneuro_msgs::NeuroOutput>(topic_pub_raw, 1);
            std::string topic_pub_normalized = "/" + this->paradigm_ + "/neuroprediction/integrated/normalized";
            this->pub_normalized_ = this->nh_.advertise<rosneuro_msgs::NeuroOutput>(topic_pub_normalized, 1);

            this->sub_artifacts_ = this->nh_.subscribe("/artifact_presence", 1, &Integrator::onReceivedData_artifacts, this);

            this->sub_events_ = this->nh_.subscribe("/events/bus", 1, &Integrator::onReceivedEvent, this);

            ROS_INFO("[%s] Integrator correctly created and configured", this->integrator_name_.c_str());

            return true;
        }

        std::vector<float> Integrator::normalize_input(const std::vector<float>& input) {
            float p_rest = 1.0f / (float)this->classes_.size();
            std::vector<float> normalized_output(input.size(), p_rest);

            for (size_t i = 0; i < input.size(); ++i) {
                if (this->thresholds_[i] > p_rest) { 
                
                    float slope = (1.0f - p_rest) / (this->thresholds_[i] - p_rest);

                    float mapped_val = p_rest + (input[i] - p_rest) * slope;
                    normalized_output[i] = std::max(0.0f, std::min(1.0f, mapped_val));

                } else {
                    normalized_output[i] = input[i]; 
                }
            }
        
            return normalized_output;
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

            uint32_t seq_num;
            if(this->paradigm_ == "cvsa"){
                seq_num = cvsa->neuroheader.seq;
            }else if(this->paradigm_ == "mi"){
                seq_num = mi->neuroheader.seq;
            }else if(this->paradigm_ == "hybrid"){
                seq_num = mi->neuroheader.seq;
            }
            int num_classes = this->classes_.size();
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
                    if(t <= this->cvsa_influence_){
                        alpha = 0.5 * (1.0 + cos(M_PI * t / this->cvsa_influence_));
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
                
                    // Merge with Bayes Theorem (LOP)
                    double sum_final = 0.0;
                    for (int i = 0; i < num_classes; i++) {
                        output[i] = mi->softpredict.data[i] * tempered_priors[i];
                        sum_final += output[i];
                    }
                    for (int i = 0; i < num_classes; i++) {
                        output[i] /= (float)sum_final;
                    }

                    // Agreement gate: pull fused output toward uniform when MI and CVSA disagree.
                    // agree_raw = dot(CVSA, MI); agree_w in [0,1] (0=disagree, 1=agree).
                    // neutral_weight = (1 - agree_w) * alpha  →  max at t=0, zero at t>=2.5 s.
                    // output = (1-neutral_weight)*lop + neutral_weight*(1/n)  (stays normalised).
                    double agree_raw = 0.0;
                    for (int i = 0; i < num_classes; i++) {
                        agree_raw += cvsa->softpredict.data[i] * mi->softpredict.data[i];
                    }
                    double uniform_p      = 1.0 / num_classes;
                    double agree_w        = std::max(0.0, (agree_raw - uniform_p) / (1.0 - uniform_p));
                    double neutral_weight = (1.0 - agree_w) * alpha;
                    for (int i = 0; i < num_classes; i++) {
                        output[i] = (float)((1.0 - neutral_weight) * output[i] + neutral_weight * uniform_p);
                    }
                }else if(this->paradigm_ == "cvsa"){
                    output = this->vectorToEigen(cvsa->softpredict.data);
                }else if(this->paradigm_ == "mi"){
                    output = this->vectorToEigen(mi->softpredict.data);
                }
                
                output = this->integrator_->apply(output);
            }
            
            std::vector<float> raw_output = this->eigenToVector(output);
        
            this->msgoutput_.header.stamp = ros::Time::now();
            this->msgoutput_.softpredict.data = raw_output;
            this->msgoutput_.neuroheader.seq = seq_num;
            this->msgoutput_.decoder.classes = this->classes_;

            this->pub_raw_.publish(this->msgoutput_);

            std::vector<float> normalized_output = this->normalize_input(raw_output);
            this->msgoutput_.softpredict.data = normalized_output;
            this->pub_normalized_.publish(this->msgoutput_);
        }

        bool Integrator::resetIntegrator(void) {
            if(!this->integrator_->reset()) {
                ROS_WARN("[%s] Integrator has not been reset", this->integrator_->name().c_str());
                return false;
            }
            ROS_INFO("[%s] Integrator has been reset", this->integrator_->name().c_str());
            this->start_cf_ = ros::Time::now();
            ros::spinOnce();
            std::vector<float> initial_vals = this->integrator_->getInitPrecentual(); 
            this->msgoutput_.header.stamp = ros::Time::now();
            this->msgoutput_.softpredict.data = initial_vals;
            this->msgoutput_.decoder.classes = this->classes_;
            this->pub_normalized_.publish(this->msgoutput_);
            this->pub_raw_.publish(this->msgoutput_);
            return true;
        }

        void Integrator::onReceivedEvent(const rosneuro_msgs::NeuroEvent& msg) {
            if(msg.event == this->reset_event_) {
                this->resetIntegrator();
            }
        }

        void Integrator::setMessage(const Eigen::VectorXf& data) {
            this->msgoutput_.softpredict.data = this->eigenToVector(data);
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
