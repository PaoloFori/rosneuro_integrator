#ifndef ROSNEURO_INTEGRATORS_INTEGRATOR_H_
#define ROSNEURO_INTEGRATORS_INTEGRATOR_H_

#include <memory>
#include <map>
#include <mutex>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <pluginlib/class_loader.h>
#include <gtest/gtest_prod.h>
#include <rosneuro_msgs/NeuroOutput.h>
#include <rosneuro_msgs/NeuroEvent.h>
#include "artifacts_bci/artifact_presence.h"
#include "rosneuro_integrator/GenericIntegrator.h"

namespace rosneuro {
    const std::vector<int> hybrid_classes = {750, 751};
	namespace integrator {
        class Integrator {
            struct Sync_Set{
                ros::Time timestamp;
                std::shared_ptr<rosneuro_msgs::NeuroOutput> msg_cvsa;
                std::shared_ptr<rosneuro_msgs::NeuroOutput> msg_mi;
                std::shared_ptr<artifacts_bci::artifact_presence> msg_artifact;

                Sync_Set() : msg_cvsa(nullptr), msg_mi(nullptr), msg_artifact(nullptr) {}
            };

            public:
                Integrator(void);
                ~Integrator(void);

                bool configure(void);
                void run(void);

            protected:
                virtual boost::shared_ptr<GenericIntegrator> setIntegrator(void);

            private:
                void onReceivedData_cvsa(const rosneuro_msgs::NeuroOutput& msg);
                void onReceivedData_mi(const rosneuro_msgs::NeuroOutput& msg);
                void onReceivedData_artifacts(const artifacts_bci::artifact_presence& msg);
                void onReceivedEvent(const rosneuro_msgs::NeuroEvent& msg);
                bool resetIntegrator(void);
                Eigen::VectorXf vectorToEigen(const std::vector<float>& in);
                std::vector<float> eigenToVector(const Eigen::VectorXf& in);
                bool isOverThreshold(const Eigen::VectorXf& values);
                void setMessage(const Eigen::VectorXf& data);
                bool loadPlugin(void);
                void pruneBuffer(const ros::TimerEvent& event);
                void integrateSyncData( std::shared_ptr<rosneuro_msgs::NeuroOutput> mi,
                                        std::shared_ptr<rosneuro_msgs::NeuroOutput> cvsa,
                                        std::shared_ptr<artifacts_bci::artifact_presence> artifact);

                ros::NodeHandle nh_, p_nh_;
                ros::Subscriber	sub_cvsa_, sub_mi_, sub_artifacts_, sub_events_;
                ros::Publisher	pub_raw_, pub_normalized_; 

                rosneuro_msgs::NeuroOutput msgoutput_;
                std::string paradigm_;

                std::vector<int> classes_;

                int  reset_event_;
                const int reset_event_default_ = 781;

                float cvsa_influence_;
                float cvsa_influence_default_ = 3.0;

                // for the data synchronization
                ros::Timer prune_timer_;
                ros::Duration max_age_;
                std::map<uint32_t, Sync_Set> sync_set_; 
                std::mutex mutex_;
                ros::Time start_cf_;

                std::string plugin_, integrator_name_;

                boost::shared_ptr<GenericIntegrator> integrator_;
                std::unique_ptr<pluginlib::ClassLoader<GenericIntegrator>> loader_;

                FRIEND_TEST(TestIntegratorSuite, TestConstructor);
                FRIEND_TEST(TestIntegratorSuite, TestConfigure);
                FRIEND_TEST(TestIntegratorSuite, TestWrongConfigure);
                FRIEND_TEST(TestIntegratorSuite, TestOnReceivedData);
                FRIEND_TEST(TestIntegratorSuite, TestSetMessage);
                FRIEND_TEST(TestIntegratorSuite, TestResetIntegrator);
                FRIEND_TEST(TestIntegratorSuite, TestOnReceivedEvent);
                FRIEND_TEST(TestIntegratorSuite, TestOnReceivedEventWrong);
                FRIEND_TEST(TestIntegratorSuite, TestVectorToEigen);
                FRIEND_TEST(TestIntegratorSuite, TestEigenToVector);
        };
	}
}

#endif
