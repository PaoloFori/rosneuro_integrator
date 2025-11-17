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
#include "artifacts_cvsa/artifact_presence.h"
#include "rosneuro_integrator/GenericIntegrator.h"

namespace rosneuro {
	namespace integrator {
        class Integrator {
            struct MessageSet{
                ros::Time timestamp;
                std::shared_ptr<rosneuro_msgs::NeuroOutput> msg_icnic;
                std::shared_ptr<rosneuro_msgs::NeuroOutput> msg_classifier;
                std::shared_ptr<artifacts_cvsa::artifact_presence> msg_artifact;

                MessageSet() : msg_icnic(nullptr), msg_classifier(nullptr), msg_artifact(nullptr) {}
            };
            public:
                Integrator(void);
                ~Integrator(void);

                bool configure(void);
                void run(void);

            protected:
                virtual boost::shared_ptr<GenericIntegrator> setIntegrator(void);

            private:
                void onReceivedData_classifier(const rosneuro_msgs::NeuroOutput& msg);
                void onReceivedData_icnic(const rosneuro_msgs::NeuroOutput& msg);
                void onReceivedData_artifacts(const artifacts_cvsa::artifact_presence& msg);
                void onReceivedEvent(const rosneuro_msgs::NeuroEvent& msg);
                bool onResetIntegrator(std_srvs::Empty::Request& req,
                                       std_srvs::Empty::Response& res);
                bool resetIntegrator(void);
                Eigen::VectorXf vectorToEigen(const std::vector<float>& in);
                std::vector<float> eigenToVector(const Eigen::VectorXf& in);
                bool isOverThreshold(const Eigen::VectorXf& values);
                void setMessage(const Eigen::VectorXf& data);
                void subscribeAdvertiseServices(void);
                bool loadPlugin(void);
                void pruneBuffer(const ros::TimerEvent& event);
                void integrateSyncData(const rosneuro_msgs::NeuroOutput& msg_icnic, 
                                       const rosneuro_msgs::NeuroOutput& msg_classifier, 
                                       const artifacts_cvsa::artifact_presence& msg_artifact);

                ros::NodeHandle nh_, p_nh_;
                ros::Subscriber	sub_icnic_, sub_classifier_, sub_artifacts_, sub_event_;
                ros::Publisher	pub_;
                ros::ServiceServer srv_reset_;

                rosneuro_msgs::NeuroOutput msgoutput_;

                int  reset_event_, ic_class_;
                const int reset_event_default_ = 781;
                const int ic_class_default_ = 1;
                float ic_threshold_;

                // for the data synchronization
                ros::Timer prune_timer_;
                ros::Duration max_age_;
                std::map<uint32_t, MessageSet> buffer_; 
                std::mutex buffer_mutex_;

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
