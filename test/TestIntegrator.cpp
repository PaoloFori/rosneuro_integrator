#include "rosneuro_integrator/Integrator.h"
#include <gtest/gtest.h>

namespace rosneuro {
    namespace integrator {
        class TestGenericIntegrator : public GenericIntegrator {
            public:
                TestGenericIntegrator() : GenericIntegrator() {}
                ~TestGenericIntegrator() {}
                bool configure(void) { return true; }
                Eigen::VectorXf apply(const Eigen::VectorXf& in) { return in; }
                bool reset(void) { return true; }
                Eigen::VectorXf getData(void) { return Eigen::VectorXf::Constant(2, 0.5f); }
                std::vector<float> getInitPrecentual(void) { return {0.5f, 0.5f}; }
        };

        class TestIntegrator : public Integrator {
            public:
                TestIntegrator() : Integrator() {}
                ~TestIntegrator() {}
                boost::shared_ptr<GenericIntegrator> setIntegrator(void) override {
                    return boost::shared_ptr<GenericIntegrator>(new TestGenericIntegrator());
                }
        };

        class TestIntegratorSuite : public ::testing::Test {
            public:
                TestIntegratorSuite() {}
                ~TestIntegratorSuite() {}
                void SetUp() {
                    integrator = new TestIntegrator();
                }
                void TearDown() {
                    ros::param::del("~plugin");
                    ros::param::del("~paradigm");
                    ros::param::del("~classes");
                    ros::param::del("~thresholds");
                    delete integrator;
                }

                void setConfigureParams(const std::string& paradigm = "mi") {
                    ros::param::set("~plugin", std::string("test"));
                    ros::param::set("~paradigm", paradigm);
                    std::vector<int> classes = {769, 770};
                    ros::param::set("~classes", classes);
                    std::vector<double> thresholds = {0.7, 0.7};
                    ros::param::set("~thresholds", thresholds);
                }

                TestIntegrator* integrator;
        };


        TEST_F(TestIntegratorSuite, TestConstructor) {
            EXPECT_NE(integrator->loader_, nullptr);
        }

        TEST_F(TestIntegratorSuite, TestConfigure) {
            setConfigureParams();
            EXPECT_TRUE(integrator->configure());
        }

        TEST_F(TestIntegratorSuite, TestWrongConfigure) {
            EXPECT_FALSE(integrator->configure());
        }

        TEST_F(TestIntegratorSuite, TestOnReceivedData) {
            setConfigureParams("mi");
            EXPECT_TRUE(integrator->configure());

            rosneuro_msgs::NeuroOutput msg;
            msg.softpredict.data = {0.6f, 0.4f};
            msg.neuroheader.seq = 42;
            msg.header.stamp = ros::Time::now();

            integrator->onReceivedData_mi(msg);
            // MI arrived but artifact not yet → one incomplete entry in sync_set_
            EXPECT_EQ(integrator->sync_set_.size(), 1u);
        }

        TEST_F(TestIntegratorSuite, TestSetMessage) {
            Eigen::VectorXf data(2);
            data << 1.0, 2.0;

            integrator->setMessage(data);

            EXPECT_EQ(integrator->msgoutput_.softpredict.data, std::vector<float>({1.0, 2.0}));
        }

        TEST_F(TestIntegratorSuite, TestResetIntegrator) {
            setConfigureParams();
            EXPECT_TRUE(integrator->configure());
            EXPECT_TRUE(integrator->resetIntegrator());
        }

        TEST_F(TestIntegratorSuite, TestOnReceivedEvent) {
            setConfigureParams();
            EXPECT_TRUE(integrator->configure());
            ros::Time before = ros::Time::now();
            rosneuro_msgs::NeuroEvent msg;
            msg.event = 781;
            integrator->onReceivedEvent(msg);
            EXPECT_GE(integrator->start_cf_, before);
        }

        TEST_F(TestIntegratorSuite, TestOnReceivedEventWrong) {
            setConfigureParams();
            EXPECT_TRUE(integrator->configure());
            ros::Time saved_cf = integrator->start_cf_;
            rosneuro_msgs::NeuroEvent msg;
            msg.event = 999; // not reset event
            integrator->onReceivedEvent(msg);
            EXPECT_EQ(integrator->start_cf_, saved_cf); // start_cf_ must not change
        }

        TEST_F(TestIntegratorSuite, TestVectorToEigen) {
            std::vector<float> in = {1.0, 2.0};
            Eigen::VectorXf out = integrator->vectorToEigen(in);
            Eigen::VectorXf expected(2);
            expected << 1.0, 2.0;
            EXPECT_EQ(out, expected);
        }

        TEST_F(TestIntegratorSuite, TestEigenToVector) {
            Eigen::VectorXf in(2);
            in << 1.0, 2.0;
            std::vector<float> out = integrator->eigenToVector(in);
            std::vector<float> expected = {1.0, 2.0};
            EXPECT_EQ(out, expected);
        }
    }
}

int main(int argc, char **argv) {
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Fatal);
    ros::init(argc, argv, "test_integrator");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
