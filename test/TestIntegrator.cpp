#include "rosneuro_integrator/Integrator.h"
#include <gtest/gtest.h>

namespace rosneuro {
    namespace integrator {
        class TestGenericIntegrator : public GenericIntegrator {
            public:
                TestGenericIntegrator() : GenericIntegrator() {}
                ~TestGenericIntegrator() {}
                bool configure(void) {
                    return true;
                }
                Eigen::VectorXf apply(const Eigen::VectorXf& in) {
                    return in;
                }
                bool reset(void) {
                    return true;
                }
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
                    delete integrator;
                }
                TestIntegrator* integrator;
                std::string integrator_plugin = "test";
        };


        TEST_F(TestIntegratorSuite, TestConstructor) {
            EXPECT_FALSE(integrator->has_new_data_);
            EXPECT_TRUE(integrator->is_first_message_);
            EXPECT_NE(integrator->loader_, nullptr);
        }

        TEST_F(TestIntegratorSuite, TestConfigure) {
            ros::param::set("~plugin", integrator_plugin);
            EXPECT_TRUE(integrator->configure());
        }

        TEST_F(TestIntegratorSuite, TestWrongConfigure) {
            EXPECT_FALSE(integrator->configure());
        }

        TEST_F(TestIntegratorSuite, TestOnReceivedData) {
            ros::param::set("~plugin", integrator_plugin);
            EXPECT_TRUE(integrator->configure());

            rosneuro_msgs::NeuroHeader header;
            header.seq = 1;

            rosneuro_msgs::NeuroOutput msg;
            msg.softpredict.data = {1.0, 2.0};
            msg.neuroheader = header;
            msg.hardpredict.data = {1, 2};
            msg.decoder.classes = {"a", "b"};
            msg.decoder.type = "type";
            msg.decoder.path = "path";

            integrator->onReceivedData(msg);
            EXPECT_TRUE(integrator->has_new_data_);
            EXPECT_FALSE(integrator->is_first_message_);
            EXPECT_EQ(integrator->msgoutput_.neuroheader, msg.neuroheader);
            EXPECT_EQ(integrator->msgoutput_.hardpredict.data, msg.hardpredict.data);
            EXPECT_EQ(integrator->msgoutput_.decoder.classes, msg.decoder.classes);
            EXPECT_EQ(integrator->msgoutput_.decoder.type, msg.decoder.type);
            EXPECT_EQ(integrator->msgoutput_.decoder.path, msg.decoder.path);
        }

        TEST_F(TestIntegratorSuite, TestSetMessage) {
            Eigen::VectorXf data(2);
            data << 1.0, 2.0;

            integrator->setMessage(data);

            EXPECT_EQ(integrator->msgoutput_.softpredict.data, std::vector<float>({1.0, 2.0}));
        }

        TEST_F(TestIntegratorSuite, TestResetIntegrator) {
            ros::param::set("~plugin", integrator_plugin);
            EXPECT_TRUE(integrator->configure());
            integrator->output_ = Eigen::VectorXf::Ones(2);

            EXPECT_TRUE(integrator->resetIntegrator());
            EXPECT_FALSE(integrator->has_new_data_);
        }

        TEST_F(TestIntegratorSuite, TestOnReceivedEvent) {
            ros::param::set("~plugin", integrator_plugin);
            EXPECT_TRUE(integrator->configure());
            integrator->has_new_data_ = true;
            rosneuro_msgs::NeuroEvent msg;
            msg.event = 781;

            integrator->onReceivedEvent(msg);

            EXPECT_FALSE(integrator->has_new_data_);
        }

        TEST_F(TestIntegratorSuite, TestOnReceivedEventWrong) {
            ros::param::set("~plugin", integrator_plugin);
            EXPECT_TRUE(integrator->configure());
            integrator->has_new_data_ = true;
            rosneuro_msgs::NeuroEvent msg;

            integrator->onReceivedEvent(msg);

            EXPECT_TRUE(integrator->has_new_data_);
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