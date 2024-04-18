#include "rosneuro_integrator/GenericIntegrator.h"
#include <gtest/gtest.h>

namespace rosneuro {
    namespace integrator {
        class TestGenericIntegrator : public GenericIntegrator {
            public:
                TestGenericIntegrator() : GenericIntegrator() {}
                ~TestGenericIntegrator() {}
                bool configure() { return true; }
                Eigen::VectorXf apply(const Eigen::VectorXf& in) { return in; }
                bool reset() { return true; }
        };

        TEST(TestGenericIntegratorSuite, TestGenericIntegratorCreation) {
            TestGenericIntegrator* integrator = new TestGenericIntegrator();

            EXPECT_EQ(integrator->name(), "undefined");
            integrator->setName("test");
            EXPECT_EQ(integrator->name(), "test");

            ASSERT_TRUE(integrator->reset());
            ASSERT_TRUE(integrator->configure());

            Eigen::VectorXf in(2), expected_out(2);
            in << 1.0, 2.0;
            expected_out << 1.0, 2.0;
            EXPECT_EQ(integrator->apply(in), expected_out);

            delete integrator;
        }
    }
}

int main(int argc, char **argv) {
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Fatal);
    ros::init(argc, argv, "test_generic_integrator");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}