//
// Created by ivandor on 8/7/19.
//

#include "ds_sensors/isd4000.h"

#include <list>
#include <gtest/gtest.h>

// Test fixture for parsing, needed because we want to call ros::Time::init() before our tests.
class isd4000Test : public ::testing::Test
{
public:
    // This method runs ONCE before a text fixture is run (not once-per-test-case)
    static void SetUpTestCase()
    {
        ros::Time::init();
    }
};

TEST_F(isd4000Test, validPressureParses)
{
// Simple function to create a ds_sensors_msg::VectorMagneticField from arguments
auto depth = [](double pressure) {
    auto msg = ds_sensor_msgs::DepthPressure{};
    msg.depth = ds_sensor_msgs::DepthPressure::DEPTH_PRESSURE_NO_DATA;
    msg.pressure_raw = pressure;
    return msg;
};

// Add new test cases here.  These should pass
const auto test_pairs =
        std::list<std::pair<std::string, ds_sensor_msgs::DepthPressure>>{ { "$ISDPT,-000.137,M,-001.0133,B,22.77,C*20", depth(-1.0133) },
{ "$ISDPT,001.245,M,001.0133,B,22.77,C*20", depth(1.0133) }, { "$ISDPT,05423.34,M,2423526523.32,B,23532505.2,C*23", depth(2423526523.32)}};

// Loop through all provided cases
for (const auto& test_pair : test_pairs)
{
auto test_str = test_pair.first;
auto test_msg = test_pair.second;

// Construct a ByteSequence message
auto byte_msg = ds_core_msgs::RawData{};
auto now = ros::Time::now();
byte_msg.ds_header.io_time = now;
byte_msg.data = std::vector<unsigned char>(std::begin(test_str), std::end(test_str));

auto ok = false;
auto parsed_msg = ds_sensor_msgs::DepthPressure{};

std::tie(ok, parsed_msg) = ds_sensors::isd4000::parse_pressure(byte_msg);

// Should have succeeded
EXPECT_TRUE(ok);

// All fields should match.
EXPECT_FLOAT_EQ(now.toSec(), parsed_msg.header.stamp.toSec());
EXPECT_FLOAT_EQ(test_msg.depth, parsed_msg.depth);
EXPECT_FLOAT_EQ(test_msg.pressure_raw, parsed_msg.pressure_raw);
}
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}