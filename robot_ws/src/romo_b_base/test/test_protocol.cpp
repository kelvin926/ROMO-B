#include "romo_b_base/protocol.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

namespace rb = romo_b_base;

TEST(CommandEncoding, MatchesManualPositiveExample)
{
  const rb::Command command{true, false, rb::SteerMode::k2Wis, 0.50, 10.0, 1};
  const std::array<std::uint8_t, rb::kCommandFrameSize> expected{
    0x53, 0x54, 0x58, 0x01, 0x00, 0x00, 0x00, 0x32, 0x00, 0x0a, 0x01, 0x0d, 0x0a};
  EXPECT_EQ(rb::encode_command(command), expected);
}

TEST(CommandEncoding, MatchesManualNegativeExample)
{
  const rb::Command command{true, false, rb::SteerMode::k2Wis, -0.20, -5.0, 2};
  const std::array<std::uint8_t, rb::kCommandFrameSize> expected{
    0x53, 0x54, 0x58, 0x01, 0x00, 0x00, 0xff, 0xec, 0xff, 0xfb, 0x02, 0x0d, 0x0a};
  EXPECT_EQ(rb::encode_command(command), expected);
}

TEST(CommandEncoding, SupportsMeasuredLittleEndianPcu)
{
  const rb::Command command{true, false, rb::SteerMode::k2Wis, 0.01, -5.0, 3};
  const std::array<std::uint8_t, rb::kCommandFrameSize> expected{
    0x53, 0x54, 0x58, 0x01, 0x00, 0x00, 0x01, 0x00, 0xfb, 0xff, 0x03, 0x0d, 0x0a};
  EXPECT_EQ(rb::encode_command(command, true), expected);
}

TEST(FeedbackDecoding, DecodesLittleEndianAndScales)
{
  const std::array<std::uint8_t, rb::kFeedbackFrameSize> frame{
    0x53, 0x54, 0x58, 0x01, 0x00, 0x00,
    0x32, 0x00, 0x31, 0x00, 0x2f, 0x00, 0x30, 0x00,
    0x9c, 0xff, 0xb0, 0xff, 0x00, 0x00, 0x00, 0x00,
    0x44, 0x0d, 0x0a};
  const auto result = rb::decode_feedback(frame);
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(result->wheel_speed_mps[0], 0.5);
  EXPECT_DOUBLE_EQ(result->wheel_speed_mps[2], 0.47);
  EXPECT_NEAR(result->wheel_steer_rad[0], -10.0 * rb::kDegreesToRadians, 1.0e-9);
  EXPECT_NEAR(result->wheel_steer_rad[1], -8.0 * rb::kDegreesToRadians, 1.0e-9);
  EXPECT_EQ(result->alive, 0x44);
}

TEST(FeedbackDecoding, TreatsPhysicalEstopBitmaskAsActive)
{
  const std::array<std::uint8_t, rb::kFeedbackFrameSize> frame{
    0x53, 0x54, 0x58, 0x00, 0x05, 0x00,
    0x00, 0x00, 0x01, 0x00, 0xff, 0xff, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x3e, 0x0d, 0x0a};
  const auto result = rb::decode_feedback(frame);
  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(result->estop);
  EXPECT_EQ(result->estop_raw, 0x05);
  EXPECT_EQ(result->alive, 0x3e);
}

TEST(FeedbackParser, RecoversFromNoiseCorruptionAndFragments)
{
  std::array<std::uint8_t, rb::kFeedbackFrameSize> good{
    0x53, 0x54, 0x58, 0x01, 0x00, 0x00,
    0x0a, 0x00, 0x0a, 0x00, 0x0a, 0x00, 0x0a, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x01, 0x0d, 0x0a};
  auto corrupt = good;
  corrupt[5] = 0xff;

  std::vector<std::uint8_t> bytes{0xaa, 0x53, 0x00};
  bytes.insert(bytes.end(), corrupt.begin(), corrupt.end());
  bytes.insert(bytes.end(), good.begin(), good.end());

  rb::FeedbackParser parser;
  EXPECT_TRUE(parser.push(bytes.data(), 7).empty());
  const auto decoded = parser.push(bytes.data() + 7, bytes.size() - 7);
  ASSERT_EQ(decoded.size(), 1U);
  EXPECT_EQ(decoded.front().alive, 1U);
}

TEST(TwistMapping, ConvertsRosLeftToPcuNegativeAndClamps)
{
  rb::ControlLimits limits;
  const auto output = rb::map_twist(0.2, 1.0, limits);
  EXPECT_TRUE(output.valid);
  EXPECT_TRUE(output.clamped);
  EXPECT_DOUBLE_EQ(output.speed_mps, 0.2);
  EXPECT_DOUBLE_EQ(output.pcu_steer_deg, -22.0);
}

TEST(TwistMapping, RejectsReverseAndPureRotation)
{
  const rb::ControlLimits limits;
  EXPECT_FALSE(rb::map_twist(-0.1, 0.0, limits).valid);
  EXPECT_FALSE(rb::map_twist(0.0, 0.2, limits).valid);
  EXPECT_TRUE(rb::map_twist(0.0, 0.0, limits).valid);
}

TEST(Odometry, RecoversEquivalentLeftTurn)
{
  rb::Feedback feedback;
  feedback.wheel_speed_mps = {0.18, 0.22, 0.18, 0.22};
  feedback.wheel_steer_rad = {
    -20.0 * rb::kDegreesToRadians, -13.0 * rb::kDegreesToRadians, 0.0, 0.0};
  const auto motion = rb::estimate_vehicle_motion(feedback);
  EXPECT_NEAR(motion.center_speed_mps, 0.20, 1.0e-9);
  EXPECT_GT(motion.equivalent_steer_rad, 0.0);
  EXPECT_GT(motion.yaw_rate_radps, 0.0);
}
