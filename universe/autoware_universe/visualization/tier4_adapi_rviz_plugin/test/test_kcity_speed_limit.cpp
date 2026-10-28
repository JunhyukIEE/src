// Copyright 2026 FML
// Licensed under the Apache License, Version 2.0.
#include "kcity_speed_limit.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace kcity = tier4_adapi_rviz_plugins::kcity;

TEST(KCitySpeedLimit, ExactCoordinatesAndBoundaries)
{
  EXPECT_NEAR(kcity::speed_zone_start_x, 2657.074, 1e-8);
  EXPECT_NEAR(kcity::speed_zone_start_y, 24401.495, 1e-8);
  EXPECT_NEAR(kcity::speed_zone_end_x, 2665.214, 1e-8);
  EXPECT_NEAR(kcity::speed_zone_end_y, 23779.252, 1e-8);
  EXPECT_TRUE(kcity::in_unlimited_speed_zone(kcity::speed_zone_start_x, kcity::speed_zone_start_y));
  EXPECT_FALSE(kcity::in_unlimited_speed_zone(kcity::speed_zone_end_x, kcity::speed_zone_end_y));
  EXPECT_FALSE(kcity::in_unlimited_speed_zone(2657, 24402));
  EXPECT_FALSE(kcity::in_unlimited_speed_zone(2665, 23779));
  EXPECT_TRUE(kcity::in_unlimited_speed_zone(2670, 24100));   // Adjacent highway lane.
  EXPECT_FALSE(kcity::in_unlimited_speed_zone(2500, 24100));  // Unrelated parallel road.
}

TEST(KCitySpeedLimit, NoToleranceAndShortExceedanceRemainsVisible)
{
  kcity::SpeedLimitWatch watch;
  EXPECT_FALSE(watch.update(2665, 23778, 60.0 / 3.6, 10, true).warning);
  auto status = watch.update(2665, 23778, 60.01 / 3.6, 11, true);
  EXPECT_TRUE(status.warning);
  EXPECT_NEAR(status.peak_kmh, 60.01, 1e-8);
  EXPECT_TRUE(watch.update(2665, 23778, 59.0 / 3.6, 12.9, true).warning);
  EXPECT_FALSE(watch.update(2665, 23778, 59.0 / 3.6, 13.0, true).warning);
}

TEST(KCitySpeedLimit, ExitReentryAndMidSectionStartup)
{
  kcity::SpeedLimitWatch watch;
  auto status = watch.update(2660, 24100, 100.0 / 3.6, 10, true);
  EXPECT_TRUE(status.unlimited);
  EXPECT_FALSE(status.warning);
  status = watch.update(kcity::speed_zone_end_x, kcity::speed_zone_end_y, 80.0 / 3.6, 11, true);
  EXPECT_FALSE(status.unlimited);
  EXPECT_TRUE(status.warning);
  EXPECT_FALSE(watch.update(2660, 24100, 100.0 / 3.6, 12, true).warning);
}

TEST(KCitySpeedLimit, InvalidOrStaleInputsNeverAppearNormal)
{
  kcity::SpeedLimitWatch watch;
  EXPECT_TRUE(watch.update(2665, 23778, 20, 10, true).warning);
  auto stale = watch.update(2665, 23778, 20, 10.5, false);
  EXPECT_FALSE(stale.valid);
  EXPECT_FALSE(stale.warning);
  EXPECT_FALSE(watch.update(2665, 23778, 0, 11, true).warning);
  const auto nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(watch.update(nan, 23778, 0, 12, true).valid);
  EXPECT_FALSE(watch.update(2665, 23778, nan, 12, true).valid);
}
