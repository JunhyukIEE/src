// Copyright 2026 FML
// Licensed under the Apache License, Version 2.0.
#include "colored_pose_with_covariance_history/value_overlay.hpp"

#include <QApplication>

#include <gtest/gtest.h>

TEST(ValueOverlay, FreshValueUsesPathColorAndStaleValueIsNeutral)
{
  int argc = 1;
  char name[] = "overlay_test";
  char * argv[] = {name, nullptr};
  QApplication app(argc, argv);
  for (bool fresh : {true, false}) {
    QImage image(200, 112, QImage::Format_ARGB32);
    image.fill(Qt::transparent);
    {
      QPainter painter(&image);
      rviz_plugins::paint_value_overlay(
        painter, "NVTL", 2.30, fresh, Qt::cyan, 2.30, 3.30, Qt::cyan, Qt::yellow);
    }
    int colored_pixels = 0;
    for (int y = 30; y < 72; ++y) {
      for (int x = 16; x < 184; ++x) {
        const auto c = image.pixelColor(x, y);
        if (c.green() > 150 && c.blue() > 150 && c.red() < 50) ++colored_pixels;
      }
    }
    if (fresh)
      EXPECT_GT(colored_pixels, 30);
    else
      EXPECT_EQ(colored_pixels, 0);
    EXPECT_EQ(image.pixelColor(100, 5).alpha(), 150);
    EXPECT_EQ(image.pixelColor(0, 0).alpha(), 0);
  }
}
