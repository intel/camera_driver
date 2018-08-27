/*********************************************************************
 * Copyright (C) 2018 Intel Corporation
 * SPDX-License-Identifier: BSD-3-Clause
*********************************************************************/

#include "../src/CameraDeviceAtomIsp.h"
// Bring in gtest
#include <gtest/gtest.h>
#include <ros/ros.h>

TEST(TestSuite, AccessCamera) {
  CameraDevice *dev = new CameraDeviceAtomIsp("/dev/video2");
  ASSERT_TRUE(dev != nullptr);

  CameraDevice::Status ret = CameraDevice::Status::SUCCESS;
  ret = dev->init();
  ASSERT_EQ(ret, CameraDevice::Status::SUCCESS);

  ret = dev->uninit();
  ASSERT_EQ(ret, CameraDevice::Status::SUCCESS);

  delete dev;
}

TEST(TestSuite, ReadCamera) {
  CameraDevice *dev = new CameraDeviceAtomIsp("/dev/video2");
  ASSERT_TRUE(dev != nullptr);

  CameraDevice::Status ret = CameraDevice::Status::SUCCESS;
  ret = dev->init();
  ASSERT_EQ(ret, CameraDevice::Status::SUCCESS);

  ret = dev->start();
  ASSERT_EQ(ret, CameraDevice::Status::SUCCESS);

  CameraFrame frame;
  ret = dev->read(frame);
  EXPECT_EQ(ret, CameraDevice::Status::SUCCESS);

  ret = dev->stop();
  ASSERT_EQ(ret, CameraDevice::Status::SUCCESS);

  ret = dev->uninit();
  ASSERT_EQ(ret, CameraDevice::Status::SUCCESS);

  delete dev;
}

TEST(TestSuite, SetCameraSize) {
  CameraDevice *dev = new CameraDeviceAtomIsp("/dev/video2");
  ASSERT_TRUE(dev != nullptr);

  CameraDevice::Status ret = CameraDevice::Status::SUCCESS;
  ret = dev->init();
  ASSERT_EQ(ret, CameraDevice::Status::SUCCESS);

  ret = dev->setSize(480, 360);
  EXPECT_EQ(ret, CameraDevice::Status::SUCCESS);

  ret = dev->start();
  ASSERT_EQ(ret, CameraDevice::Status::SUCCESS);

  uint32_t width, height;
  ret = dev->getSize(width, height);
  EXPECT_EQ(ret, CameraDevice::Status::SUCCESS);
  EXPECT_EQ(width, 480);
  EXPECT_EQ(height, 360);

  CameraFrame frame;
  ret = dev->read(frame);
  EXPECT_EQ(ret, CameraDevice::Status::SUCCESS);

  ret = dev->stop();
  ASSERT_EQ(ret, CameraDevice::Status::SUCCESS);

  ret = dev->uninit();
  ASSERT_EQ(ret, CameraDevice::Status::SUCCESS);

  delete dev;
}

TEST(TestSuite, SetCameraPixelFormat) {
  CameraDevice *dev = new CameraDeviceAtomIsp("/dev/video2");
  ASSERT_TRUE(dev != nullptr);

  CameraDevice::Status ret = CameraDevice::Status::SUCCESS;
  ret = dev->init();
  ASSERT_EQ(ret, CameraDevice::Status::SUCCESS);

  ret = dev->setPixelFormat(CameraDevice::GREY);
  EXPECT_EQ(ret, CameraDevice::Status::SUCCESS);

  ret = dev->start();
  ASSERT_EQ(ret, CameraDevice::Status::SUCCESS);

  CameraDevice::PixelFormat format;
  ret = dev->getPixelFormat(format);
  EXPECT_EQ(ret, CameraDevice::Status::SUCCESS);
  EXPECT_EQ(format, CameraDevice::GREY);

  CameraFrame frame;
  ret = dev->read(frame);
  EXPECT_EQ(ret, CameraDevice::Status::SUCCESS);

  ret = dev->stop();
  ASSERT_EQ(ret, CameraDevice::Status::SUCCESS);

  ret = dev->uninit();
  ASSERT_EQ(ret, CameraDevice::Status::SUCCESS);

  delete dev;
}

TEST(TestSuite, SetCameraMode) {
  CameraDevice *dev = new CameraDeviceAtomIsp("/dev/video2");
  ASSERT_TRUE(dev != nullptr);

  CameraDevice::Status ret = CameraDevice::Status::SUCCESS;
  ret = dev->init();
  ASSERT_EQ(ret, CameraDevice::Status::SUCCESS);

  ret = dev->setMode(CameraDevice::VIDEO);
  EXPECT_EQ(ret, CameraDevice::Status::SUCCESS);

  ret = dev->start();
  ASSERT_EQ(ret, CameraDevice::Status::SUCCESS);

  CameraDevice::Mode mode;
  ret = dev->getMode(mode);
  EXPECT_EQ(ret, CameraDevice::Status::SUCCESS);
  EXPECT_EQ(mode, CameraDevice::VIDEO);

  CameraFrame frame;
  ret = dev->read(frame);
  EXPECT_EQ(ret, CameraDevice::Status::SUCCESS);

  ret = dev->stop();
  ASSERT_EQ(ret, CameraDevice::Status::SUCCESS);

  ret = dev->uninit();
  ASSERT_EQ(ret, CameraDevice::Status::SUCCESS);

  delete dev;
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  // ros::init(argc, argv, "utest");
  // ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
