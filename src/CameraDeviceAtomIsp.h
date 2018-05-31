/*********************************************************************
 * Copyright (C) 2018 Intel Corporation
 * SPDX-License-Identifier: BSD-3-Clause
*********************************************************************/

#pragma once
#include <atomic>
#include <mutex>
#include <string>
#include <thread>

#include "CameraDevice.h"

class CameraDeviceAtomIsp final : public CameraDevice {
public:
  CameraDeviceAtomIsp(std::string device);
  ~CameraDeviceAtomIsp();
  std::string getDeviceId();
  int getInfo(CameraInfo &camInfo);
  std::string getGstSrc();
  int init();
  int uninit();
  int start();
  int stop();
  int read(CameraFrame &frame);
  int setSize(uint32_t width, uint32_t height);
  int getSize(uint32_t &width, uint32_t &height) const;
  int setPixelFormat(CameraDevice::PixelFormat format);
  int getPixelFormat(uint32_t &format);
  int setMode(uint32_t mode);
  int getMode();

private:
  static void uyvy2mono8(const uint8_t *UYVY, uint8_t *MONO, int width,
                         int height, int stride);
  void transform(const uint8_t *input, uint8_t *output);
  int allocFrameBuffer(int bufCnt, size_t bufSize);
  int freeFrameBuffer();
  int setState(int state);
  int getState();
  int pollCamera(int camFd);
  std::string mDeviceId;
  int mFd;
  int mMode;
  uint32_t mWidth;
  uint32_t mHeight;
  PixelFormat mPixelFormat;
  uint32_t mOutWidth;
  uint32_t mOutHeight;
  PixelFormat mOutPixelFormat;
  std::mutex mLock;
  void **mFrmBuf;
  size_t mBufLen;
  uint32_t mBufCnt;
  uint8_t *mOutBuf;
  std::atomic<int> mState;
};
