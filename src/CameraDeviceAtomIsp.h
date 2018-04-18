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
  int getSize(uint32_t &width, uint32_t &height);
  int setPixelFormat(CameraDevice::PixelFormat format);
  int getPixelFormat(uint32_t &format);
  int setMode(uint32_t mode);
  int getMode();

private:
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
  std::mutex mLock;
  void **mFrmBuf;
  size_t mBufLen;
  uint32_t mBufCnt;
  char *outBuf;
  std::atomic<int> mState;
};
