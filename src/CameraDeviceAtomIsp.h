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
  std::string getDeviceId() const;
  Status getInfo(CameraInfo &camInfo) const;
  std::string getGstSrc() const;
  Status init();
  Status uninit();
  Status start();
  Status stop();
  Status read(CameraFrame &frame);
  Status setSize(uint32_t width, uint32_t height);
  Status getSize(uint32_t &width, uint32_t &height) const;
  Status setPixelFormat(CameraDevice::PixelFormat format);
  Status getPixelFormat(CameraDevice::PixelFormat &format) const;
  Status setMode(CameraDevice::Mode mode);
  Status getMode(CameraDevice::Mode &mode) const;

private:
  static void uyvy2mono8(const uint8_t *UYVY, uint8_t *MONO, int width,
                         int height, int stride);
  void transform(const uint8_t *input, uint8_t *output);
  int allocFrameBuffer(int bufCnt, size_t bufSize);
  int freeFrameBuffer();
  Status setState(State state);
  State getState() const;
  int pollCamera(int camFd);
  std::string mDeviceId;
  int mFd;
  Mode mMode;
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
  std::atomic<State> mState;
};
