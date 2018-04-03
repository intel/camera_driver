/*
 * This file is part of the Camera Streaming Daemon
 *
 * Copyright (C) 2017  Intel Corporation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
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
  int getInfo();
  std::string getGstSrc();
  int init();
  int uninit();
  int start();
  int stop();
  int read(CameraFrame &frame);
  int getSize(uint32_t &width, uint32_t &height);
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
  uint32_t mPixelFormat;
  std::mutex mLock;
  void **mFrmBuf;
  size_t mBufLen;
  uint32_t mBufCnt;
  std::atomic<int> mState;
};
