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
  // int getInfo(struct CameraInfo &camInfo);
  int getInfo();
  bool isGstV4l2Src();
  // int init(CameraParameters &camParam);
  int init();
  int uninit();
  int start();
  // int start(std::function<void(std::vector<uint8_t>)> cb);
  int start(std::function<void(void *, size_t)> cb);
  int stop();
  std::vector<uint8_t> read();
  int getSize(uint32_t &width, uint32_t &height);
  int getPixelFormat(uint32_t &format);
  int setMode(uint32_t mode);
  int getMode();

private:
  int allocFrameBuffer(int bufCnt, size_t bufSize);
  int freeFrameBuffer();
  void cameraThread(int fd);
  int read_frame(void);
  std::string mDeviceId;
  int mFd;
  int mMode;
  uint32_t mWidth;
  uint32_t mHeight;
  uint32_t mPixelFormat;
  std::thread mThread;
  std::mutex mLock;
  std::vector<uint8_t> mFrameBuffer = {};
  void *buffers[4];
  uint32_t bufLen;
  uint32_t mBufCnt;
  std::atomic<int> mState;
  std::function<void(std::vector<uint8_t>)> mFrameCB;
  std::function<void(void *, size_t)> mFrameCB2;
};
