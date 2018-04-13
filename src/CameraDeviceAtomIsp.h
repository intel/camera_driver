/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (C) 2018 Intel Corporation
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the author nor other contributors may be
*     used to endorse or promote products derived from this software
*     without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
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
