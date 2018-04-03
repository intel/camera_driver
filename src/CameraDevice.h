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
#include <errno.h>
#include <functional>
#include <stdint.h>
#include <vector>

#define log_error(fmt, ...)                                                    \
  printf("[Error] ");                                                          \
  printf(fmt, ##__VA_ARGS__);                                                  \
  printf("\n")
#define log_debug(fmt, ...)                                                    \
  printf("[Debug] ");                                                          \
  printf(fmt, ##__VA_ARGS__);                                                  \
  printf("\n")

class CameraInfo {
public:
  CameraInfo() {}
  ~CameraInfo() {}

  std::string name;
  uint32_t width;
  uint32_t height;
};

class CameraFrame {
public:
  CameraFrame() {}
  ~CameraFrame() {}

  uint32_t width;
  uint32_t height;
  uint32_t stride;
  uint32_t pixFmt;
  void *buf;
  size_t bufSize;
};

class CameraDevice {
public:
  CameraDevice() {}
  virtual ~CameraDevice() {}

  enum State {
    STATE_ERROR = -1,
    STATE_IDLE = 0,
    STATE_INIT = 1,
    STATE_RUN = 2,
  };

  enum Mode {
    MODE_STILL = 0,
    MODE_VIDEO = 1,
  };

  enum PixelFormat {
    PIXEL_FORMAT_INVALID = 0,
    PIXEL_FORMAT_YUV420,
    PIXEL_FORMAT_YUV422,
    PIXEL_FORMAT_UYVY,
    PIXEL_FORMAT_RGB24,
    PIXEL_FORMAT_RGB32
  };

  virtual std::string getDeviceId() = 0;
  virtual int getInfo() = 0;
  virtual std::string getGstSrc() = 0;
  virtual int init() = 0;
  virtual int uninit() = 0;
  virtual int start() = 0;
  virtual int stop() = 0;
  virtual int read(CameraFrame &frame) = 0;
  virtual int setSize(uint32_t &width, uint32_t &height) { return -ENOTSUP; }
  virtual int getSize(uint32_t &width, uint32_t &height) { return -ENOTSUP; }
  virtual int setPixelFormat(CameraDevice::PixelFormat format) {
    return -ENOTSUP;
  }
  virtual int getPixelFormat(CameraDevice::PixelFormat &format) {
    return -ENOTSUP;
  }
  virtual int setMode(CameraDevice::Mode mode) { return -ENOTSUP; }
  virtual int getMode(CameraDevice::Mode &mode) { return -ENOTSUP; }
};