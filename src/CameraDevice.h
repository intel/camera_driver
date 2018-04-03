/*
 * This file is part of the Camera Streaming Daemon
 *
 * Copyright (C) 2018  Intel Corporation. All rights reserved.
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
