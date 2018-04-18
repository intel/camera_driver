/*********************************************************************
 * Copyright (C) 2018 Intel Corporation
 * SPDX-License-Identifier: BSD-3-Clause
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
    PIXEL_FORMAT_MIN = 0,
    PIXEL_FORMAT_GREY,    /* 8  bpp monochrome images */
    PIXEL_FORMAT_YUV420,  /* 12 bpp YUV 4:2:0 */
    PIXEL_FORMAT_YUV422P, /* 16 bpp YVU422 planar */
    PIXEL_FORMAT_UYVY,    /* 16 bpp YUV 4:2:2 */
    PIXEL_FORMAT_RGB24,   /* 24 bpp RGB 8:8:8 */
    PIXEL_FORMAT_RGB32,   /* 32 bpp RGB 8:8:8:8 */
    PIXEL_FORMAT_MAX = 99
  };

  virtual std::string getDeviceId() = 0;
  virtual int getInfo(CameraInfo &camInfo) = 0;
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
