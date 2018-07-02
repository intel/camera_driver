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

  std::string name = "/dev/video0";
  uint32_t width = 0;
  uint32_t height = 0;
};

class CameraFrame {
public:
  CameraFrame() {}
  ~CameraFrame() {}

  uint32_t sec = 0;  // system time in sec
  uint32_t nsec = 0; // system time in nano sec
  uint32_t width = 0;
  uint32_t height = 0;
  uint32_t stride = 0;
  uint32_t pixFmt = 0;
  void *buf = nullptr;
  size_t bufSize = 0;
};

class CameraDevice {
public:
  CameraDevice() {}
  virtual ~CameraDevice() {}

  enum class Status {
    SUCCESS,
    ERROR_UNKNOWN,
    INVALID_ARGUMENT,
    INVALID_STATE,
    NO_MEMORY,
    PERM_DENIED,
    TIMED_OUT,
    NOT_SUPPORTED
  };

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
  virtual Status getInfo(CameraInfo &camInfo) = 0;
  virtual std::string getGstSrc() = 0;
  virtual Status init() = 0;
  virtual Status uninit() = 0;
  virtual Status start() = 0;
  virtual Status stop() = 0;
  virtual Status read(CameraFrame &frame) = 0;
  virtual Status setSize(uint32_t width, uint32_t height) {
    return Status::NOT_SUPPORTED;
  }
  virtual Status getSize(uint32_t &width, uint32_t &height) const {
    return Status::NOT_SUPPORTED;
  }
  virtual Status setPixelFormat(CameraDevice::PixelFormat format) {
    return Status::NOT_SUPPORTED;
  }
  virtual Status getPixelFormat(CameraDevice::PixelFormat &format) {
    return Status::NOT_SUPPORTED;
  }
  virtual Status setMode(CameraDevice::Mode mode) {
    return Status::NOT_SUPPORTED;
  }
  virtual Status getMode(CameraDevice::Mode &mode) {
    return Status::NOT_SUPPORTED;
  }
};
