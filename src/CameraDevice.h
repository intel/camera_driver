/*********************************************************************
 * Copyright (C) 2018 Intel Corporation
 * SPDX-License-Identifier: BSD-3-Clause
*********************************************************************/

#pragma once
#include <errno.h>
#include <functional>
#include <stdint.h>
#include <vector>

/**
 *  The CameraInfo class is used to hold the information of the camera device.
 */
class CameraInfo {
public:
  CameraInfo() {}
  ~CameraInfo() {}

  std::string name = "/dev/video0";
  uint32_t width = 0;
  uint32_t height = 0;
};

/**
 *  The CameraFrame class is used to hold the camera image data and its
 * meta-data.
 */
struct CameraFrame {
  uint32_t sec = 0;  /**< system time in sec. */
  uint32_t nsec = 0; /**< system time in nano sec. */
  uint32_t width = 0;
  uint32_t height = 0;
  uint32_t stride = 0;
  uint32_t pixFmt = 0;
  void *buf = nullptr;
  size_t bufSize = 0;
};

/**
 *  The CameraDevice class is an abstraction for different camera devices.
 */
class CameraDevice {
public:
  /**
  *  Constructor. Creates the camera device object.
  */
  CameraDevice() {}

  /**
   *  Destructor.
   */
  virtual ~CameraDevice() {}

  /**
   *  Possible return values from class methods.
   */
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

  /**
   *  States of the object.
   */
  enum State {
    ERROR = -1,
    IDLE = 0,
    INIT = 1,
    RUN = 2,
  };

  /**
   *  Modes of camera device.
   */
  enum Mode {
    STILL = 0,
    VIDEO = 1,
  };

  /**
   *  Pixel formats of the image.
   */
  enum PixelFormat {
    MIN = 0,
    GREY,    /* 8  bpp monochrome images */
    YUV420,  /* 12 bpp YUV 4:2:0 */
    YUV422P, /* 16 bpp YVU422 planar */
    UYVY,    /* 16 bpp YUV 4:2:2 */
    RGB24,   /* 24 bpp RGB 8:8:8 */
    RGB32,   /* 32 bpp RGB 8:8:8:8 */
    MAX = 99
  };

  /**
   *  Get the identifier of the Camera Device.
   *
   *  @return string Camera device Id.
   */
  virtual std::string getDeviceId() const = 0;

  /**
   *  Get camera device information.
   *
   *  @param[out] camInfo Camera Information
   *
   *  @return Status of request.
   */
  virtual Status getInfo(CameraInfo &camInfo) const = 0;

  /**
   *  Get gstreamer src element supported by the Camera Device
   *
   *  @param[out] camInfo Camera Information
   *
   *  @return string Gstreamer src element name, empty if gstreamer not
   * supported
   */
  virtual std::string getGstSrc() const = 0;

  /**
   *  Initialize camera device.
   *
   *  @return Status of request.
   */
  virtual Status init() = 0;

  /**
   *  Uninitialize camera device.
   *
   *  @return Status of request.
   */
  virtual Status uninit() = 0;

  /**
   *  Start camera device.
   *
   *  @return Status of request.
   */
  virtual Status start() = 0;

  /**
   *  Stop camera device.
   *
   *  @return Status of request.
   */
  virtual Status stop() = 0;

  /**
   *  Read camera images from camera device.
   *
   *  @param[out] frame CameraFrame to hold frame and meta-data.
   *
   *  @return Status of request.
   */
  virtual Status read(CameraFrame &frame) = 0;

  /**
   *  Set camera image resolution.
   *
   *  @param[in] width Width of the output image to be set.
   *  @param[in] height Height of the output image to be set.
   *
   *  @return Status of request.
   */
  virtual Status setSize(uint32_t width, uint32_t height) {
    return Status::NOT_SUPPORTED;
  }

  /**
   *  Get camera image resolution.
   *
   *  @param[out] width Width of the output image set.
   *  @param[out] height Height of the output image set.
   *
   *  @return Status of request.
   */
  virtual Status getSize(uint32_t &width, uint32_t &height) const {
    return Status::NOT_SUPPORTED;
  }

  /**
   *  Set camera image pixel format.
   *
   *  @param[in] format Pixel format of the output image to be set.
   *
   *  @return Status of request.
   */
  virtual Status setPixelFormat(CameraDevice::PixelFormat format) {
    return Status::NOT_SUPPORTED;
  }

  /**
   *  Set camera image pixel format.
   *
   *  @param[out] format Pixel format of the output image set.
   *
   *  @return Status of request.
   */
  virtual Status getPixelFormat(CameraDevice::PixelFormat &format) const {
    return Status::NOT_SUPPORTED;
  }

  /**
   *  Set mode of the camera device.
   *
   *  @param[in] mode Camera device mode to be set.
   *
   *  @return Status of request.
   */
  virtual Status setMode(CameraDevice::Mode mode) {
    return Status::NOT_SUPPORTED;
  }

  /**
   *  Get mode of the camera device.
   *
   *  @param[out] mode Camera device mode that is set.
   *
   *  @return Status of request.
   */
  virtual Status getMode(CameraDevice::Mode &mode) const {
    return Status::NOT_SUPPORTED;
  }
};
