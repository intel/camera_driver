/*********************************************************************
 * Copyright (C) 2018 Intel Corporation
 * SPDX-License-Identifier: BSD-3-Clause
*********************************************************************/

#include <algorithm>
#include <iostream>
#include <linux/videodev2.h>
#include <malloc.h>
#include <string.h>
#include <unistd.h>

#include "CameraDeviceAtomIsp.h"
#include "v4l2_interface.h"

#define DEFAULT_WIDTH 640
#define DEFAULT_HEIGHT 480
#define DEFAULT_BUFFER_COUNT 4

CameraDeviceAtomIsp::CameraDeviceAtomIsp(std::string device)
    : mDeviceId(device), mFd(-1), mMode(VIDEO), mWidth(DEFAULT_WIDTH),
      mHeight(DEFAULT_HEIGHT), mPixelFormat(UYVY),
      mOutWidth(mWidth), mOutHeight(mHeight), mOutPixelFormat(mPixelFormat),
      mState(IDLE), mFrmBuf(nullptr), mBufLen(0),
      mBufCnt(DEFAULT_BUFFER_COUNT), mOutBuf(nullptr) {
  log_debug("%s path:%s", __func__, mDeviceId.c_str());
}

CameraDeviceAtomIsp::~CameraDeviceAtomIsp() {
  stop();
  uninit();
  delete mOutBuf;
}

std::string CameraDeviceAtomIsp::getDeviceId() const { return mDeviceId; }

CameraDevice::Status CameraDeviceAtomIsp::getInfo(CameraInfo &camInfo) const {
  camInfo.name = mDeviceId;
  camInfo.width = mWidth;
  camInfo.height = mHeight;

  return Status::SUCCESS;
}

std::string CameraDeviceAtomIsp::getGstSrc() const { return {}; }

CameraDevice::Status CameraDeviceAtomIsp::init() {
  log_debug("%s", __func__);

  int ret = -1;
  Status retStatus = Status::ERROR_UNKNOWN;

  if (getState() != IDLE)
    return Status::INVALID_STATE;

  // Open Camera device
  mFd = v4l2_open(mDeviceId.c_str());
  if (mFd < 0) {
    log_error("Error in opening camera device");
    return retStatus;
  }

  // Set Device ID
  ret = v4l2_set_input(mFd, 1);
  if (ret)
    goto error;

  // Query Capabilities
  ret = v4l2_query_cap(mFd);
  if (ret)
    goto error;

  // Set Parameter - preview mode
  ret = v4l2_set_capturemode(mFd, 0x8000);
  if (ret)
    goto error;

  // Set Image Format
  // TODO:: Check if yuv format or gray format
  // ret = v4l2_set_pixformat(mFd, 640, 480, V4L2_PIX_FMT_YUV420);
  ret = v4l2_set_pixformat(mFd, mWidth, mHeight, V4L2_PIX_FMT_UYVY);
  if (ret)
    goto error;

  // Allocate User buffer
  ret = allocFrameBuffer(mBufCnt, mWidth * mHeight * 2);
  if (ret)
    goto error;

  setState(INIT);
  return Status::SUCCESS;
error:
  v4l2_close(mFd);
  return retStatus;
}

CameraDevice::Status CameraDeviceAtomIsp::uninit() {
  log_debug("%s", __func__);

  if (getState() == IDLE)
    return Status::SUCCESS;

  // Close the Camera Device
  v4l2_close(mFd);
  mFd = -1;

  // Free User buffer
  freeFrameBuffer();

  setState(IDLE);
  return Status::SUCCESS;
}

CameraDevice::Status CameraDeviceAtomIsp::start() {
  log_debug("%s", __func__);

  int ret = 0;

  if (getState() != INIT)
    return Status::INVALID_STATE;

  // request buffer
  ret = v4l2_buf_req(mFd, 4);
  if (ret)
    return Status::ERROR_UNKNOWN;

  struct v4l2_buffer buf;
  CLEAR(buf);
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_USERPTR;
  for (uint8_t i = 0; i < 4; i++) {
    buf.index = i;
    buf.m.userptr = (unsigned long)mFrmBuf[i];
    buf.length = mBufLen;
    ret = v4l2_buf_q(mFd, &buf);
    if (ret)
      return Status::ERROR_UNKNOWN;
  }

  ret = v4l2_streamon(mFd);
  if (ret)
    return Status::ERROR_UNKNOWN;

  setState(RUN);

  return Status::SUCCESS;
}

CameraDevice::Status CameraDeviceAtomIsp::stop() {
  log_debug("%s", __func__);

  if (getState() != RUN)
    return Status::INVALID_STATE;

  v4l2_streamoff(mFd);

  setState(INIT);

  return Status::SUCCESS;
}

CameraDevice::Status CameraDeviceAtomIsp::read(CameraFrame &frame) {
  std::lock_guard<std::mutex> locker(mLock);

  // log_debug("%s", __func__);

  if (getState() != RUN)
    return Status::INVALID_STATE;

  if (pollCamera(mFd))
    return Status::ERROR_UNKNOWN;

  int ret = 0;
  struct v4l2_buffer buf;
  CLEAR(buf);

  // dequeue buffer
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_USERPTR;
  ret = v4l2_buf_dq(mFd, &buf);
  if (ret) {
    log_error("Error in dq buffer");
    return Status::ERROR_UNKNOWN;
  }

  if (!buf.m.userptr) {
    log_error("Null buffer returned");
    return Status::ERROR_UNKNOWN;
  }

  struct timeval timeofday;
  gettimeofday(&timeofday, NULL);
  frame.sec = timeofday.tv_sec;
  frame.nsec = timeofday.tv_usec * 1000;

// TODO:Use v4l2 buffer timestamp instead for more accuracy

#if DEBUG
  log_debug("System Timestamp %ld.%06ld", timeofday.tv_sec, timeofday.tv_usec);
  log_debug("Buffer Timestamp %ld.%06ld", buf.timestamp.tv_sec,
            buf.timestamp.tv_usec);
#endif

  // TODO :: Check if buffer valid
  // log_debug("Buffer size:%d used:%d", buf.length, buf.bytesused);

  // Check if there is need to change format or size
  if ((mOutPixelFormat != mPixelFormat) ||
      (mWidth != mOutWidth || mHeight != mOutHeight)) {
    switch (mOutPixelFormat) {
    case GREY:
      if (!mOutBuf) {
        mOutBuf = new uint8_t[(mOutWidth * mOutHeight)];
      }
      frame.stride = mOutWidth;
      frame.bufSize = mOutWidth * mOutHeight;
      break;
    case UYVY:
      if (!mOutBuf) {
        mOutBuf = new uint8_t[(2 * mOutWidth * mOutHeight)];
      }
      frame.stride = 2 * mOutWidth;
      frame.bufSize = 2 * mOutWidth * mOutHeight;
      break;
    default:
      break;
    }

    transform((uint8_t *)buf.m.userptr, mOutBuf);

    frame.pixFmt = mOutPixelFormat;
    frame.width = mOutWidth;
    frame.height = mOutHeight;
    frame.buf = mOutBuf;

  } else {
    frame.width = mWidth;
    frame.height = mHeight;
    frame.pixFmt = mPixelFormat;
    frame.buf = (void *)buf.m.userptr;
    frame.bufSize = buf.bytesused;
    frame.stride = mWidth * 2;
  }

  // queue buffer for refill
  ret = v4l2_buf_q(mFd, &buf);
  if (ret) {
    log_error("Error in enq buffer");
    return Status::ERROR_UNKNOWN;
  }

  return Status::SUCCESS;
}

CameraDevice::Status CameraDeviceAtomIsp::setSize(uint32_t width,
                                                  uint32_t height) {

  if (width == 0 || height == 0) {
    log_error("Invalid Size");
    return Status::INVALID_ARGUMENT;
  }

  if (getState() == RUN) {
    log_debug("Invalid State");
    return Status::INVALID_STATE;
  }

  mOutWidth = width;
  mOutHeight = height;

  return Status::SUCCESS;
}

CameraDevice::Status CameraDeviceAtomIsp::getSize(uint32_t &width,
                                                  uint32_t &height) const {
  width = mOutWidth;
  height = mOutHeight;

  return Status::SUCCESS;
}

CameraDevice::Status
CameraDeviceAtomIsp::setPixelFormat(CameraDevice::PixelFormat format) {

  if (format <= MIN || format >= MAX) {
    log_error("Invalid Pixel format");
    return Status::INVALID_ARGUMENT;
  }

  if (getState() == RUN) {
    log_debug("Invalid State");
    return Status::INVALID_STATE;
  }

  mOutPixelFormat = format;

  return Status::SUCCESS;
}

CameraDevice::Status
CameraDeviceAtomIsp::getPixelFormat(CameraDevice::PixelFormat &format) const {
  format = mPixelFormat;

  return Status::SUCCESS;
}

CameraDevice::Status CameraDeviceAtomIsp::setMode(CameraDevice::Mode mode) {
  mMode = mode;
  return Status::SUCCESS;
}

CameraDevice::Status CameraDeviceAtomIsp::getMode(CameraDevice::Mode &mode) const {
  mode = mMode;
  return Status::SUCCESS;
}

/*
    FSM :: IDLE <--> INIT <--> RUN
*/
CameraDevice::Status CameraDeviceAtomIsp::setState(State state) {
  log_debug("%s : %d", __func__, state);

  if (mState == state)
    return Status::SUCCESS;

  if (state == ERROR) {
    mState = state;
    return Status::SUCCESS;
  }

  switch (mState) {
  case IDLE:
    if (state == INIT)
      mState = state;
    break;
  case INIT:
    if (state == IDLE || state == RUN)
      mState = state;
    break;
  case RUN:
    if (state == INIT)
      mState = state;
    break;
  case ERROR:
    log_error("In Error State");
    // Free up resources, restart?
    break;
  default:
    break;
  }

  if (mState != state) {
    log_error("InValid State Transition");
    return Status::ERROR_UNKNOWN;
  }

  return Status::SUCCESS;
  ;
}

CameraDevice::State CameraDeviceAtomIsp::getState() const { return mState; }

void CameraDeviceAtomIsp::uyvy2mono8(const uint8_t *UYVY, uint8_t *MONO,
                                     int width, int height, int stride) {
  int i, j;
  int k = 0;
  for (j = 0; j < height; j++) {
    for (i = 0; i < width; i++) {
      MONO[k++] = UYVY[(j * 2 * stride) + (2 * i) + 1];
    }
  }
}

// Crop and convert pixel format
// TODO :: Add support for more transormations
void CameraDeviceAtomIsp::transform(const uint8_t *input, uint8_t *output) {
  if (mPixelFormat == UYVY &&
      mOutPixelFormat == GREY) {
    uyvy2mono8(input, output, mOutWidth, mOutHeight, mWidth);
  }
}

int CameraDeviceAtomIsp::allocFrameBuffer(int bufCnt, size_t bufSize) {
  log_debug("%s count:%d", __func__, bufCnt);

  int ret = -1;

  // Check for valid input
  if (!bufCnt || !bufSize)
    return ret;

  int pageSize = getpagesize();
  size_t bufLen = (bufSize + pageSize - 1) & ~(pageSize - 1);
  log_debug("pagesize=%i buffer_len=%li", pageSize, bufLen);

  mFrmBuf = (void **)calloc(bufCnt, sizeof(void *));
  for (int i = 0; i < bufCnt; i++) {
    mFrmBuf[i] = memalign(pageSize, bufLen);
    if (!mFrmBuf[i]) {
      log_error("memalign failure");
      while (i) {
        i--;
        free(mFrmBuf[i]);
        mFrmBuf[i] = NULL;
      }
      break;
    }

    ret = 0;
  }

  mBufLen = bufLen;
  log_debug("%s Exit", __func__);
  return ret;
}

int CameraDeviceAtomIsp::freeFrameBuffer() {
  log_debug("%s", __func__);

  if (!mFrmBuf)
    return 0;

  for (uint8_t i = 0; i < mBufCnt; i++) {
    free(mFrmBuf[i]);
  }
  free(mFrmBuf);
  mFrmBuf = NULL;

  return 0;
}

int CameraDeviceAtomIsp::pollCamera(int fd) {
  int ret = -1;
  while (mState == RUN) {
    fd_set fds;
    struct timeval tv;
    int r;

    FD_ZERO(&fds);
    FD_SET(fd, &fds);

    /* Timeout. */
    tv.tv_sec = 2;
    tv.tv_usec = 0;

    r = select(fd + 1, &fds, NULL, NULL, &tv);

    if (-1 == r) {
      if (EINTR == errno)
        continue;
      log_error("select");
      ret = -1;
      break;
    }

    if (0 == r) {
      log_error("select timeout");
      stop();
      uninit();
      init();
      start();
      continue;
    }

    ret = 0;
    break;
  }

  return ret;
}
