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
    : mDeviceId(device), mWidth(DEFAULT_WIDTH), mHeight(DEFAULT_HEIGHT),
      mPixelFormat(CameraDevice::PIXEL_FORMAT_UYVY), mState(STATE_IDLE),
      mBufCnt(DEFAULT_BUFFER_COUNT) {
  log_debug("%s path:%s", __func__, mDeviceId.c_str());
}

CameraDeviceAtomIsp::~CameraDeviceAtomIsp() {
  stop();
  uninit();
}

std::string CameraDeviceAtomIsp::getDeviceId() { return mDeviceId; }

int CameraDeviceAtomIsp::getInfo() { return 0; }

std::string CameraDeviceAtomIsp::getGstSrc() { return {}; }

int CameraDeviceAtomIsp::init() {
  log_debug("%s", __func__);

  int ret = -1;

  if (getState() != STATE_IDLE)
    return -1;

  // Open Camera device
  mFd = v4l2_open(mDeviceId.c_str());
  if (mFd < 0) {
    log_error("Error in opening camera device");
    return ret;
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

  setState(STATE_INIT);
  return 0;
error:
  v4l2_close(mFd);
  return ret;
}

int CameraDeviceAtomIsp::uninit() {
  log_debug("%s", __func__);

  if (getState() == STATE_IDLE)
    return 0;

  // Close the Camera Device
  v4l2_close(mFd);
  mFd = -1;

  // Free User buffer
  freeFrameBuffer();

  setState(STATE_IDLE);
  return 0;
}

int CameraDeviceAtomIsp::start() {
  log_debug("%s", __func__);

  int ret = 0;

  if (getState() != STATE_INIT)
    return -1;

  // request buffer
  ret = v4l2_buf_req(mFd, 4);
  if (ret)
    return ret;

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
      return ret;
  }

  ret = v4l2_streamon(mFd);
  if (ret)
    return ret;

  setState(STATE_RUN);

  return ret;
}

int CameraDeviceAtomIsp::stop() {
  log_debug("%s", __func__);

  if (getState() != STATE_RUN)
    return -1;

  v4l2_streamoff(mFd);

  setState(STATE_INIT);

  return 0;
}

int CameraDeviceAtomIsp::read(CameraFrame &frame) {
  std::lock_guard<std::mutex> locker(mLock);

  // log_debug("%s", __func__);

  if (getState() != STATE_RUN)
    return -1;

  if (pollCamera(mFd))
    return -1;

  int ret = 0;
  struct v4l2_buffer buf;
  CLEAR(buf);

  frame.width = mWidth;
  frame.height = mHeight;
  frame.stride = mWidth * 2;
  frame.pixFmt = mPixelFormat;

  // dequeue buffer
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_USERPTR;
  ret = v4l2_buf_dq(mFd, &buf);
  if (ret) {
    log_error("Error in dq buffer");
    return ret;
  }

  if (!buf.m.userptr) {
    log_error("Null buffer returned");
    return -1;
  }

  // log_debug("Buffer size:%d used:%d", buf.length, buf.bytesused);

  // TODO :: Check if buffer valid

  // pass the buffer to caller
  frame.buf = (void *)buf.m.userptr;
  frame.bufSize = buf.bytesused;

  // queue buffer for refill
  ret = v4l2_buf_q(mFd, &buf);
  if (ret) {
    log_error("Error in enq buffer");
    return ret;
  }

  return 0;
}

/*
    FSM :: IDLE <--> INIT <--> RUN
*/
int CameraDeviceAtomIsp::setState(int state) {
  int ret = 0;
  log_debug("%s : %d", __func__, state);

  if (mState == state)
    return 0;

  if (state == STATE_ERROR) {
    mState = state;
    return 0;
  }

  switch (mState) {
  case STATE_IDLE:
    if (state == STATE_INIT)
      mState = state;
    break;
  case STATE_INIT:
    if (state == STATE_IDLE || state == STATE_RUN)
      mState = state;
    break;
  case STATE_RUN:
    if (state == STATE_INIT)
      mState = state;
    break;
  case STATE_ERROR:
    log_error("In Error State");
    // Free up resources, restart?
    break;
  default:
    break;
  }

  if (mState != state) {
    ret = -1;
    log_error("InValid State Transition");
  }

  return ret;
}

int CameraDeviceAtomIsp::getState() { return mState; }

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

int CameraDeviceAtomIsp::getSize(uint32_t &width, uint32_t &height) {
  width = mWidth;
  height = mHeight;

  return 0;
}

int CameraDeviceAtomIsp::getPixelFormat(uint32_t &format) {
  format = mPixelFormat;

  return 0;
}

int CameraDeviceAtomIsp::setMode(uint32_t mode) {
  mMode = mode;
  return 0;
}

int CameraDeviceAtomIsp::getMode() { return mMode; }

int CameraDeviceAtomIsp::pollCamera(int fd) {
  int ret = -1;
  while (mState == STATE_RUN) {
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
      v4l2_streamoff(mFd);
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
