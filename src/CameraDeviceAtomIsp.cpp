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

#include <algorithm>
#include <iostream>
#include <linux/videodev2.h>
#include <malloc.h>
#include <string.h>
#include <unistd.h>

#include "CameraDeviceAtomIsp.h"
#include "v4l2_interface.h"

CameraDeviceAtomIsp::CameraDeviceAtomIsp(std::string device)
    : mDeviceId(device), mWidth(640), mHeight(480), mPixelFormat(-1) {
  log_debug("%s path:%s", __func__, mDeviceId.c_str());
}

CameraDeviceAtomIsp::~CameraDeviceAtomIsp() {
  stop();
  uninit();
}

std::string CameraDeviceAtomIsp::getDeviceId() { return mDeviceId; }

int CameraDeviceAtomIsp::getInfo() { return 0; }

bool CameraDeviceAtomIsp::isGstV4l2Src() { return false; }

int CameraDeviceAtomIsp::init() {
  log_debug("%s", __func__);

  int ret = -1;

  // Open Camera device
  mFd = v4l2_open(mDeviceId.c_str());
  if (mFd == -1) {
    log_error("Error in opening camera device");
    return ret;
  }

  // Set Device ID
  ret = v4l2_set_input(mFd, 1);

  // Query Capabilities
  ret = v4l2_query_cap(mFd);

  // Set Parameter - preview mode
  ret = v4l2_set_capturemode(mFd, 0x8000);

  // Set Image Format
  // TODO:: Check if yuv format or gray format
  // ret = v4l2_set_pixformat(mFd, 640, 480, V4L2_PIX_FMT_YUV420);
  ret = v4l2_set_pixformat(mFd, 640, 480, V4L2_PIX_FMT_UYVY);

  // Allocate User buffer
  // TODO:: Remove hardcoding for the buffer count and size
  ret = allocFrameBuffer(4, 640 * 480 * 2);

  return ret;
}

int CameraDeviceAtomIsp::uninit() {
  log_debug("%s", __func__);

  // Close the Camera Device
  v4l2_close(mFd);

  // Free User buffer
  freeFrameBuffer();

  mState = STATE_IDLE;
  return 0;
}

#if 0
int CameraDeviceAtomIsp::start(std::function<void(std::vector<uint8_t>)> cb)
{
    mFrameCB = cb;

    start();
}
#endif

int CameraDeviceAtomIsp::start(std::function<void(void *, size_t)> cb) {
  log_debug("%s", __func__);

  mFrameCB2 = cb;

  start();
}

int CameraDeviceAtomIsp::start() {
  log_debug("%s", __func__);
  int ret = -1;

  // request buffer
  ret = v4l2_buf_req(mFd, 4);

  struct v4l2_buffer buf;
  CLEAR(buf);
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_USERPTR;
  for (uint8_t i = 0; i < 4; i++) {
    buf.index = i;
    buf.m.userptr = (unsigned long)buffers[i];
    buf.length = bufLen;
    v4l2_buf_q(mFd, &buf);
  }

  v4l2_streamon(mFd);

  mState = STATE_RUN;

  // Create a camera thread to read frames
  mThread = std::thread(&CameraDeviceAtomIsp::cameraThread, this, mFd);
  return ret;
}

int CameraDeviceAtomIsp::stop() {
  log_debug("%s", __func__);

  // signal camera thread
  mState = STATE_INIT;

  // join thread
  if (mThread.joinable())
    mThread.join();

  v4l2_streamoff(mFd);

  return 0;
}

int CameraDeviceAtomIsp::allocFrameBuffer(int bufCnt, size_t bufSize) {
  log_debug("%s", __func__);

  int ret = -1;

  // Check for valid input
  if (!bufCnt || !bufSize)
    return ret;

  int pageSize = getpagesize();
  size_t bufLen = (bufSize + pageSize - 1) & ~(pageSize - 1);
  log_debug("pagesize=%i buffer_len=%li", pageSize, bufLen);

  for (int i = 0; i < bufCnt; i++) {
    log_debug("before memalign");
    buffers[i] = memalign(pageSize, bufLen);
    log_debug("after memalign");
    if (!buffers[i]) {
      log_error("memalign failure");
      while (i) {
        i--;
        free(buffers[i]);
        buffers[i] = NULL;
      }
      break;
    }
    ret = 0;
  }

  log_debug("%s Exit", __func__);
  return ret;
}

int CameraDeviceAtomIsp::freeFrameBuffer() {
  log_debug("%s", __func__);

  for (uint8_t i = 0; i < 4; i++) {
    free(buffers[i]);
    buffers[i] = NULL;
  }

  return 0;
}

std::vector<uint8_t> CameraDeviceAtomIsp::read() {
  std::lock_guard<std::mutex> locker(mLock);
  return mFrameBuffer;
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

int CameraDeviceAtomIsp::read_frame(void) {
  log_debug("%s", __func__);

  int ret = -1;
  struct v4l2_buffer buf;
  CLEAR(buf);

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
    return ret;
  }

  log_error("Buffer size:%d used:%d", buf.length, buf.bytesused);

  // Check if buffer valid

  // memcpy the buffer
  // void *frame = malloc(buf.bytesused);
  // memcpy(frame, (void *)buf.m.userptr, buf.bytesused);
  // std::vector<uint8_t> frame;
  // frame.reserve(buf.bytesused);
  // std::copy((uint8_t *)buf.m.userptr, (uint8_t *)buf.m.userptr +
  // buf.bytesused, std::back_inserter(frame));
  // mFrameBuffer = std::vector<uint8_t>(buf.m.userptr, buf.m.userptr +
  // buf.bytesused);

  // pass the buffer to
  mFrameCB2((void *)buf.m.userptr, buf.bytesused);

  // queue buffer for refill
  ret = v4l2_buf_q(mFd, &buf);
  if (ret) {
    log_error("Error in enq buffer");
    return ret;
  }

  return ret;
}

void CameraDeviceAtomIsp::cameraThread(int fd) {
  log_debug("cameraThread");

  // set streaming ON

  while (mState == STATE_RUN) {
    for (;;) {
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
        break;
      }

      if (0 == r) {
        log_error("select timeout");
        v4l2_streamoff(mFd);
        uninit();
        init();
        start();
        continue;
        // break;
      }

      if (read_frame())
        break;
      /* EAGAIN - continue select loop. */
    }
  }
}
