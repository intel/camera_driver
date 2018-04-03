/*
 * This file is part of the Camera Streaming Daemon project
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

#include <cerrno>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <unistd.h>

#include "v4l2_interface.h"

#define log_error(fmt, ...)                                                    \
  printf("[Error] ");                                                          \
  printf(fmt, ##__VA_ARGS__);                                                  \
  printf("\n")
#define log_debug(fmt, ...)                                                    \
  printf("[Debug] ");                                                          \
  printf(fmt, ##__VA_ARGS__);                                                  \
  printf("\n")

int xioctl(int fd, int request, void *arg) {
  int r;

  do
    r = ioctl(fd, request, arg);
  while (-1 == r && EINTR == errno);

  return r;
}

int v4l2_list_devices(std::vector<std::string> &devList) {
  DIR *dp;
  struct dirent *ep;

  dp = opendir(V4L2_DEVICE_PATH);
  if (dp == NULL) {
    log_error("Could not open directory %d", errno);
    return -1;
  }

  while ((ep = readdir(dp))) {
    if (std::strncmp(V4L2_VIDEO_PREFIX, ep->d_name,
                     sizeof(V4L2_VIDEO_PREFIX) - 1) == 0) {
      log_debug("Found V4L2 camera device %s", ep->d_name);
      // add device path to list
      devList.push_back(std::string(V4L2_DEVICE_PATH) + ep->d_name);
    }
  }

  closedir(dp);
  return 0;
}

int v4l2_open(const char *devicepath) {
  int fd = -1;
  struct stat st;
  int ret = stat(devicepath, &st);
  if (ret) {
    log_error("Unable to get device stat");
    return fd;
  }

  if (!S_ISCHR(st.st_mode)) {
    log_error("Device is not a character device");
    return fd;
  }

  // fd = open(devicepath, O_RDWR, 0);
  fd = open(devicepath, O_RDWR | O_NONBLOCK);
  if (fd < 0) {
    log_error("Cannot open device '%s': %d: ", devicepath, errno);
  }

  return fd;
}

int v4l2_close(int fd) {
  if (fd < 1)
    return -1;

  close(fd);
  return 0;
}

int v4l2_query_cap(int fd) {
  int ret = -1;
  if (fd < 1)
    return ret;

  struct v4l2_capability vcap;
  ret = xioctl(fd, VIDIOC_QUERYCAP, &vcap);
  if (ret)
    return ret;

  log_debug("\tDriver name   : %s", vcap.driver);
  log_debug("\tCard type     : %s", vcap.card);
  log_debug("\tBus info      : %s", vcap.bus_info);
  log_debug("\tDriver version: %d.%d.%d", vcap.version >> 16,
            (vcap.version >> 8) & 0xff, vcap.version & 0xff);
  log_debug("\tCapabilities  : 0x%08X", vcap.capabilities);
  if (vcap.capabilities & V4L2_CAP_DEVICE_CAPS) {
    log_debug("\tDevice Caps   : 0x%08X", vcap.device_caps);
  }
  // TODO:: Return the caps
  return 0;
}

int v4l2_query_control(int fd) {
  int ret = -1;
  if (fd < 1)
    return ret;

  const unsigned next_fl =
      V4L2_CTRL_FLAG_NEXT_CTRL | V4L2_CTRL_FLAG_NEXT_COMPOUND;
  struct v4l2_control ctrl = {0};
  struct v4l2_queryctrl qctrl = {0};
  qctrl.id = next_fl;
  while (xioctl(fd, VIDIOC_QUERYCTRL, &qctrl) == 0) {
    if ((qctrl.flags & V4L2_CTRL_TYPE_BOOLEAN) ||
        (qctrl.type & V4L2_CTRL_TYPE_INTEGER)) {
      log_debug("Ctrl: %s Min:%d Max:%d Step:%d dflt:%d", qctrl.name,
                qctrl.minimum, qctrl.maximum, qctrl.step, qctrl.default_value);
      ctrl.id = qctrl.id;
      if (xioctl(fd, VIDIOC_G_CTRL, &ctrl) == 0) {
        log_debug("Ctrl: %s Id:%x Value:%d\n", qctrl.name, ctrl.id, ctrl.value);
      }
    }
    qctrl.id |= next_fl;
  }
  // TODO:: Return the list of controls
  return 0;
}

int v4l2_query_framesizes(int fd) {
  int ret = -1;
  if (fd < 1)
    return ret;

  struct v4l2_fmtdesc fmt = {};
  struct v4l2_frmsizeenum frame_size = {};
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.index = 0;
  while (xioctl(fd, VIDIOC_ENUM_FMT, &fmt) != -1) {
    // TODO:: Save the format in some DS
    log_debug("Pixel Format: %u", fmt.pixelformat);
    log_debug("Name : %s\n", fmt.description);
    frame_size.index = 0;
    frame_size.pixel_format = fmt.pixelformat;
    while (xioctl(fd, VIDIOC_ENUM_FRAMESIZES, &frame_size) != -1) {
      switch (frame_size.type) {
      case V4L2_FRMSIZE_TYPE_DISCRETE:
        log_debug(" FrameSize: %ux%u\n", frame_size.discrete.width,
                  frame_size.discrete.height);
        break;
      case V4L2_FRMSIZE_TYPE_CONTINUOUS:
      case V4L2_FRMSIZE_TYPE_STEPWISE:
        log_debug(" FrameSize: %ux%u -> %ux%u\n", frame_size.stepwise.min_width,
                  frame_size.stepwise.min_height, frame_size.stepwise.max_width,
                  frame_size.stepwise.max_height);
        break;
      }
      // TODO::Save the framesizes against format in some DS
    }

    fmt.index++;
  }

  // TODO::Return the list of pixformats & framesizes
  return 0;
}

int v4l2_set_input(int fd, int id) {
  int ret = -1;

  // set device_id
  ret = xioctl(fd, VIDIOC_S_INPUT, (int *)&id);
  if (ret) {
    log_error("Error setting device id: %s", strerror(errno));
  }

  return ret;
}

int v4l2_get_input(int fd) {
  int ret = -1;

  return ret;
}

int v4l2_set_capturemode(int fd, uint32_t mode) {
  int ret = -1;

  // set stream parameters
  struct v4l2_streamparm parm;
  memset(&parm, 0, sizeof(struct v4l2_streamparm));
  parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  parm.parm.capture.capturemode = mode;
  ret = xioctl(fd, VIDIOC_S_PARM, &parm);
  if (ret) {
    log_error("Unable to set stream parameters: %s", strerror(errno));
  }

  return ret;
}

int v4l2_get_param(int fd) {
  int ret = -1;

  return ret;
}

int v4l2_set_pixformat(int fd, uint32_t w, uint32_t h, uint32_t pf) {
  int ret = -1;

  // set pixel format
  struct v4l2_format fmt;
  memset(&fmt, 0, sizeof(struct v4l2_format));
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = w;
  fmt.fmt.pix.height = h;
  fmt.fmt.pix.pixelformat = pf;
  fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;
  ret = xioctl(fd, VIDIOC_S_FMT, &fmt);
  if (ret) {
    log_error("Setting pixel format: %s", strerror(errno));
  }

  return ret;
}

int v4l2_get_format(int fd) {
  int ret = -1;

  return ret;
}

int v4l2_streamon(int fd) {
  int ret = -1;

  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  ret = xioctl(fd, VIDIOC_STREAMON, &type);
  if (ret) {
    log_error("Error starting streaming: %s", strerror(errno));
  }

  return ret;
}

int v4l2_streamoff(int fd) {
  int ret = -1;

  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  ret = xioctl(fd, VIDIOC_STREAMOFF, &type);
  if (ret) {
    log_error("Error stopping streaming: %s", strerror(errno));
  }

  return ret;
}

int v4l2_buf_req(int fd, uint32_t count) {
  int ret = -1;

  // Initiate I/O
  struct v4l2_requestbuffers req;
  memset(&req, 0, sizeof(struct v4l2_requestbuffers));
  req.count = count;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_USERPTR;
  ret = xioctl(fd, VIDIOC_REQBUFS, &req);
  if (ret) {
    log_error("Error in REQBUFS %s", strerror(errno));
  }

  return ret;
}

int v4l2_buf_q(int fd, uint32_t i, unsigned long bufptr, uint32_t buflen) {
  int ret = -1;

  struct v4l2_buffer buf;
  memset(&buf, 0, sizeof(struct v4l2_buffer));
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_USERPTR;
  buf.length = buflen;
  buf.index = i;
  buf.m.userptr = (unsigned long)bufptr;
  ret = xioctl(fd, VIDIOC_QBUF, &buf);
  if (ret) {
    log_error("Error giving buffers to backend: %s | i=%i", strerror(errno), i);
  }

  return ret;
}

int v4l2_buf_q(int fd, struct v4l2_buffer *pbuf) {
  int ret = -1;

  ret = xioctl(fd, VIDIOC_QBUF, pbuf);
  if (ret) {
    log_error("Error giving frame to camera: %s ", strerror(errno));
  }

  return ret;
}

int v4l2_buf_dq(int fd, struct v4l2_buffer *pbuf) {
  int ret = -1;

  ret = xioctl(fd, VIDIOC_DQBUF, pbuf);
  if (ret) {
    log_error("Error getting frame from camera: %s", strerror(errno));
  }

  return ret;
}

int v4l2_buf_dq(int fd) {
  int ret = -1;

  struct v4l2_buffer buf;
  memset(&buf, 0, sizeof(struct v4l2_buffer));
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_USERPTR;

  ret = xioctl(fd, VIDIOC_DQBUF, &buf);
  if (ret) {
    log_error("Error getting frame from camera: %s", strerror(errno));
  }

  return ret;
}

int v4l2_get_control(int fd, int ctrl_id) {
  int ret = -1;
  if (fd < 1)
    return ret;

  struct v4l2_control ctrl = {0};
  ctrl.id = ctrl_id;
  ret = xioctl(fd, VIDIOC_G_CTRL, &ctrl);
  if (ret == 0)
    return ctrl.value;
  else
    return ret;
}

int v4l2_set_control(int fd, int ctrl_id, int value) {
  int ret = -1;
  if (fd < 1)
    return ret;

  struct v4l2_control c = {0};
  c.id = ctrl_id;
  c.value = value;
  ret = xioctl(fd, VIDIOC_S_CTRL, &c);

  return ret;
}
