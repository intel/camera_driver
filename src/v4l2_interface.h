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
#include <stdint.h>
#include <stdio.h>
#include <string>
#include <vector>

#define V4L2_DEVICE_PATH "/dev/"
#define V4L2_VIDEO_PREFIX "video"

#define CLEAR(x) memset(&(x), 0, sizeof(x))

int v4l2_ioctl(int fd, int request, void *arg);
int v4l2_list_devices(std::vector<std::string> &devList);
int v4l2_open(const char *devicepath);
int v4l2_close(int fd);
int v4l2_query_cap(int fd);
int v4l2_query_control(int fd);
int v4l2_query_framesizes(int fd);
int v4l2_get_control(int fd, int ctrl_id);
int v4l2_set_control(int fd, int ctrl_id, int value);
int v4l2_set_input(int fd, int id);
int v4l2_get_input(int fd);
int v4l2_set_capturemode(int fd, uint32_t mode);
int v4l2_get_param(int fd);
int v4l2_set_pixformat(int fd, uint32_t w, uint32_t h, uint32_t pf);
int v4l2_get_format(int fd);
int v4l2_streamon(int fd);
int v4l2_streamoff(int fd);
int v4l2_buf_req(int fd, uint32_t count);
int v4l2_buf_q(int fd, struct v4l2_buffer *pbuf);
int v4l2_buf_q(int fd, uint32_t i, unsigned long bufptr, uint32_t buflen);
int v4l2_buf_dq(int fd, struct v4l2_buffer *pbuf);
int v4l2_buf_dq(int fd);
int v4l2_get_control(int fd, int ctrl_id);
int v4l2_set_control(int fd, int ctrl_id, int value);
