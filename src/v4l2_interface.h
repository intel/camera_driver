/*********************************************************************
 * Copyright (C) 2018 Intel Corporation
 * SPDX-License-Identifier: BSD-3-Clause
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
