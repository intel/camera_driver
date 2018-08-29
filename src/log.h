/*********************************************************************
 * Copyright (C) 2018 Intel Corporation
 * SPDX-License-Identifier: BSD-3-Clause
*********************************************************************/
#pragma once

#include <cstdio>

#define log_error(fmt, ...)                                                    \
  fprintf(stderr, "[Error] ");                                                 \
  fprintf(stderr, fmt, ##__VA_ARGS__);                                         \
  fprintf(stderr, "\n")

#ifdef DEBUG
#define log_debug(fmt, ...)                                                    \
  printf(stdout, "[Debug] ");                                                  \
  printf(stdout, fmt, ##__VA_ARGS__);                                          \
  printf(stdout, "\n")
#else
#define log_debug(fmt, ...)
#endif
