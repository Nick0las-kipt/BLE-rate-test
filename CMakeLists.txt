# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)
# Re-direct the directory where the 'boards' directory is found from
# $ZEPHYR_BASE to this directory.
set(BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR})

if(NOT DEFINED BOARD)
   set(BOARD bt840f_dbg)
endif()

include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(BT-rate-test)

target_sources(app PRIVATE
  src/main.c
  src/bt.c
  src/lna.c
  src/hog.c
)

zephyr_library_include_directories(src/)