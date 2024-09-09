#
# Copyright (c) 2024 Springer Design, Inc.
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(peripheral_spo2)

# SDI REF OX START
target_sources(app PRIVATE
  src/main.c
  src/PPGRecorder.c
  src/DataFrame.c
  src/battery.c
)
# SDI REF OX END

#zephyr_library_include_directories(.)
zephyr_library_include_directories(${ZEPHYR_BASE}/samples/bluetooth)

# Create a wrapper CMake library that our app can link with
set(maxlib_src_dir   ${CMAKE_CURRENT_SOURCE_DIR}/maxlib)

set(MAXLIB_LIB_DIR     ${maxlib_src_dir}/lib)
#set(MAXLIB_INCLUDE_DIR ${maxlib_src_dir}/include)

add_library(max86171lib STATIC IMPORTED GLOBAL)
#add_dependencies(
#  max86171lib
#  )
set_target_properties(max86171lib PROPERTIES IMPORTED_LOCATION             ${MAXLIB_LIB_DIR}/max86171lib.a)
#set_target_properties(max86171lib PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${MAXLIB_INCLUDE_DIR})

target_link_libraries(app PUBLIC max86171lib)
