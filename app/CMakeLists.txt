#-------------------------------------------------------------------------------
# Zephyr Example Application
#
# Copyright (c) 2022 François Gervais
# SPDX-License-Identifier: Apache-2.0

list(APPEND OVERLAY_CONFIG "thread-med.conf")
list(APPEND OVERLAY_CONFIG "secret.conf")
list(APPEND OVERLAY_CONFIG "dns.conf")
list(APPEND OVERLAY_CONFIG "mqtt.conf")

cmake_minimum_required(VERSION 3.20.0)
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(app LANGUAGES C VERSION 1.0.0)

configure_file(app_version.h.in ${CMAKE_BINARY_DIR}/app/include/app_version.h)
target_include_directories(app PRIVATE ${CMAKE_BINARY_DIR}/app/include src)

target_sources(app PRIVATE src/drv_ir.c)
target_sources(app PRIVATE src/main.c)

zephyr_include_directories(
 configuration/${BOARD}
 )
