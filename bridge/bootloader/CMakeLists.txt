# The ESP-IDF project that builds only the second-stage bootloader this firmware is installed with
# (README.md beside it). The app it would link is never used.
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(hoverboard-bridge-bootloader)
