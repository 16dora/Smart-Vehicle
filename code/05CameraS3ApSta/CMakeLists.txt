cmake_minimum_required(VERSION 3.16)

# 05 固定为 S3，避免误用旧工程的 ESP32 目标生成固件。
set(IDF_TARGET "esp32s3")
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/source")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(smart_vehicle_camera_s3_apsta)
