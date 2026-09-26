# µnleashed camsat: the satellite's sources, and the core's link engine and
# picture code that tools/fetch_core.py copies into ../core.
# Copyright 2026 - Robert Mech. SPDX-License-Identifier: GPL-3.0-or-later
FILE(GLOB app_sources ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
set(core_dir ${CMAKE_CURRENT_SOURCE_DIR}/../core)
idf_component_register(SRCS ${app_sources} ${core_dir}/core/link.cpp ${core_dir}/core/linkcrypto.cpp
                       INCLUDE_DIRS "." ${core_dir}/core ${core_dir}/plugins
                       REQUIRES esp_wifi esp_event esp_netif nvs_flash esp_timer esp_rom mbedtls
                                esp_driver_gpio esp_hw_support esp_psram)
# The watermark re-encodes with the camera driver's own JPEG encoder, whose
# header the driver keeps private (as the core does).
idf_component_get_property(cam_dir espressif__esp32-camera COMPONENT_DIR)
target_include_directories(${COMPONENT_LIB} PRIVATE "${cam_dir}/conversions/private_include")
