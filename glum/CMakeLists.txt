# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(RealsenseExamplesAlign)

if(BUILD_GRAPHICAL_EXAMPLES)
    add_executable(rs-glum rs-glum.cpp find_brick.cpp ../../third-party/imgui/imgui.cpp ../../third-party/imgui/imgui_draw.cpp ../../third-party/imgui/imgui_impl_glfw.cpp)
    set_property(TARGET rs-glum PROPERTY CXX_STANDARD 11)
    target_link_libraries(rs-glum ${DEPENDENCIES})
    include_directories(../../common ../../third-party/imgui ../../examples)
    set_target_properties (rs-glum PROPERTIES FOLDER Examples)
    install(TARGETS rs-glum RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
