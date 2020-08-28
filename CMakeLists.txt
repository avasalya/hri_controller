cmake_minimum_required(VERSION 2.8)

include(cmake/base.cmake)
include(cmake/boost.cmake)

set(PROJECT_NAME mc_hri1_controller)
set(PROJECT_DESCRIPTION "hri1 controller for mc_rtc")
set(PROJECT_URL https://gite.lirmm.fr/multi-contact/mc_hri1_controller)

setup_project()

# Enable C++11
if(NOT WIN32)
  add_definitions(-std=c++0x)
endif()

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/launch/hri1.launch.in
               ${CMAKE_CURRENT_SOURCE_DIR}/launch/hri1.launch)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/launch/hri1_control.launch.in
               ${CMAKE_CURRENT_SOURCE_DIR}/launch/hri1_control.launch)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/launch/hri1_display.launch.in
               ${CMAKE_CURRENT_SOURCE_DIR}/launch/hri1_display.launch)

# Use jrl-cmakemoduels to get mc_rtc depency
set(PKG_CONFIG_ADDITIONAL_VARIABLES "${PKG_CONFIG_ADDITIONAL_VARIABLES} libdir")
add_required_dependency(mc_rtc)

add_subdirectory(src)

setup_project_finalize()
