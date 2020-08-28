configure_file(datapath.h.in "${CMAKE_BINARY_DIR}/include/datapath.h")
include_directories("${CMAKE_BINARY_DIR}/include")

set(controller_SRC
  mc_hri1_controller.cpp
  min_jerk_traj.cpp
)

set(controller_HDR
  mc_hri1_controller.h
  min_jerk_traj.h
)

add_library(${PROJECT_NAME} SHARED ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DMC_CONTROL_EXPORTS" PREFIX "")
pkg_config_use_dependency(${PROJECT_NAME} mc_rtc)
install(TARGETS ${PROJECT_NAME} DESTINATION ${MC_RTC_LIBDIR}/mc_controller)
