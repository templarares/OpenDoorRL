set(controller_SRC
  OpenDoorRL.cpp
)

set(controller_HDR
  OpenDoorRL.h
)

add_library(${PROJECT_NAME} SHARED ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DOpenDoorRL_EXPORTS")
target_link_libraries(${PROJECT_NAME} PUBLIC mc_rtc::mc_control_fsm)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION "${MC_RTC_LIBDIR}"
  LIBRARY DESTINATION "${MC_RTC_LIBDIR}"
  RUNTIME DESTINATION bin)

add_controller(${PROJECT_NAME}_controller lib.cpp "")
target_link_libraries(${PROJECT_NAME}_controller PUBLIC ${PROJECT_NAME})

add_subdirectory(states)

pybind11_add_module(mc_rtc_rl python_module.cpp)
target_link_libraries(mc_rtc_rl PUBLIC ${PROJECT_NAME} mc_mujoco::mc_mujoco)
set_target_properties(mc_rtc_rl PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/python)