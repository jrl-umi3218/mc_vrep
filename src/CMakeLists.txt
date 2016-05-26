# Needed by VREP API
add_definitions(-DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255)

set(mc_vrep_src
  vrep_simulation.cpp
  mc_vrep_cli.cpp
  mc_vrep.cpp
)

add_executable(mc_vrep ${mc_vrep_src})
if(NOT WIN32)
  target_link_libraries(mc_vrep pthread)
endif()
pkg_config_use_dependency(mc_vrep vrep-api-wrapper)
pkg_config_use_dependency(mc_vrep mc_rtc)
set_target_properties(mc_vrep PROPERTIES COMPILE_FLAGS "-std=c++0x")

install(TARGETS mc_vrep
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

