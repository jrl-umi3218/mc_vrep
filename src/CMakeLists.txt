#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

set(mc_vrep_src
  vrep_simulation.cpp
  vrep_cli.cpp
  mc_vrep.cpp
)

add_executable(mc_vrep ${mc_vrep_src})
target_link_libraries(mc_vrep PUBLIC vrep-api-wrapper::vrep-api-wrapper mc_rtc::mc_control)

install(TARGETS mc_vrep
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

