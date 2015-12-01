# Needed by VREP API
add_definitions(-DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255)

set(vrep_remote_api_src
  vrep_remote_api/extApi.c
  vrep_remote_api/extApiPlatform.c
)

set(vrep_remote_api_hdr
  vrep_remote_api/extApi.h
  vrep_remote_api/extApiInternal.h
  vrep_remote_api/extApiPlatform.h
  vrep_remote_api/v_repConst.h
)

add_library(vrep_remote_api SHARED ${vrep_remote_api_hdr} ${vrep_remote_api_src})

add_definitions("-std=c++0x")

set(mc_vrep_src
  vrep_remote_api_wrapper.cpp
  mc_vrep.cpp
)

add_executable(mc_vrep ${mc_vrep_src})
target_link_libraries(mc_vrep vrep_remote_api pthread)

install(TARGETS vrep_remote_api mc_vrep
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

