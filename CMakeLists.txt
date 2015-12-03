cmake_minimum_required(VERSION 2.8)

include(cmake/base.cmake)
include(cmake/eigen.cmake)

# These variables have to be defined before running setup_project
set(PROJECT_NAME mc_vrep)
set(PROJECT_DESCRIPTION "A VREP interface for QP controller")
set(PROJECT_URL https://gite.lirmm.fr/multi-contact/mc_vrep)

set(CXX_DISABLE_WERROR 1)
setup_project()
enable_language(C)

search_for_eigen()

add_required_dependency(mc_rtc)

add_subdirectory(src)

# This should be called at the end
setup_project_finalize()
