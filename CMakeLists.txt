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

IF(NOT WIN32)
  SEARCH_FOR_EIGEN()
ELSE()
  find_path(Eigen_INCLUDE_DIR NAMES signature_of_eigen3_matrix_library
    PATHS
    ${CMAKE_INSTALL_PREFIX}/include
    PATH_SUFFIXES eigen3 eigen
  )
  INCLUDE_DIRECTORIES(SYSTEM ${Eigen_INCLUDE_DIR})
ENDIF()

if(WIN32)
  # TODO: ask explicitly the user for the value of PKG_CONFIG_INCLUDE_DIR
  set( ENV{PKG_CONFIG_PATH} "$ENV{PKG_CONFIG_PATH};${PKG_CONFIG_INCLUDE_DIR}" )
ENDIF()

#if needed, set your conf path here
#set(CONF_PATH "C:/Work/Code/Joris/mc_vrep/etc")

add_required_dependency(mc_rtc)

add_subdirectory(src)

# This should be called at the end
setup_project_finalize()
