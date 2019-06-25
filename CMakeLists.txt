#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

cmake_minimum_required(VERSION 3.1)

# These variables have to be defined before running setup_project
set(PROJECT_NAME mc_vrep)
set(PROJECT_DESCRIPTION "A VREP interface for QP controller")
set(PROJECT_URL https://gite.lirmm.fr/multi-contact/mc_vrep)

set(CMAKE_CXX_STANDARD 11)

enable_language(C)

project(${PROJECT_NAME})

find_package(vrep-api-wrapper REQUIRED)
find_package(mc_rtc REQUIRED)

add_subdirectory(src)
