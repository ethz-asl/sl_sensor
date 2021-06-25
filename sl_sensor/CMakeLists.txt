cmake_minimum_required(VERSION 2.8.3)
project(sl_sensor)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-Wall -Wextra)
set (CMAKE_CXX_STANDARD 17)

##########
# EXPORT #
##########
cs_install()
cs_export()
