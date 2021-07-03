cmake_minimum_required(VERSION 2.8.3)
project(sl_sensor_codec)
find_package(OpenCV REQUIRED)

find_package(catkin_simple REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-Wall -Wextra)
set (CMAKE_CXX_STANDARD 17)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/phase_shift_utilities.cpp
  src/codec_common.cpp
  src/encoder.cpp
  src/decoder.cpp
  src/decoder_nodelet.cpp
  src/phase_shift_with_tpu_encoder.cpp
  src/phase_shift_with_tpu_decoder.cpp
  )
  
target_include_directories(${PROJECT_NAME} PUBLIC "/include")
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

##############
# EXECUTABLES #
##############

cs_add_executable(generate_patterns_node src/generate_patterns_node.cpp)
target_link_libraries(generate_patterns_node ${PROJECT_NAME} ${OpenCV_LIBS} yaml-cpp)

##########
# EXPORT #
##########
cs_install()
cs_export()
