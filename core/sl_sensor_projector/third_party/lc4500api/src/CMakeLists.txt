find_path(LIBUSB_INCLUDE_DIR
  NAMES libusb.h
  PATH_SUFFIXES "include" "libusb" "libusb-1.0")

find_package(PkgConfig REQUIRED)
pkg_check_modules(libusb REQUIRED libusb-1.0)
pkg_check_modules(libudev REQUIRED libudev)

set(HEADER_LIST 
"${lc4500api_SOURCE_DIR}/include/dlpc350_api.h"
"${lc4500api_SOURCE_DIR}/include/dlpc350_BMPParser.h"
"${lc4500api_SOURCE_DIR}/include/dlpc350_common.h" 
"${lc4500api_SOURCE_DIR}/include/dlpc350_error.h" 
"${lc4500api_SOURCE_DIR}/include/dlpc350_firmware.h"
"${lc4500api_SOURCE_DIR}/include/dlpc350_flashDevice.h"
"${lc4500api_SOURCE_DIR}/include/dlpc350_usb.h"
"${lc4500api_SOURCE_DIR}/include/dlpc350_version.h"
"${lc4500api_SOURCE_DIR}/include/Helper.h"
"${lc4500api_SOURCE_DIR}/include/hidapi.h"
"${lc4500api_SOURCE_DIR}/include/hidapi.hpp"
)

# Make an automatic library - will be static or dynamic based on user setting
add_library(lc4500api SHARED
dlpc350_api.cpp
dlpc350_BMPParser.cpp
dlpc350_common.cpp
dlpc350_firmware.cpp
dlpc350_usb.cpp
hid.Libusb.c
${HEADER_LIST})

# We need this directory, and users of our library will need it too
target_include_directories(lc4500api PUBLIC "${lc4500api_SOURCE_DIR}/include")
target_include_directories(lc4500api PUBLIC ${LIBUSB_INCLUDE_DIR})
target_link_libraries(lc4500api PUBLIC usb-1.0)
target_link_libraries(lc4500api PUBLIC udev)


