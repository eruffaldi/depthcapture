include(cmakego)

set(CMAKEGO_BOOST program_options)
usepackage(libusb x264 boost opencv ffmpeg REQUIRED)

add_library(p::libuvc INTERFACE IMPORTED)
set_property(TARGET p::libuvc PROPERTY INTERFACE_INCLUDE_DIRECTORIES "${LIBUVC_INCLUDE_DIR}")
set_property(TARGET p::libuvc PROPERTY INTERFACE_LINK_LIBRARIES ${LIBUVC_LIBRARIES} p::libusb)

set(FLIF_INCLUDE_DIR /Users/eruffaldi/variousbuilds/FLIF/src)
set(FLIF_LIBRARIES /Users/eruffaldi/variousbuilds/FLIF/src/libflif.dylib)
add_library(p::flif INTERFACE IMPORTED)
set_property(TARGET p::flif PROPERTY INTERFACE_INCLUDE_DIRECTORIES "${FLIF_INCLUDE_DIR}")
set_property(TARGET p::flif PROPERTY INTERFACE_LINK_LIBRARIES ${FLIF_LIBRARIES})

add_library(p::librealsense INTERFACE IMPORTED)
set_property(TARGET p::librealsense PROPERTY INTERFACE_INCLUDE_DIRECTORIES "${REALSENSE_INCLUDE_DIR}")
set_property(TARGET p::librealsense PROPERTY INTERFACE_LINK_LIBRARIES /Users/eruffaldi/variousbuilds/librealsense/librealsense.dylib ${REALSENSE_LIBRARIES} p::libusb)

set(CMAKE_CXX_FLAGS --std=c++11)
add_executable(recordzed recordzed.cpp x264encoder.cpp lodepng.cpp)
target_link_libraries(recordzed p::x264 p::libuvc p::boost p::boost p::opencv p::ffmpeg::swscale)

add_executable(recordr200 recordr200.cpp timing_mach.c x264encoder.cpp  lodepng.cpp)
target_link_libraries(recordr200 p::x264 p::librealsense p::boost p::opencv p::ffmpeg::swscale)
