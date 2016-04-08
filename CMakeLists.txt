include(cmakego)

usepackage(libusb x264 boost opencv ffmpeg REQUIRED)

add_library(p::libuvc INTERFACE IMPORTED)
set_property(TARGET p::libuvc PROPERTY INTERFACE_INCLUDE_DIRECTORIES "${LIBUVC_INCLUDE_DIR}")
set_property(TARGET p::libuvc PROPERTY INTERFACE_LINK_LIBRARIES ${LIBUVC_LIBRARIES} p::libusb)

add_library(p::librealsense INTERFACE IMPORTED)
set_property(TARGET p::librealsense PROPERTY INTERFACE_INCLUDE_DIRECTORIES "${REALSENSE_INCLUDE_DIR}")
set_property(TARGET p::librealsense PROPERTY INTERFACE_LINK_LIBRARIES ${REALSESNE_LIBRARIES} p::libusb)

add_definitions(--std=c++11)
add_executable(recordzed recordzed.cpp x264encoder.cpp)
target_link_libraries(recordzed p::x264 p::libuvc p::boost p::boost::program_options p::opencv p::ffmpeg::swscale)

add_definitions(--std=c++11)
add_executable(recordr200 recordr200.cpp x264encoder.cpp)
target_link_libraries(recordr200 p::x264 p::librealsense p::boost p::opencv p::ffmpeg::swscale)
