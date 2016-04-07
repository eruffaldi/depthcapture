/**
 * ZED reader viewer and recorder
 */
#include <stdio.h>
#include <opencv/highgui.h>

#include <thread>
#include <iostream>
#include <fstream>
#include "pooledchannel.hpp"
#include "x264encoder.h"

#include <librealsense/rs.hpp>

#include <cstdio>
#include <stdint.h>
#include <vector>
#include <limits>
#include <iostream>

 struct uvc_frame_t
 {
  int width,height;
  int step;
  uint8_t * data = 0;
 };
 void uvc_free_frame(uvc_frame_t * ) { delete[] (uint8_t*) uvc_free_frame; }

 uvc_frame_t * uvc_allocate_frame(int n) {  
    uint8_t *p = new uint8_t[sizeof(uvc_frame_t)+n];

    auto r = (uvc_frame_t*)p;
    r->data = p+sizeof(uvc_frame_t);
    return r;
 }

/// handles a libuvc frame
class FrameHandler
{
public:

  /// custom encoder that stores NALS to file
  class x264EncoderX : public x264Encoder 
  {
  public:
    std::ofstream * ponf = 0;
    void handlenals(int frame_size,x264_nal_t * p,int n) override
    {
      if(!ponf) return;

      for(int i = 0; i < n; i++)
          ponf->write((const char*)p[i].p_payload, p[i].i_payload);
    }
  } encoder;

  /// wrapper of uvc_frame in C++
  struct uvc_frame_t_wrap
  {
      uvc_frame_t * p = 0;
      ~uvc_frame_t_wrap () { if(p) uvc_free_frame(p); }
  };

  PooledChannel<uvc_frame_t_wrap> channel;
  std::ofstream onf;

  FrameHandler() : onf("out.x264",std::ios::binary) ,channel(5,true,true){}

  /// handles one file from the lambda
  void operator()(uvc_frame_t * frame)
  {
    std::cout << "." << std::endl;
    if(!encoder)
    {
          // TODO: IS BGR

        encoder.initialize(frame->width,frame->height, x264Encoder::InputFormat::RGB,4);
        encoder.ponf = &onf;
    } 
#if 0
    // every some frames pushes to OpenCV
    // TODO: OpenMP could split this in two
    if(++frames % 3 == 0)
    {
        uvc_frame_t_wrap * pp = channel.writerGet(false);
        if(!pp)
          --frames;
        else
        {
          if(!pp->p)
              pp->p = uvc_allocate_frame(frame->width * frame->height * 3);
          // we want to release frame
          //uvc_any2bgr(frame, pp->p);
          channel.writerDone(pp);
        }
    } 
#endif
    /// encoding and storage
    encoder.encodeFrame((uint8_t*)frame->data,frame->step);
  }

  int frames = 0;
};


int main(int argc, char **argv) 
{
    rs::log_to_console(rs::log_severity::warn);
    //rs::log_to_file(rs::log_severity::debug, "librealsense.log");
    FrameHandler fh;
    rs::context ctx;
    printf("There are %d connected RealSense devices.\n", ctx.get_device_count());
    if(ctx.get_device_count() == 0) return EXIT_FAILURE;

    rs::device * dev = ctx.get_device(0);
    printf("\nUsing device 0, an %s\n", dev->get_name());
    printf("    Serial number: %s\n", dev->get_serial());
    printf("    Firmware version: %s\n", dev->get_firmware_version());

    // TODO: RS_FORMAT_YUYV
    // Configure depth to run at VGA resolution at 30 frames per second
    dev->enable_stream(rs::stream::depth, rs::preset::best_quality); //640, 480, rs::format::z16, 30);
    dev->enable_stream(rs::stream::color, rs::preset::best_quality); //640, 480, rs::format::rgb8, 30);
    //dev->enable_stream(rs::stream::infrared, 640, 480, rs::format::y8, 30);
    std::cout << "best color format is "  << dev->get_stream_format(rs::stream::color) << std::endl;

    const int width = 640;
    const int height = 480;

    dev->start();
    for (int i = 0; i < 30; ++i) dev->wait_for_frames();

    // send it to another THREAD
    rs::intrinsics depth_intrin = dev->get_stream_intrinsics(rs::stream::depth);
    rs::extrinsics depth_to_color = dev->get_extrinsics(rs::stream::depth, rs::stream::color);
    rs::intrinsics color_intrin = dev->get_stream_intrinsics(rs::stream::color);

    while(true)
    {
      dev->wait_for_frames();
      const uint16_t * depth_frame = reinterpret_cast<const uint16_t *>(dev->get_frame_data(rs::stream::depth));
      const uint8_t * color_frame = reinterpret_cast<const uint8_t *>(dev->get_frame_data(rs::stream::color));

      uvc_frame_t f;
      f.width = color_intrin.width;
      f.height = color_intrin.height;
      f.step = 3*f.width;
      f.data = (uint8_t*)color_frame;
      fh(&f);


      // TODO: IS BGR
                        auto cvImg = cvCreateImageHeader(cvSize(color_intrin.width,color_intrin.height), IPL_DEPTH_8U,3);

              cvSetData(cvImg, (void*)color_frame, color_intrin.width * 3); 
              cvNamedWindow("Test", CV_WINDOW_AUTOSIZE);
              cvShowImage("Test", cvImg);
              int q = cvWaitKey(10);
              if(q != -1)
                break;
              std::cout << "out " << q << std::endl;

      /*
      if(depth_value == 0) continue;

                // Map from pixel coordinates in the depth image to pixel coordinates in the color image
                rs::float2 depth_pixel = {(float)dx, (float)dy};
                rs::float3 depth_point = depth_intrin.deproject(depth_pixel, depth_in_meters);
                rs::float3 color_point = depth_to_color.transform(depth_point);
                rs::float2 color_pixel = color_intrin.project(color_point);
      //const uint8_t * ir_frame = reinterpret_cast<const uint8_t *>(dev->get_frame_data(rs::stream::infrared));
      std::vector<uint8_t> coloredDepth(width * height * 3);
      normalize_depth_to_rgb(coloredDepth.data(), depth_frame, width, height);
      */
      // store color
      // then image someway
    }
    dev->stop();

/*
              fh.channel.readerDone(pp);
              cvReleaseImageHeader(&cvImg);
            */
  return 0;
}

