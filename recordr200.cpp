/**
 * ZED reader viewer and recorder
 */
#include <opencv/highgui.h>
#include <stdio.h>

#include <sys/time.h>
#include <fstream>
#include <iostream>
#include <thread>
#include "pooledchannel.hpp"
#include "timing_mach.h"
#include "x264encoder.h"
//#define STB_IMAGE_WRITE_IMPLEMENTATION
//#include "stb_image_write.h"
#include <librealsense/rs.hpp>
#include "lodepng.h"

#include <stdint.h>
#include <cstdio>
#include <iostream>
#include <limits>
#include <vector>

bool stop;

void mySigintHandler(int sig)
{
  stop = true;
}


struct uvc_frame_t
{
  int width, height;
  int step;
  uint8_t *data = 0;
};

void uvc_free_frame(uvc_frame_t *) { delete[](uint8_t *) uvc_free_frame; }
/// wrapper of uvc_frame in C++
struct uvc_frame_t_wrap
{
  uvc_frame_t *p = 0;
  ~uvc_frame_t_wrap()
  {
    if (p) uvc_free_frame(p);
  }
};

uvc_frame_t *uvc_allocate_frame(int n)
{
  uint8_t *p = new uint8_t[sizeof(uvc_frame_t) + n];

  auto r = (uvc_frame_t *)p;
  r->data = p + sizeof(uvc_frame_t);
  return r;
}

void saveR200YAML(rs::intrinsics &&di, rs::extrinsics &dc, rs::intrinsics &ci,
                  const char *filename)
{
  // width height ... ?
}

class DepthFrameHandler
{
 public:

  void operator()(const uint16_t *d, int w, int h, int frame)
  {
    // save frame
    // save compressed size
    // save data
    char fn[23];
    sprintf(fn, "out/outd%00004d.png", frame);

    unsigned error =
        lodepng::encode(fn, (unsigned char *)d, w, h, LCT_GREY, 16);
    // stbi_write_png(fn,w,h,1,d,w*2);
  }

  void operator()(const uint8_t *d, int w, int h, int frame)
  {
    // save frame
    // save compressed size
    // save data
    char fn[23];
    sprintf(fn, "out/outd%00004d.png", frame);

    unsigned error = lodepng::encode(fn, (unsigned char *)d, w, h, LCT_GREY, 8);
    // stbi_write_png(fn,w,h,1,d,w*2);
  }};

class IrFrameHandler
{
 public:
  PooledChannel<uvc_frame_t_wrap> channel;
  std::ofstream onf;

  IrFrameHandler(const char *name = "out/outir.x264")
      : onf(name, std::ios::binary), channel(5, true, true)
  {
  }

#if 0
  /// handles one file from the lambda
  void operator()(uvc_frame_t * frame)
  {
/*
    std::cout << "." << std::endl;
    if(!encoder)
    {
        encoder.initialize(frame->width,frame->height, x264Encoder::InputFormat::RGB,4);
        encoder.ponf = &onf;
    } 
*/
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
   // encoder.encodeFrame((uint8_t*)frame->data,frame->step);
  }

  int frames = 0;
#endif

  void operator()( uint16_t *d, int w, int h, int frame)
  {
    // save frame
    // save compressed size
    // save data
    char fn[23];
    sprintf(fn, "out/outir%00004d.png", frame);
    for(int i = 0; i < w*h; i++)
    {
      d[i] = (d[i] <<8) | ((d[i] >>8) &0xFF);
    }
    unsigned error =
        lodepng::encode(fn, (unsigned char *)d, w, h, LCT_GREY, 16);
    // stbi_write_png(fn,w,h,1,d,w*2);
  }

  void operator()(const uint8_t *d, int w, int h, int frame)
  {
    // save frame
    // save compressed size
    // save data
    char fn[23];
    sprintf(fn, "out/outir%00004d.png", frame);

    unsigned error = lodepng::encode(fn, (unsigned char *)d, w, h, LCT_GREY, 8);
    // stbi_write_png(fn,w,h,1,d,w*2);
  }
};

/// handles a libuvc frame
class FrameHandler
{
 public:
  /// custom encoder that stores NALS to file
  class x264EncoderX : public x264Encoder
  {
   public:
    std::ofstream *ponf = 0;
    void handlenals(int frame_size, x264_nal_t *p, int n) override
    {
      if (!ponf) return;

      for (int i = 0; i < n; i++)
        ponf->write((const char *)p[i].p_payload, p[i].i_payload);
    }
  } encoder;

  PooledChannel<uvc_frame_t_wrap> channel;
  std::ofstream onf;

  FrameHandler(const char *name = "out/out.x264")
      : onf(name, std::ios::binary), channel(5, true, true)
  {
  }

  /// handles one file from the lambda
  void operator()(uvc_frame_t *frame)
  {
    if (!encoder)
    {
      encoder.initialize(frame->width, frame->height,
                         x264Encoder::InputFormat::RGB, 4);
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
    encoder.encodeFrame((uint8_t *)frame->data, frame->step);
  }

  int frames = 0;
};

struct TimeHandler
{
  std::ofstream onf;
  timespec lastts;
  TimeHandler() : onf("timestamp.bin", std::ios::binary) {}
  void now()
  {
    timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);  // Works on Linux and OSX
    uint64_t t[2] = {(uint64_t)ts.tv_sec, (uint64_t)ts.tv_nsec};
    onf.write((char *)t, sizeof(t));
    lastts = ts;
    std::cout << "time " << t[0] << " " << t[1] << std::endl;
  }
};

int main(int argc, char **argv)
{
/* initialize mach timing */
#ifdef __MACH__
  timing_mach_init();
#endif
  rs::log_to_console(rs::log_severity::warn);
  rs::context ctx;
  if (ctx.get_device_count() == 0)
    throw std::runtime_error("No device detected. Is it plugged in?");
  printf("There are %d connected RealSense devices.\n", ctx.get_device_count());
  std::cerr << "ctx\n";
  // rs::log_to_file(rs::log_severity::debug, "librealsense.log");
  FrameHandler fh;
  signal(SIGINT, mySigintHandler);
  DepthFrameHandler dh;
  IrFrameHandler irh;
  TimeHandler th;
  bool irmode = false;
  bool depthmode = false;
  int rate = 30;
  bool off = false;
  bool colormode = true;
  bool doauto = false;
  int width = 640;
  int irbits = 16;
  int height = 480;
  if (ctx.get_device_count() == 0) return EXIT_FAILURE;

  for (int i = 1; i < argc; i++)
  {
    if (strcmp(argv[i], "color") == 0)
    {
      irmode = false;
      depthmode = false;
    }
    else if (strcmp(argv[i], "rgbd") == 0)
    {
      irmode = false;
      depthmode = true;
    }
    else if (strcmp(argv[i], "ir") == 0)
    {
      irmode = true;
      depthmode = false;
    }
    else if (strcmp(argv[i], "ironly") == 0)
    {
      colormode = false;
      irmode = true;
      depthmode = false;
    }
    else if (strcmp(argv[i], "fast") == 0)
    {
      rate = 60;
    }
    else if(strcmp(argv[i],"off") == 0)
    {
      off = true;
      depthmode = false; 
    }
    else if(strcmp(argv[i],"auto") == 0)
      doauto = true;
    else if(strcmp(argv[i],"f200") == 0)
    {
      irmode = true;
      doauto = false;
      colormode = true;
      depthmode = true;  
      width = 640;
      height = 480;
      irbits = 16;
    }
  }

  std::cout << "staring: color:" << colormode << " depth:" << depthmode << " ir:"<<irmode << " rate:" << rate << " poweroff:" << off << std::endl;

  rs::device *dev = ctx.get_device(0);
  printf("\nUsing device 0, an %s\n", dev->get_name());
  printf("    Serial number: %s\n", dev->get_serial());
  printf("    Firmware version: %s\n", dev->get_firmware_version());

  /*
  Capturing DEPTH at 480 x 360, fov = 56.4 x 43.8, distortion = NONE
Capturing COLOR at 640 x 480, fov = 54.0 x 41.4, distortion = MODIFIED_BROWN_CONRADY
Capturing INFRARED at 480 x 360, fov = 56.4 x 43.8, distortion = NONE
Capturing INFRARED2 at 480 x 360, fov = 56.4 x 43.8, distortion = NONE
*/
rs::format depth_fmt = (rs::format)0,color_fmt=(rs::format)0,ir_fmt = (rs::format)0;
  rs::intrinsics depth_intrin;
  rs::extrinsics depth_to_color;
  rs::intrinsics ir_intrin;
  rs::intrinsics color_intrin;
  memset(&depth_intrin,0,sizeof(depth_intrin));
  memset(&depth_to_color,0,sizeof(depth_to_color));
  memset(&ir_intrin,0,sizeof(ir_intrin));
  memset(&color_intrin,0,sizeof(color_intrin));
  if(doauto)
  {

    std::vector<rs::stream> supported_streams;

    for (int i = (int)rs::capabilities::depth; i <= (int)rs::capabilities::fish_eye; i++)
        if (dev->supports((rs::capabilities)i))
            supported_streams.push_back((rs::stream)i);

      for (auto & stream : supported_streams)
          dev->enable_stream(stream, rs::preset::best_quality);

      // Compute field of view for each enabled stream
      for (auto & stream : supported_streams)
      {
          if (!dev->is_stream_enabled(stream)) continue;
          auto intrin = dev->get_stream_intrinsics(stream);
          auto fmt = dev->get_stream_format(stream);
          switch(stream)
          {
            case (rs::stream)rs::capabilities::depth:
              depth_intrin = intrin;
              depth_fmt = fmt;
              break;
            case (rs::stream)rs::capabilities::infrared:
              ir_intrin = intrin;
              ir_fmt = fmt;
              break;
            case (rs::stream)rs::capabilities::color:
              color_intrin = intrin;
              color_fmt = fmt;
              break;
          }
          std::cout << "Capturing " << stream << " at " << intrin.width << " x " << intrin.height << " format " << fmt ;
          std::cout <<  ", fov = " << intrin.hfov() << " x " << intrin.vfov() << ", distortion = " << intrin.model() << std::endl;
      }
      colormode = color_intrin.width != 0;
      depthmode = depth_intrin.width != 0;
      irmode = ir_intrin.width != 0;
      std::cout << "effective " << colormode << depthmode << irmode << std::endl;

  }
  else
  {
      // TODO: RS_FORMAT_YUYV
      // Configure depth to run at VGA resolution at 30 frames per second
      if (depthmode)
      {
        dev->enable_stream(rs::stream::depth, 640, 480, rs::format::z16, rate);
        std::cout << "depth enabled\n";
      }
      if (colormode)
      {
        dev->enable_stream(rs::stream::color, width, height, rs::format::rgb8,
                           rate);
        std::cout << "color enabled\n";
      std::cout << "best color format is "
                << dev->get_stream_format(rs::stream::color) << std::endl;

      }
      if (irmode)
      {
        dev->enable_stream(rs::stream::infrared, 640, 480, irbits == 16 ? rs::format::y16 : rs::format::y8, rate);
        std::cout << "ir enabled\n";
      }
  }
  std::cout << "starting\n";
  dev->start();
  std::cout << "started\n";
  for (int i = 0; i < 30; ++i) dev->wait_for_frames();
  std::cout << "waited 30\n";

  // send it to another THREAD
  if (depthmode) depth_intrin = dev->get_stream_intrinsics(rs::stream::depth);
  if (depthmode && colormode)
    depth_to_color = dev->get_extrinsics(rs::stream::depth, rs::stream::color);
  if (irmode) ir_intrin = dev->get_stream_intrinsics(rs::stream::infrared);
  if (colormode) color_intrin = dev->get_stream_intrinsics(rs::stream::color);
  std::cout << "extrinsics\n";


  // saveR200YAML(depth_intrin,depth_to_color,color_intrin,"out.yaml");

  if(off)
  {
    if( dev->supports_option(rs::option::r200_emitter_enabled))
    {
                int value = 0; //!dev->get_option(rs::option::r200_emitter_enabled);
                std::cout << "Setting emitter to " << value << std::endl;
                dev->set_option(rs::option::r200_emitter_enabled, value);
    }
    else if(dev->supports_option(rs::option::f200_laser_power))
    {
                int value = 0; //!dev->get_option(rs::option::r200_emitter_enabled);
                std::cout << "Setting laser to " << value << std::endl;
                dev->set_option(rs::option::f200_laser_power, value);

    }
    else if(dev->supports_option(rs::option::sr300_auto_range_enable_laser))
    {
                int value = 0; //!dev->get_option(rs::option::r200_emitter_enabled);
                std::cout << "Setting laser to " << value << std::endl;
                dev->set_option(rs::option::sr300_auto_range_enable_laser, value);
    }


  }
  int frames = 0;
  while (!stop)
  {
    dev->wait_for_frames();
    th.now();
    const uint8_t *color_frame;
    if (colormode)
    {
      color_frame = reinterpret_cast<const uint8_t *>(
          dev->get_frame_data(rs::stream::color));
      uvc_frame_t f;
      f.width = color_intrin.width;
      f.height = color_intrin.height;
      f.step = 3 * f.width;
      f.data = (uint8_t *)color_frame;
      fh(&f);
    }

    if (depthmode)
    {
      const uint16_t *depth_frame = reinterpret_cast<const uint16_t *>(
          dev->get_frame_data(rs::stream::depth));
      dh(depth_frame, depth_intrin.width, depth_intrin.height, frames);
    }
    if(irmode)
    {
      if(irbits == 16)
      {
      const uint16_t * ir_frame = reinterpret_cast<const uint16_t
       *>(dev->get_frame_data(rs::stream::infrared));
       irh((uint16_t*)ir_frame,ir_intrin.width,ir_intrin.height,frames);
     }
     else
     {
      const uint8_t *ir_frame = reinterpret_cast<const uint8_t *>(
          dev->get_frame_data(rs::stream::infrared));
      irh(ir_frame, ir_intrin.width, ir_intrin.height, frames);
    }
    }


    if (colormode)
    {
      // TODO: IS BGR
      auto cvImg = cvCreateImageHeader(
          cvSize(color_intrin.width, color_intrin.height), IPL_DEPTH_8U, 3);

      cvSetData(cvImg, (void *)color_frame, color_intrin.width * 3);
      cvNamedWindow("Test", CV_WINDOW_AUTOSIZE);
      cvShowImage("Test", cvImg);
      int q = cvWaitKey(1);
      if (q != -1) break;
    }
    frames++;
    /*
    if(depth_value == 0) continue;

              // Map from pixel coordinates in the depth image to pixel
    coordinates in the color image
              rs::float2 depth_pixel = {(float)dx, (float)dy};
              rs::float3 depth_point = depth_intrin.deproject(depth_pixel,
    depth_in_meters);
              rs::float3 color_point = depth_to_color.transform(depth_point);
              rs::float2 color_pixel = color_intrin.project(color_point);
    //const uint8_t * ir_frame = reinterpret_cast<const uint8_t
    *>(dev->get_frame_data(rs::stream::infrared));
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
