/**
 * ZED reader viewer and recorder
 */
#include <stdio.h>
#include <opencv/highgui.h>

#include "libuvc/libuvc.h"
#include <thread>
#include <iostream>
#include <fstream>
#include "pooledchannel.hpp"
#include "x264encoder.h"

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
      ~uvc_frame_t_wrap () { uvc_free_frame(p); }
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
        encoder.initialize(frame->width,frame->height, x264Encoder::InputFormat::YUYV,4);
        encoder.ponf = &onf;
    } 

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
          uvc_any2bgr(frame, pp->p);
          channel.writerDone(pp);
        }
    } 
    /// encoding and storage
    encoder.encodeFrame((uint8_t*)frame->data,frame->step);
  }

  int frames = 0;
};

int main(int argc, char **argv) 
{
  uvc_context_t *ctx;
  uvc_error_t res;
  uvc_device_t *dev;
  uvc_device_handle_t *devh;
  uvc_stream_ctrl_t ctrl;
  FrameHandler fh;

  res = uvc_init(&ctx, NULL);

  if (res < 0) {
    uvc_perror(res, "uvc_init");
    return res;
  }

  puts("UVC initialized");

  res = uvc_find_device(
      ctx, &dev,
      0, 0, NULL);

  if (res < 0) {
    uvc_perror(res, "uvc_find_device");
  } else {
    puts("Device found");

    res = uvc_open(dev, &devh);

    if (res < 0) {
      uvc_perror(res, "uvc_open");
    } else {
      puts("Device opened");

      uvc_print_diag(devh, stderr);

      res = uvc_get_stream_ctrl_format_size(
          devh, &ctrl, UVC_FRAME_FORMAT_YUYV, 3840, 1080, 30
      );

      uvc_print_stream_ctrl(&ctrl, stderr);

      if (res < 0) {
        uvc_perror(res, "get_mode");
      } else {
        res = uvc_start_streaming(devh, &ctrl, [] (uvc_frame_t *frame, void *p) { (*(FrameHandler*)p)(frame); }, (void*)&fh, 0);

        if (res < 0) {
          uvc_perror(res, "start_streaming");
        } else {
          puts("Streaming for 10 seconds...");
          uvc_error_t resAEMODE = uvc_set_ae_mode(devh, 0);
          uvc_perror(resAEMODE, "set_ae_mode");
          while(true)
          {
            FrameHandler::uvc_frame_t_wrap * pp = 0;
            fh.channel.readerGet(pp);
            if(pp)
            {
                        auto cvImg = cvCreateImageHeader(
                cvSize(pp->p->width, pp->p->height),
                IPL_DEPTH_8U,
                3);

              cvSetData(cvImg, pp->p->data, pp->p->width * 3); 
              cvNamedWindow("Test", CV_WINDOW_AUTOSIZE);
              cvShowImage("Test", cvImg);
              int q = cvWaitKey(10);
              if(q != -1)
                break;
              std::cout << "out " << q << std::endl;
              fh.channel.readerDone(pp);
              cvReleaseImageHeader(&cvImg);
            }
          }
          uvc_stop_streaming(devh);
          puts("Done streaming.");
        }
      }

      uvc_close(devh);
      puts("Device closed");
    }

    uvc_unref_device(dev);
  }

  uvc_exit(ctx);
  puts("UVC exited");

  return 0;
}

