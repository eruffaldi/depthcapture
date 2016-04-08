/**
 * ZED reader viewer and recorder
 */
#include <stdio.h>
#include <opencv/highgui.h>
#include <boost/program_options.hpp>
#include "libuvc/libuvc.h"
#include <thread>
#include <iostream>
#include <fstream>
#include "timingex.hpp"
#include "pooledchannel.hpp"
#include "x264encoder.h"

namespace po = boost::program_options;

std::string makefilename()
{
    const  char *MON[13]={"*","JAN","FEB","MAR","APR","MAY","JUN",
            "JUL","AUG","SEP","OCT","NOV","DEC"};
  char name[255];
#ifdef __WIN32__
    SYSTEMTIME Now;
    GetLocalTime(&Now);
    sprintf(name,"%d%0d%0d_%02d%02d%02d", Now.wYear, Now.wMonth,
            Now.wDay, Now.wHour, Now.wMinute, Now.wSecond);
#else
    time_t NowS;
    struct tm *Now;
    time(&NowS);
    Now = localtime(&NowS);
    sprintf(name,"%d%0d%0d_%02d%02d%02d", Now->tm_year+1900, Now->tm_mon,
            Now->tm_mday, Now->tm_hour, Now->tm_min, Now->tm_sec);

#endif
    return name;

}


/// handles a libuvc frame
class FrameHandler
{
public:
  PeriodTiming<> tf= {"frame"};
  PeriodTiming<> tc={"compression"};
  PeriodTiming<> to={"opencvtransfer"};
  int allocatedpools = 0;

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

  FrameHandler(const char * co) : onf(co,std::ios::binary) ,channel(6,true,true)
  {
    if(!onf)
    {
      std::cerr << "cannot create " << co << std::endl;
      exit(0);
    }
  }

  /// handles one file from the lambda
  void operator()(uvc_frame_t * frame)
  {
    tf.start();
    if(!encoder)
    {
        encoder.initialize(frame->width,frame->height, x264Encoder::InputFormat::YUYV,4);
        encoder.ponf = &onf;
    } 

    // every some frames pushes to OpenCV
    // TODO: OpenMP could split this in two
    if(++frames % 3 == 0)
    {
        to.start();
        uvc_frame_t_wrap * pp = channel.writerGet(false);
        if(!pp)
          --frames;
        else
        {
          if(!pp->p)
          {
              pp->p = uvc_allocate_frame(frame->width * frame->height * 3);
              allocatedpools++;
          }
          // we want to release frame
          uvc_any2bgr(frame, pp->p);
          channel.writerDone(pp);
        }
        to.stop();
    } 
    /// encoding and storage
    tc.start();
    encoder.encodeFrame((uint8_t*)frame->data,frame->step);
    tc.stop();
    tf.stop();

    if(tf.count() > 100)
    {
      dostat();
    }
  }

  void dostat()
  {
      std::cout << "-----\n\t"<<tf << "\n\t"  << tc << "\n\t" << to << std::endl;
      tf.statreset();
      tc.statreset();
      to.statreset();

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

  po::options_description desc("Record ZED");
  desc.add_options()
      ("help", "produce help message")
      ("outputfile", po::value<std::string>()->default_value("zed"+makefilename() + ".h264"), "output file")
      ("width", po::value<int>()->default_value(3840), "width")
      ("height", po::value<int>()->default_value(1080), "height")
      ("rate", po::value<int>()->default_value(30), "rate")
  ;

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);    

  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 1;
  }

  std::cout << "Starting with " << vm["width"].as<int>() << " x " << vm["height"].as<int>() << "@" << vm["rate"].as<int>() << " to " << vm["outputfile"].as<std::string>()<< std::endl;

  FrameHandler fh(vm["outputfile"].as<std::string>().c_str());

  res = uvc_init(&ctx, NULL);

  if (res < 0) 
  {
    uvc_perror(res, "uvc_init");
    return res;
  }

  puts("UVC initialized");

  res = uvc_find_device(      ctx, &dev,      0, 0, NULL);

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
          devh, &ctrl, UVC_FRAME_FORMAT_YUYV, vm["width"].as<int>(), vm["height"].as<int>(), vm["rate"].as<int>());

      uvc_print_stream_ctrl(&ctrl, stderr);

      if (res < 0) {
        uvc_perror(res, "get_mode");
      } else {
        PeriodTiming<> ta= {"streaming"};
        res = uvc_start_streaming(devh, &ctrl, [] (uvc_frame_t *frame, void *p) { (*(FrameHandler*)p)(frame); }, (void*)&fh, 0);

        if (res < 0) {
          uvc_perror(res, "start_streaming");
        } else {
          // UVC_AUTO_EXPOSURE_MODE_MANUAL=1
          // UVC_AUTO_EXPOSURE_MODE_AUTO=2
          // UVC_AUTO_EXPOSURE_MODE_SHUTTER_PRIORITY=3
          // UVC_AUTO_EXPOSURE_MODE_APERTURE_PRIORITY=4
          #define UVC_AUTO_EXPOSURE_MODE_MANUAL 1
          uvc_error_t e = uvc_set_ae_mode(devh, UVC_AUTO_EXPOSURE_MODE_MANUAL);
          uvc_perror(e, "FAILED set_ae_mode");

          //  0.0001 = 100 for 10ms
          uvc_set_exposure_abs(devh,200);
          uvc_perror(e, "FAILED uvc_set_exposure_abs");
          while(true)
          {
            FrameHandler::uvc_frame_t_wrap * pp = 0;
            fh.channel.readerGet(pp);
            if(pp)
            {
                        auto cvImg = cvCreateImageHeader(cvSize(pp->p->width, pp->p->height), IPL_DEPTH_8U,3);

              cvSetData(cvImg, pp->p->data, pp->p->width * 3); 
              cvNamedWindow("Test", CV_WINDOW_AUTOSIZE);
              cvShowImage("Test", cvImg);
              int q = cvWaitKey(10);
              if(q != -1)
                break;
              fh.channel.readerDone(pp);
              cvReleaseImageHeader(&cvImg);
            }
          }          
          uvc_stop_streaming(devh);
          puts("Done streaming.");
          fh.dostat();
        }
        ta.stop();
        std::cout << ta << std::endl;
        std::cout << "allocatedpools " << fh.allocatedpools<< std::endl;
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

