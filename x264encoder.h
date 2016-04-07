#pragma once

#ifdef __cplusplus
#define __STDINT_MACROS
#define __STDC_CONSTANT_MACROS
#endif
#include <iostream>
#include <queue>
#include <stdint.h>

#ifndef INT64_C
#define INT64_C(c) (c ## LL)
#define UINT64_C(c) (c ## ULL)
#endif

extern "C" {
    #include "x264.h"
    #include <libswscale/swscale.h>
}


class x264Encoder
{
    public:
    enum class InputFormat { RGB,YUV420,YUYV};
    ~x264Encoder();
    void initialize(int w, int h, InputFormat fmt, int threads=1);
    void unInitilize();
    void encodeFrame(uint8_t *buffer, int buffer_size);
    bool isNalsAvailableInOutputQueue();
    operator bool () const { return image_w_ != 0; }
    x264_nal_t getNalUnit();
    x264_t *getx264Encoder() const { return encoder_; }
    int nal_size() const { return output_queue_.size(); }
    virtual void handlenals(int frame_size,x264_nal_t * p,int n);
private:
    int image_h_ = 0;
    int image_w_ = 0;
    InputFormat inputformat_ = InputFormat::YUV420;
    // Use this context to convert your BGR Image to YUV image since x264 do not support RGB input
    SwsContext* convert_context_ = NULL;
    std::queue<x264_nal_t> output_queue_;
    x264_param_t parameters_;
    x264_picture_t picture_in_, picture_out_;
    x264_t* encoder_ = 0;

};
