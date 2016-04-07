#include "x264encoder.h"

using namespace std;

#if 1
    #define PIX_FMT_RGB24 AV_PIX_FMT_RGB24
    #define PIX_FMT_YUV420P AV_PIX_FMT_YUV420P
    #define PixelFormat AVPixelFormat
    #define CODEC_ID_H264 AV_CODEC_ID_H264
#endif 

x264Encoder::~x264Encoder()
{
    unInitilize();
    encoder_ = 0;
    convert_context_ = 0;
}

void x264Encoder::initialize(int w , int h , InputFormat fmt, int threads)
{
    image_w_ = w;
    image_h_ = h;
    AVPixelFormat inputformat = PIX_FMT_RGB24;
    inputformat_ = fmt;
    switch(fmt)
    {
        case InputFormat::RGB: inputformat = PIX_FMT_RGB24;
        case InputFormat::YUV420: inputformat = PIX_FMT_YUV420P;
        case InputFormat::YUYV: inputformat = PIX_FMT_YUYV422;
    }
    x264_param_default_preset(&parameters_, "veryfast", "zerolatency");

    parameters_.i_log_level = X264_LOG_INFO;
    parameters_.i_threads = threads;
    parameters_.i_width = image_w_;
    parameters_.i_height = image_h_;
    parameters_.i_fps_num = 30;
    parameters_.i_fps_den = 1;
    parameters_.i_keyint_max = 25;
    parameters_.b_intra_refresh = 1;
    parameters_.rc.i_rc_method = X264_RC_CRF;
    parameters_.rc.i_vbv_buffer_size = 2000000;
    parameters_.rc.i_vbv_max_bitrate = 90000;
    parameters_.rc.f_rf_constant = 25;
    parameters_.rc.f_rf_constant_max = 35;
    parameters_.i_sps_id = 7;
        // the following two value you should keep 1
        parameters_.b_repeat_headers = 1;    // to get header before every I-Frame
        parameters_.b_annexb = 1; // put start code in front of nal. we will remove start code later
        x264_param_apply_profile(&parameters_, "high");

    encoder_ = x264_encoder_open(&parameters_);

    picture_in_.i_qpplus1         = 0;
    picture_in_.img.i_plane       = 1;
    picture_in_.i_type = X264_TYPE_AUTO;
    picture_in_.img.i_csp = X264_CSP_I420;
    x264_picture_alloc(&picture_in_, X264_CSP_I420, 
                       parameters_.i_width, parameters_.i_height);
    if(inputformat != PIX_FMT_YUV420P)
    convert_context_ = sws_getContext(parameters_.i_width,
                                      parameters_.i_height,
                                      inputformat, 
                                      parameters_.i_width,
                                      parameters_.i_height,
                                      PIX_FMT_YUV420P,
                                      SWS_FAST_BILINEAR, NULL, NULL, NULL);
}

void x264Encoder::unInitilize()
{
    x264_encoder_close(encoder_);
    sws_freeContext(convert_context_);
}

//Encode the rgb frame into a sequence of NALs unit that are stored in a std::vector
void x264Encoder::encodeFrame(uint8_t *buffer, int step)
{
    x264_nal_t* nals ;
    int i_nals = 0;
    int frame_size = -1;

    if(!convert_context_)
    {
        x264_picture_t pin;
        pin.i_qpplus1         = 0;
        pin.i_type = X264_TYPE_AUTO;
        pin.img.i_plane       = 3;
        pin.img.i_csp = X264_CSP_I420;
        pin.img.i_stride[0] = step;
        pin.img.i_stride[1] = pin.img.i_stride[2] = step/2;
        pin.img.plane[0] = buffer;
        pin.img.plane[1] = buffer + step*image_h_; // first is full
        pin.img.plane[2] = pin.img.plane[1] + step*image_h_/4; // then we have /4

        frame_size = x264_encoder_encode(encoder_, &nals, &i_nals,&pin, &picture_out_);
    }
    else if(inputformat_ == InputFormat::RGB)
    {
        const uint8_t * rgb_buffer_slice[1] = {buffer};//{(const uint8_t *)rgb_buffer};
        int stride[1] = { step }; // RGB stride

        //Convert the frame from RGB to YUV420
        int slice_size = sws_scale(convert_context_, rgb_buffer_slice,
                                   stride, 0, image_h_, picture_in_.img.plane,
                                   picture_in_.img.i_stride);
        frame_size = x264_encoder_encode(encoder_, &nals, &i_nals,
                                         &picture_in_, &picture_out_);
    }
    else if(inputformat_ == InputFormat::YUYV)
    {
        const uint8_t * rgb_buffer_slice[1] = {buffer};//{(const uint8_t *)rgb_buffer};
        int stride[1] = { step }; // RGB stride

        //Convert the frame from RGB to YUV420
        int slice_size = sws_scale(convert_context_, rgb_buffer_slice,
                                   stride, 0, image_h_, picture_in_.img.plane,
                                   picture_in_.img.i_stride);
        frame_size = x264_encoder_encode(encoder_, &nals, &i_nals,
                                         &picture_in_, &picture_out_);
    }    
    if(frame_size > 0)
        handlenals(frame_size,nals,i_nals);
}

void x264Encoder::handlenals(int frame_size,x264_nal_t * p,int n)
{
    for(int i = 0; i <n; i++)
        output_queue_.push(p[i]);
}

bool x264Encoder::isNalsAvailableInOutputQueue()
{
    if(output_queue_.empty() == true)
        return false;
    else
        return true;
}

x264_nal_t x264Encoder::getNalUnit()
{
    x264_nal_t nal;
    nal = output_queue_.front();
    output_queue_.pop();
    return nal;
}
