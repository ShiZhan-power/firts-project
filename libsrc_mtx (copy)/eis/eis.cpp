#include "eis.h"
#include <math.h>
#include <stdio.h>
#include "dma_alloc.h"

#include "rga/RgaApi.h"
#include "rga/im2d.h"
#include "rga/RgaUtils.h"

#include "mpp_common.h"


EisProcess::EisProcess()
{
    t_ = 0.0f;
    
    eis_buf_.fd = -1;
    eis_buf_.ptr = NULL;
    eis_buf_.size = 0;
}

int EisProcess::init(
    int src_w,
    int src_h)
{
    src_w_ = src_w;
    src_h_ = src_h;

    crop_w_ = 1280;
    crop_h_ = 720;

    int hor_stride = MPP_ALIGN(crop_w_,64);

    int ver_stride = MPP_ALIGN(crop_h_,16);

    eis_buf_.size = hor_stride * ver_stride * 3 / 2;

    int ret =
        dma_buf_alloc(
            DMA_HEAP_DMA32_UNCACHE_PATCH,
            eis_buf_.size,
            &eis_buf_.fd,
            (void**)&eis_buf_.ptr);

    if(ret < 0)
    {
        printf("EIS dma alloc failed\n");
        return -1;
    }

    printf("EIS buffer alloc success fd=%d\n", eis_buf_.fd);
        
    return 0;
}

MotionData EisProcess::getMotion()
{
    MotionData m;

    t_ += 0.03f;

    m.dx = sin(t_) * 120.0f;
    m.dy = cos(t_ * 0.7f) * 60.0f;

    return m;
}


int EisProcess::process(int src_fd)
{
    MotionData m = getMotion();
    
    printf(
       "[DEBUG] t=%.3f dx=%.3f dy=%.3f\n",
       t_,
       m.dx,
       m.dy);
    
    
    
    int center_x = src_w_ / 2;
    int center_y = src_h_ / 2;

    //int crop_x = center_x - crop_w_ / 2 + (int)m.dx;
    //int crop_y = center_y - crop_h_ / 2 + (int)m.dy;     
   
    int crop_x = center_x - crop_w_ / 2;
    int crop_y = center_y - crop_h_ / 2; 
    crop_x &= ~1;
    crop_y &= ~1;
    
    if(crop_x < 0)
       crop_x = 0;

    if(crop_y < 0)
       crop_y = 0;

    if(crop_x + crop_w_ > src_w_)
       crop_x = src_w_ - crop_w_;

    if(crop_y + crop_h_ > src_h_)
       crop_y = src_h_ - crop_h_;  

    printf("[EIS] dx=%.2f dy=%.2f -> crop=(%d, %d)\n", m.dx, m.dy, crop_x, crop_y);

    // 注意：尚未执行 RGA 裁剪，只是计算坐标
    // 下一步我们会把 src_fd 中 (crop_x, crop_y, crop_w_, crop_h_) 拷贝到 eis_buf_.fd
    
    
    
    
    //新增：RGA裁剪
    // 计算源图的 stride（通常原始帧 stride 就是对齐后的宽高）
    int src_hor_stride = MPP_ALIGN(src_w_, 64);
    int src_ver_stride = MPP_ALIGN(src_h_, 16);
    
    // 计算目标图的 stride（必须与 init 中分配 eis_buf_ 时使用的 stride 一致）
    int dst_hor_stride = MPP_ALIGN(crop_w_, 64);
    int dst_ver_stride = MPP_ALIGN(crop_h_, 16);
    
    rga_buffer_t src = wrapbuffer_fd(src_fd,
                                     src_w_, src_h_,
                                     RK_FORMAT_YCbCr_420_SP,
                                     src_hor_stride, src_ver_stride);
    rga_buffer_t dst = wrapbuffer_fd(eis_buf_.fd,
                                     crop_w_, crop_h_,
                                     RK_FORMAT_YCbCr_420_SP,
                                     dst_hor_stride, dst_ver_stride);
                                     
    im_rect src_rect = { crop_x, crop_y, crop_w_, crop_h_ };
    im_rect dst_rect = { 0, 0, crop_w_, crop_h_ };
    
    
    IM_STATUS ret = improcess(src, dst, {}, src_rect, dst_rect, {}, IM_SYNC);
    if (ret != IM_STATUS_SUCCESS) {
        printf("[EIS] RGA failed: %s\n", imStrError(ret));
        return -1;
    }
    
    return 0;
}

void EisProcess::uninit()
{


    if(eis_buf_.fd >= 0)
    {
        dma_buf_free(
            eis_buf_.size,
            &eis_buf_.fd,
            &eis_buf_.ptr);

        eis_buf_.fd = -1;
    }
}
int EisProcess::getOutputFd()
{
    return eis_buf_.fd;
}

