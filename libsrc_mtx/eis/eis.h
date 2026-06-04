#ifndef __EIS_H__
#define __EIS_H__

#include "motion_provider.h"
#include "sim_motion.h"
#include "motion_filter.h"
struct EisBuffer
{
    int fd;
    void* ptr;
    int size;
};

class EisProcess
{
public:

    EisProcess();

    int init(
        int src_w,
        int src_h);

    int process(
        int src_fd);

    void uninit();
    
    int getOutputFd();
    
    // === 新增：获取裁剪输出图像的宽高 ===
    int getCropW() const { return crop_w_; }
    int getCropH() const { return crop_h_; }
    

private:

    
    MotionProvider* provider_;
    MotionFilter filter_;

    int src_w_;
    int src_h_;
    
    int crop_w_;
    int crop_h_;
    EisBuffer eis_buf_;
};

#endif
