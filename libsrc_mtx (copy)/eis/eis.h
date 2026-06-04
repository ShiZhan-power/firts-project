#ifndef __EIS_H__
#define __EIS_H__


struct MotionData
{
    float dx;
    float dy;	

};

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

    MotionData getMotion();

private:

    float t_;

    int src_w_;
    int src_h_;
    
    int crop_w_;
    int crop_h_;
    EisBuffer eis_buf_;
};

#endif
