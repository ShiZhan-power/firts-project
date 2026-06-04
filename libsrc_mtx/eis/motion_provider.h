#ifndef __MOTION_PROVIDER_H__
#define __MOTION_PROVIDER_H__
#include <stdint.h>
struct MotionData
{
    float dx;
    float dy;
    
    float yaw;
    float pitch;
    float roll;

    uint64_t timestamp;
};


class MotionProvider
{
public:

    virtual ~MotionProvider(){}

    virtual MotionData getMotion() = 0;
};

#endif
