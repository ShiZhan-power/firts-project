#include "sim_motion.h"

#include <math.h>
#include <cstdio>

SimMotionProvider::SimMotionProvider()
{
    t_ = 0.0f;
}

MotionData SimMotionProvider::getMotion()
{
    MotionData m;
    
    m.yaw = 0.0f;
    m.pitch = 0.0f;
    m.roll = 0.0f;

    m.timestamp = 0;
    
    

    t_ += 0.03f;
    printf("[SIM] t=%.3f\n", t_);
    
    
    

    m.dx = sin(t_) * 120.0f;
    m.dy = cos(t_ * 0.7f) * 60.0f;

    return m;
}
