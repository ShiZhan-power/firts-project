#ifndef __SIM_MOTION_H__
#define __SIM_MOTION_H__

#include "motion_provider.h"

class SimMotionProvider :
    public MotionProvider
{
public:

    SimMotionProvider();

    MotionData getMotion() override;

private:

    float t_;
};

#endif
