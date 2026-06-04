#ifndef __MOTION_FILTER_H__
#define __MOTION_FILTER_H__

#include "motion_provider.h"

class MotionFilter
{
public:

    MotionFilter();

    MotionData update(
        const MotionData& raw);

private:

    MotionData smooth_;

    bool initialized_;
};

#endif
