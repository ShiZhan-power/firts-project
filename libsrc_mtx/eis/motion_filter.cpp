#include "motion_filter.h"

MotionFilter::MotionFilter()
{
    initialized_ = false;
}

MotionData MotionFilter::update(
    const MotionData& raw)
{
    if(!initialized_)
    {
        smooth_ = raw;
        initialized_ = true;

        return smooth_;
    }

    float alpha = 0.05f;

    smooth_.dx =
        smooth_.dx * (1.0f - alpha)
        + raw.dx * alpha;

    smooth_.dy =
        smooth_.dy * (1.0f - alpha)
        + raw.dy * alpha;

    smooth_.yaw =
        smooth_.yaw * (1.0f - alpha)
        + raw.yaw * alpha;

    smooth_.pitch =
        smooth_.pitch * (1.0f - alpha)
        + raw.pitch * alpha;

    smooth_.roll =
        smooth_.roll * (1.0f - alpha)
        + raw.roll * alpha;

    smooth_.timestamp =
        raw.timestamp;

    return smooth_;
}
