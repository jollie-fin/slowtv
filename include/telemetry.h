#pragma once

#include <opencv2/core/types.hpp>
#include "configuration.h"

class Telemetry
{
    public:
    Telemetry(const Configuration &);

    enum Channel {LATITUDE, LONGITUDE, ELEVATION, SPEED, HEADING, ROLL, SLOPE, DISTANCE, TIME, NB_CHANNELS};
    cv::Mat data() const
    {
        return data_;
    }

    protected:
        cv::Mat data_;
};

