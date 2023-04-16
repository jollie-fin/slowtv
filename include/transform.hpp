#pragma once
#include <cassert>
#include <vector>
#include <array>
#include <iterator>
#include <algorithm>
#include <type_traits>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <cmath>

#include "utilities.h"
#include "constants.h"
#include <iostream>

inline auto to_rad(auto angle)
{
    angle *= pi / 180.;
    return angle;
}

inline auto to_deg(auto angle)
{
    angle /= pi / 180.;
    return angle;
}

inline double fast_haversine_m(double lat1, double lon1, double lat2, double lon2)
{
    lat1 *= pi / 180.;
    lon1 *= pi / 180.;
    lat2 *= pi / 180.;
    lon2 *= pi / 180.;
    auto lon_scal = std::cos(lat1);
    return std::hypot(lat1 - lat2, (lon1 - lon2)*lon_scal) * earth_radius;
}

inline double fast_haversine_m(cv::Point2d p1, cv::Point2d p2)
{
    return fast_haversine_m(p1.y, p1.x, p2.y, p2.x);
}

inline cv::Mat to_image(auto &container)
{
    auto nb_datapoints = container.size();
    using TYPE = std::remove_cvref_t<decltype(container[0])>;

    if constexpr (arithmetic<TYPE>)
    {
        return cv::Mat(1, nb_datapoints, cv_type<TYPE>, container.data());
    }
    else
    {
        std::size_t columns = container[0].size();
        using TYPE2 = typename TYPE::value_type;
        cv::Mat src(nb_datapoints, columns, cv_type<TYPE2>, container.data());
        cv::Mat dst;
        cv::transpose(src, dst);
        return dst;
    }
}

inline void resample(cv::Mat output,
                     cv::Mat input)
{
    assert(output.size().height == input.size().height);
    cv::resize(input, output, output.size());
}
