#pragma once

#include <boost/math/constants/constants.hpp>
#include <string_view>

constexpr double pi = boost::math::double_constants::pi;
constexpr double tau = boost::math::double_constants::two_pi;
constexpr double earth_radius = 6371e3;

static const cv::Point2i I{0,1};
static const cv::Point2i J{1,0};
constexpr int thres_cleaning_max = 200;
constexpr int thres_cleaning_min = 150;
static const char lonlat_srs[] = "+proj=longlat +ellps=WGS84 +datum=WGS84 +no_defs";
