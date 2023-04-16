#pragma once

#include <filesystem>
#include <gdal_priv.h>
#include <memory>
#include <array>
#include <opencv2/core/types.hpp>
#include "projection.h"

class Elevation
{
    public:
    static void init();
    Elevation(std::filesystem::path path);
    cv::Point2d to_lonlat(cv::Point2d) const;
    cv::Point2d to_pix(cv::Point2d) const;
    double operator()(cv::Point2d lonlat) const;
    double operator()(double lon, double lat) const;
    using Transform = std::array<double, 6>;
    protected:
    std::shared_ptr<GDALDataset> ds_shared_;
    GDALDataset &ds_;
    Transform transform_;
    Transform inverse_;
    Projection projection_;
};
