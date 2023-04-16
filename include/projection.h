#pragma once

#include <string>
#include <mapnik/projection.hpp>
#include <mapnik/proj_transform.hpp>
#include <mapnik/box2d.hpp>
#include <opencv2/core/types.hpp>

class Projection
{
    public:
    Projection(std::string srs);
    cv::Point2d from_lonlat(cv::Point2d src) const;
    cv::Point2d to_lonlat(cv::Point2d src) const;
    mapnik::box2d<double> from_lonlat(mapnik::box2d<double> src) const;
    mapnik::box2d<double> to_lonlat(mapnik::box2d<double> src) const;
    protected:
    mapnik::projection proj_src_;
    mapnik::projection proj_dst_;
    mapnik::proj_transform transform_proj_;
};
