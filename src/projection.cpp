#include "projection.h"
#include "constants.h"

Projection::Projection(std::string srs_dst)
    : proj_src_(lonlat_srs)
    , proj_dst_(srs_dst)
    , transform_proj_(proj_src_, proj_dst_)
{
}

cv::Point2d Projection::from_lonlat(cv::Point2d src) const
{
    double z = 0.;
    transform_proj_.forward(src.x, src.y, z);
    return src;
}

cv::Point2d Projection::to_lonlat(cv::Point2d src) const
{
    double z = 0.;
    transform_proj_.backward(src.x, src.y, z);
    return src;
}

mapnik::box2d<double> Projection::from_lonlat(mapnik::box2d<double> src) const
{
    if (!transform_proj_.forward(src))
        throw std::runtime_error("Impossible to change coordinates for box2d");
    return src;
}

mapnik::box2d<double> Projection::to_lonlat(mapnik::box2d<double> src) const
{
    if (!transform_proj_.backward(src))
        throw std::runtime_error("Impossible to change coordinates for box2d");
    return src;
}
