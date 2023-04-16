#include <opencv2/core.hpp>
#include <gdal.h>
#include <ogr_spatialref.h>
#include <iostream>
#include <cmath>
#include "constants.h"
#include "elevation.h"

namespace fs = std::filesystem;

static std::shared_ptr<GDALDataset> open_gdal(fs::path path)
{
    auto dataset = reinterpret_cast<GDALDataset*>(
                GDALOpen(path.c_str(), GDALAccess::GA_ReadOnly));
    return {dataset, GDALClose};
}

static auto &deref_or_throw(const auto &ptr, fs::path path, auto name)
{
    if (!ptr)
        throw std::runtime_error("No " + std::string(name) + " in " + path.string());
    return *ptr;
}

static Elevation::Transform get_transform(GDALDataset &ds)
{
    Elevation::Transform t;
    if (ds.GetGeoTransform(t.data()) != CPLErr::CE_None)
        throw std::runtime_error("Impossible to find the geotransform");
    return t;
}

static Elevation::Transform get_inverse(const Elevation::Transform &transform)
{
    Elevation::Transform retval;
    if (GDALInvGeoTransform(const_cast<double *>(transform.data()), retval.data()) != true)
        throw std::runtime_error("Impossible to inverse the geotransform");
    return retval;
}

static std::string get_srs(GDALDataset &ds)
{
    OGRSpatialReference ogr(ds.GetProjectionRef());
    char *str;
    ogr.exportToProj4(&str);
    std::string retval{str};
    CPLFree(str);
    return retval;
}

void Elevation::init()
{
    GDALAllRegister();
}

Elevation::Elevation(fs::path path)
    : ds_shared_(open_gdal(path))
    , ds_(deref_or_throw(ds_shared_, path, "dataset"))
    , transform_(get_transform(ds_))
    , inverse_(get_inverse(transform_))
    , projection_(get_srs(ds_))
{
    assert(ds_.GetRasterCount() == 1);
}

cv::Point2d Elevation::to_lonlat(cv::Point2d input) const
{
    cv::Point2d retval;

    GDALApplyGeoTransform(const_cast<double *>(transform_.data()), input.x, input.y, &retval.x, &retval.y);
    return projection_.to_lonlat(retval);
}

cv::Point2d Elevation::to_pix(cv::Point2d input) const
{
    input = projection_.from_lonlat(input);
    cv::Point2d retval;
    GDALApplyGeoTransform(const_cast<double *>(inverse_.data()), input.x, input.y, &retval.x, &retval.y);
    return retval;
}

double Elevation::operator()(double lon, double lat) const
{
    return (*this)(cv::Point2d(lon, lat));
}

double Elevation::operator()(cv::Point2d lonlat) const
{
    cv::Point2d input = to_pix(lonlat);

    int x = input.x;
    int y = input.y;
    double zs[4];
    if (ds_.RasterIO(GDALRWFlag::GF_Read, x, y, 2, 2, &zs[0], 2, 2, GDALDataType::GDT_Float64, 1, nullptr, 0, 0, 0) != CPLErr::CE_None)
        return std::nan("");
    double alphax = input.x - x;
    double top    = zs[1] * alphax + zs[0] * (1 - alphax);
    double bottom = zs[3] * alphax + zs[2] * (1 - alphax);
    double alphay = input.y - y;
    double retval = bottom * alphay + top * (1 - alphay);
    return retval;
}