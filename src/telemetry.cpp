#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <optional>
#include <vector>
#include <set>
#include <array>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <filesystem>
#include <cstdio>
#include <boost/process.hpp>
#include <functional>
#include <memory>
#include "transform.hpp"
#include "gpmf_reader.h"
#include "collection_video.h"
#include "elevation.h"
#include "constants.h"
#include "configuration.h"
#include "telemetry.h"
#include "catenate_video.h"
#include "projection.h"

namespace bp = boost::process;

static cv::Range range_row(int row)
{
    return {row, row + 1};
}

static const auto range_all = cv::Range::all();

static std::pair<double, double> weighted_sum()
{

}

static void interpolate(cv::Mat mat, cv::Mat distance = {}, double modulo = -1.)
{
    std::size_t height = mat.rows;
    std::size_t  count = mat.cols;
    if (!count)
        return;
    std::size_t end_mat = count - 1;
    assert(count == static_cast<std::size_t>(distance.cols) || distance.empty());
    double offset_distance = distance.empty() ? 0. : distance.at<double>(0);
    double span = distance.empty() ? static_cast<double>(end_mat) : distance.at<double>(end_mat) - offset_distance                   ;

    for (std::size_t row = 0; row != height; row++)
    {
        double start = mat.at<double>(row, 0);
        double end = mat.at<double>(row, end_mat);

        if (modulo > 0.)
        {
            if (start < end - modulo / 2.)
                end -= modulo;
            else if (start > end + modulo / 2.)
                end += modulo;
        }

        for (std::size_t j = 1; j < end_mat; j++)
        {
            double accumul = distance.empty() ? static_cast<double>(j) : distance.at<double>(j);
            double alpha = accumul / span;
            double new_val = alpha * end + (1. - alpha) * start;
            mat.at<double>(row, j) = new_val;
        }
    }
}


static void patch(cv::Mat mat, int lat_row, int lon_row, int spe_row, double max_distance)
{
    std::vector<std::pair<std::size_t, double>> retval{{0,0.}};
    int count = mat.size().width;
    int previous = 0;
    assert(mat.type() == cv_type<double>);

    auto *lats = mat.ptr<double>(lat_row);
    auto *lons = mat.ptr<double>(lon_row);

    for (int i = 0; i != count; i++)
    {
        double distance = fast_haversine_m(lats[previous], lons[previous], lats[i], lons[i]);
        if (distance < max_distance)
        {
            if (previous + 1 < i)
            {
                cv::Range to_interpolate{previous, i + 1};
                interpolate(mat(range_all, to_interpolate), mat(range_row(spe_row), to_interpolate));
                std::cout << "Fixing between " << previous << " and " << i << std::endl;
            }
            previous = i;
        }
        else
            std::cout << "! between " << previous << "&" << i << std::endl;
    }

}

static std::vector<std::size_t> fill_distance(cv::Mat mat_lats, cv::Mat mat_lons, cv::Mat mat_dist, double max_distance)
{
    std::vector<std::size_t> retval;
    assert(mat_lats.rows == mat_lons.rows);
    assert(mat_lats.rows == mat_dist.rows);
    assert(mat_lats.cols == 1);
    assert(mat_lons.cols == 1);
    assert(mat_dist.cols == 1);
    assert(mat_lats.type() == cv_type<double>);
    assert(mat_lons.type() == cv_type<double>);
    assert(mat_dist.type() == cv_type<double>);

    auto count = mat_lats.rows;
    double *lats = mat_lats.ptr<double>();
    double *lons = mat_lons.ptr<double>();
    double *dist = mat_dist.ptr<double>();

    double lat1 = to_rad(lats[0]);
    double lon1 = to_rad(lons[0]);
    double lon_scale = std::cos(lat1);
    double accumul = 0.;

    for (int i = 0; i < count; i++)
    {
        double lat2 = to_rad(lats[i]);
        double lon2 = to_rad(lons[i]);

        double distance = std::hypot(lat2 - lat1, (lon2 - lon1) * lon_scale) * earth_radius;
        dist[i] = accumul + distance;

        if (i == 0 || distance > max_distance || i == count - 1)
        {
            retval.emplace_back(i);
            lat1 = lat2;
            lon1 = lon2;
            lon_scale = std::cos(lat1);
            accumul += distance;
        }
    }
    return retval;
}

struct GeoPosition
{
    int index_{-1};
    double cumul_{0.};
    cv::Point2d position_;
};

struct Run
{
    int start_;
    int end_;
    bool is_pause_;
};

static std::vector<Run> extract_runs(cv::Mat mat_dist, double length_stop, int duration_stop)
{
    std::vector<Run> retval;
    assert(mat_dist.cols == 1);
    assert(mat_dist.type() == cv_type<double>);
    int last = 0;
    int count = mat_dist.rows;
    double *distance = mat_dist.ptr<double>(0);
    for (auto i = 0; i != count; i++)
    {
        double length_run = distance[i] - distance[last];
        if (length_run > length_stop || i == count - 1)
        {
            retval.emplace_back(last, i, length_run * duration_stop < length_stop * (i - last));
            last = i;
        }
    }
    return retval;
}

static std::vector<Run> cut_runs(cv::Mat mat_dist, double max_distance, const std::vector<Run> &runs)
{
    std::vector<Run> retval;
    assert(mat_dist.cols == 1);
    assert(mat_dist.type() == cv_type<double>);
    int count = mat_dist.rows;
    double *distance = mat_dist.ptr<double>(0);
    for (const auto &run : runs)
    {
        if (run.is_pause_)
        {
            retval.emplace_back(run);
            continue;
        }

        double run_length = distance[run.end_] - distance[run.start_];
        int divisor = std::ceil(run_length / max_distance);
        int left = run.start_;
        int segment = 1;
        double length;

        for (int j = run.start_; j <= run.end_; j++)
        {
            length = distance[j] - distance[run.start_];
            if (length >= segment * run_length / divisor)
            {
                int right = j - 1;
                retval.emplace_back(left, right, length);
                left = right + 1;
                segment++;
            }
        }
        if (left != run.end_)
            retval.emplace_back(left, run.end_, length);
    }
    return retval;
}

static void interpolate_along_distance(
    cv::Mat mat_dest, cv::Mat mat_distance, const std::vector<Run> &runs,
    std::function<double(double, const Run &run)> functor, double modulo = -1.)
{
    assert(mat_dest.cols == 1);
    assert(mat_dest.type() == cv_type<double>);

    for (const auto &run : runs)
    {
        assert(run.end_ > 0);
        mat_dest.at<double>(0, run.end_ - 1) = functor(mat_dest.at<double>(0, run.start_), run.start_, run.end_);
        cv::Range range{run.start_, run.end_};
        interpolate(mat_dest(range_all, range), mat_distance(range_all, range), modulo);
    }
}

// static std::vector<std::pair<std::size_t, double>> normalize_distance_new(cv::Mat mat_lats, cv::Mat mat_lons, double max_distance)
// {
//     std::vector<std::pair<std::size_t, double>> retval{{0,0.}};
//     assert(mat_lats.size() == mat_lons.size());
//     assert(mat_lats.size().height == 1);
//     std::size_t count = mat_lats.size().width;
//     std::size_t previous = 0;
//     assert(mat_lats.type() == cv_type<double>);
//     assert(mat_lons.type() == cv_type<double>);

//     auto *lats = mat_lats.ptr<double>(0);
//     auto *lons = mat_lons.ptr<double>(0);

//     std::cout << count << std::endl;
//     for (std::size_t i = 0; i != count; i++)
//     {
//         double distance = fast_haversine_m(lats[previous], lons[previous], lats[i], lons[i]);
//         if (distance > max_distance * 2)
//         {
//             std::cout << "!!!!! " << i << ";" << lats[previous] << "," << lons[previous] << "," << lats[i] << "," << lons[i] << std::endl;
//         }
//         if (distance > max_distance || i == count - 1)
//         {
//             double cumul_distance = distance;
//             cumul_distance += retval.back().second;
//             retval.emplace_back(i, cumul_distance);
//             previous = i;
//         }
//     }
//     return retval;
// }

// static std::vector<std::pair<std::size_t, double>> normalize_distance(cv::Mat mat_lats, cv::Mat mat_lons, double max_distance)
// {
//     std::vector<std::pair<std::size_t, double>> retval;
//     assert(mat_lats.size() == mat_lons.size());
//     assert(mat_lats.size().height == 1);
//     std::size_t count = mat_lats.size().width;
//     std::size_t previous = 0;
//     assert(mat_lats.type() == cv_type<double>);
//     assert(mat_lons.type() == cv_type<double>);

//     auto *lats = mat_lats.ptr<double>(0);
//     auto *lons = mat_lons.ptr<double>(0);

//     for (std::size_t i = 0; i != count; i++)
//     {
//         double distance = fast_haversine_m(lats[previous], lons[previous], lats[i], lons[i]);
//         while (distance > max_distance)
//         {
//             previous++;
//             distance = fast_haversine_m(lats[previous], lons[previous], lats[i], lons[i]);
//         }
//         double cumul_distance = distance;
//         if (previous != i)
//             cumul_distance += retval[previous].second;
//         retval.emplace_back(previous, cumul_distance);
//     }
//     return retval;
// }


// std::vector<std::pair<std::size_t, std::size_t>> bucketize_recur(const std::vector<std::pair<std::size_t, double>> &per_distance, std::size_t start, std::size_t end)
// {
//     if (start >= end)
//         return {};

//     auto biggest_bucket = std::make_pair(start, start);
//     std::size_t biggest_bucket_size = 0;
//     for (std::size_t i = start; i != end; i++)
//     {
//         auto previous = std::max(start, per_distance[i].first);
//         auto bucket_size = i - previous;
//         if (bucket_size > biggest_bucket_size)
//         {
//             biggest_bucket_size = bucket_size;
//             biggest_bucket = std::make_pair(previous, i);
//         }
//     }

//     auto left = bucketize_recur(per_distance, start, biggest_bucket.first);
//     left.emplace_back(biggest_bucket);
//     auto right = bucketize_recur(per_distance, biggest_bucket.second + 1, end);
//     left.insert(left.end(), right.begin(), right.end());
//     return left;
// }

// std::vector<std::pair<std::size_t, std::size_t>> bucketize(const std::vector<std::pair<std::size_t, double>> &per_distance)
// {
//     return bucketize_recur(per_distance, 0, per_distance.size());
// }

//ffmpeg -y -i GOPR0001.MP4 -codec copy -map 0:m:handler_name:" GoPro MET" -f rawvideo GOPR0001.bin

Telemetry::Telemetry(const Configuration &conf)
{
    CatenateVideo catv(conf);

//     int count;
//     int fps;

//     {
//         cv::VideoCapture video = std::string(file);
//         count = video.get(cv::CAP_PROP_FRAME_COUNT);
//         fps = std::round(video.get(cv::CAP_PROP_FPS));
//     }

//     std::cout << "Fps:" << fps << std::endl;

// // latitude
// // longitude
// // z
// // speed
// // ax
// // ay
// // az

    auto count = catv.count();
    auto fps = catv.fps();
    cv::Mat rawdata{6, count, cv_type<double>, cv::Scalar(0.)};

    {
        int position = 0;
        for (auto path : conf.videos_)
        {
            bp::ipstream pipe_stream;
            bp::child c(
                bp::search_path("ffmpeg"), "-i", path.string(),
                "-codec", "copy", "-map", "0:3", "-f", "rawvideo", "-",
                bp::std_out > pipe_stream, bp::std_err > bp::null);

            std::cout << "Reading " << path << std::endl;
            for (const Packet &packet : Gpmf(pipe_stream))
            {
                assert(position < count);

                auto end = std::min(position + fps, count);
                auto range_dest = cv::Range(position, end);
            
                resample(rawdata(cv::Range(0,2), range_dest), packet.gps_position_(cv::Range(0,2), range_all));
                resample(rawdata(range_row(2), range_dest), packet.gps_position_(range_row(2), range_all));
                resample(rawdata(cv::Range(3,6), range_dest), packet.accelerometer_);
                position += fps;
            }
            c.wait();
        }

        count = position;
    }

    std::cout << "Prepare output" << std::endl;

    data_ = cv::Mat{NB_CHANNELS, count, cv_type<double>, cv::Scalar(0.)};
    std::cout << "data_.size " << data_.size() << std::endl;
    std::cout << "data_.rows " << data_.rows << std::endl;
    rawdata(cv::Range(0,4), cv::Range(0, count))
        .copyTo(data_(cv::Range(LATITUDE, SPEED + 1), cv::Range(0, count)));


    std::cout << "patch" << std::endl;
    patch(data_, LATITUDE, LONGITUDE, SPEED, 100.);

    auto lats = data_.ptr<double>(LATITUDE);
    auto lons = data_.ptr<double>(LONGITUDE);

    std::cout << "cap + pause + distance + elevation" << std::endl;

    std::vector<std::pair<std::size_t, std::size_t>> pauses;

    {
        std::cout << "Compute Heading" << std::endl;
        Projection projection(conf.osm_srs_);
        cv::Point2d previous_position;
        interpolate_along_distance(data_.row(LATITUDE), data_.row(LONGITUDE), data_.row(SPEED), data_.row(HEADING), 5.,
            [&](double, const GeoPosition &previous, const GeoPosition &next)
            {
                double retval = 0.;
                cv::Point2d merc_pos = projection.from_lonlat(next.position_);
                
                if (previous.index_ != next.index_)
                {
                    auto diff = merc_pos - previous_position;
                    retval = std::atan2(diff.y, diff.x) * 180. / pi;
                }
                previous_position = merc_pos;
                return retval;
            },
            360);
        std::cout << "Compute Heading : Ok" << std::endl;

        std::cout << "Compute Elevation" << std::endl;
        Elevation elevation(conf.elevation_);
        interpolate_along_distance(data_.row(LATITUDE), data_.row(LONGITUDE), data_.row(SPEED), data_.row(ELEVATION), 5.,
            [&](double, const GeoPosition &, const GeoPosition &next)
            {
                return elevation(next.position_);
            });
        std::cout << "Compute Elevation : Ok" << std::endl;

        
        int previous = 0;
        cv::Point2d previous_position;


        for (auto [position, distance, geo] : IterateDistance(data_.row(LATITUDE), data_.row(LONGITUDE), 5.))
        {
            if (position - previous > 60 * fps)
                pauses.emplace_back(previous, position);

            auto z = elevation(lons[position], lats[position]);
            auto merc_pos = projection.from_lonlat({lons[position], lats[position]});

            data_.at<double>(DISTANCE, position) = distance;
            data_.at<double>(ELEVATION, position) = z;

            if (previous < position)
            {
                cv::Range range_dest{static_cast<int>(previous), static_cast<int>(position) + 1};
                auto speed = data_(range_row(SPEED), range_dest);
                auto diff = merc_pos - previous_position;
                data_.at<double>(HEADING, position) = std::atan2(diff.y, diff.x) * 180. / pi;
                interpolate(data_(range_row(HEADING), range_dest), speed, 360.);
                interpolate(data_(range_row(DISTANCE), range_dest), speed);
                interpolate(data_(range_row(ELEVATION), range_dest), speed);
            }
            previous = position;
            previous_position = merc_pos;
        }
    }


    std::cout << data_(range_all, range_row(3600)) << std::endl;
    
    // std::vector<std::size_t> pauses;
    // for (std::size_t i = 0; i != buckets.size(); i++)
    // {
    //     auto [start, end] = buckets[i];
    //     if (end - start > 60ul * catv.fps())
    //     {
    //         pauses.emplace_back(i);
    //         std::cout << "Pause " << start << ":" << ((end - start) / catv.fps() / 60) << "min" << std::endl;
    //     }
    //     // if (i > 10749)
    //     // {
    //     //     std::cout << "??? " << start << "," << end << ":" << (end - start) << std::endl;
    //     // }
    // }
    // std::cout << "Pause [0," << buckets.size() << "]" << std::endl;

    // std::size_t previous = 0;
    // for (auto [n, d] : normalized_new)
    // {
    //     if (n - previous > static_cast<std::size_t>(60 * catv.fps()))
    //     {
    //         std::cout << "Pausenew " << previous << ":" << ((n - previous) / catv.fps() / 60) << "min" << std::endl;
    //     }
    //     previous = n;
    // }

    std::cout << "distance " << (data_.at<double>(DISTANCE, count - 1)) << std::endl;

    // std::size_t previous = 0;
        //     double previouslat = data_.at<double>(0, previous);
        //     double previouslon = data_.at<double>(1, previous);
        //     double startlat = data_.at<double>(0, start);
        //     double startlon = data_.at<double>(1, start);
        //     double previousz = elevation(previouslon, previouslat);
        //     double startz = elevation(startlon, startlat);
        //     double previouscumul = normalized[previous].second;
        //     double startcumul = normalized[start].second;

        //     std::cout << "Path " << ((start - previous)/catv.fps()/60) << "min " << ((startcumul - previouscumul)/1000.) << "km " << (startz - previousz) << "m " << ((startz - previousz)/(startcumul - previouscumul)*100) << "%" << std::endl;
        //     previous = end;
        // }

    // for (std::size_t i = 1610000; i != 1624552; i++)
    // {
    //     std::cout << "(" << i << "," << normalized[i].first << "," << normalized[i].second << "),";
    //     if (i % 8 == 0)
    //         std::cout << std::endl;
    // }
    // std::cout << std::endl;






// latitude
// longitude
// z
// speed
// horizon
// pente

}

// void test()
// {

//     std::vector<std::array<double, 5>> tab = {
//         {60,61,2,3,4},
//         {61,62,3,4,5},
//         {61,62,3,4,5},
//         {61,62,3,4,5},
//         {62,63,5,6,7},
//         {63,64,10,11,12}};

//     std::vector<std::array<double, 5>> tab2(8);
//     resample(tab2.begin(), tab2.end(), tab.begin(), tab.end());
//     auto nd = normalize_distance(tab2.begin(), tab2.end(), 130e3);
//     auto buckets = bucketize(nd);
//     int i = 0;
//     for (const auto &col : nd)
//     {
//         std::cout << i << "," << col.first << "," << col.second << std::endl;
//         i++;
//     }
//     for (const auto &col : buckets)
//     {
//         std::cout << col.first << "," << col.second << std::endl;
//     }

//     for (const auto &img : {tab, tab2})
//     {
//         for (const auto &col : img)
//         {
//             for (auto elt : col)
//                 std::cout << elt << ",";
//             std::cout << std::endl;
//         }
//         std::cout << std::endl;
//         std::cout << std::endl;
//     }

//     std::vector<int16_t> tab3 = {
//         {1,2,3,4}};

//     std::vector<int16_t> tab4(2);
//     resample(tab4.begin(), tab4.end(), tab3.begin(), tab3.end());
//     std::cout << tab4[0] << "," << tab4[1] << std::endl;
// }

