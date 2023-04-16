#pragma once

#include <mapnik/map.hpp>
#include <mapnik/image_util.hpp>
#include <mapnik/agg_renderer.hpp>
#include <opencv2/core.hpp>
#include <tuple>
#include <filesystem>
#include <chrono>
#include "projection.h"

class MapGenerator
{
    public:
        MapGenerator(std::filesystem::path xml_file, std::filesystem::path tile_cache, int size_pxl);
        static void init();
        static std::string read_srs(std::filesystem::path xml_file);
        std::string_view srs() const;
        cv::Point2d deg2num(cv::Point2d deg, int zoom) const;
        cv::Point2d num2deg(cv::Point2d num, int zoom) const;
        cv::Mat get_tile(cv::Point2i position, int zoom);
        cv::Mat operator()(double latitude, double longitude, int zoom);

    protected:
        void initialize_tile_cache();
        void update_tile_cache(const std::filesystem::path &path);
        void drop_oldest();

        std::filesystem::path tile_cache_;
        cv::Size size_;
        double zoom_pow_;
        cv::Point2i last_tile_pos_;
        cv::Mat last_tile_;
        mapnik::Map osm_map_;
        std::string srs_;
        Projection projection_;

        using MapAccessTime = std::map<std::filesystem::path, std::filesystem::file_time_type>;

        struct OrderFile
        {
            bool operator()(MapAccessTime::iterator a, MapAccessTime::iterator b) const
            {
                return a->second < b->second;
            }
        };

        MapAccessTime tile_files_;
        std::set<MapAccessTime::iterator, OrderFile> tile_files_by_date_;
};

void init_mapnik();
