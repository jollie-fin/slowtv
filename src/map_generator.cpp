#include <cmath>
#include <mapnik/map.hpp>
#include <mapnik/agg_renderer.hpp>
#include <mapnik/image.hpp>
#include <mapnik/image_util.hpp>
#include <mapnik/load_map.hpp>
#include <mapnik/font_engine_freetype.hpp>
#include <mapnik/datasource_cache.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <filesystem>
#include <iterator>
#include <iostream>
#include <sstream>
#include <chrono>
#include <stdexcept>
#include <cassert>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include "constants.h"
#include "utilities.h"
#include "map_generator.h"

#define stringify(s) xstringify(s)
#define xstringify(s) #s

namespace fs = std::filesystem;
namespace bpt = boost::property_tree;

void MapGenerator::init()
{
    mapnik::freetype_engine::register_fonts(stringify(MAPNIK_DEFAULT_FONT_PATH), true);
    mapnik::datasource_cache::instance().register_datasources(stringify(MAPNIK_DEFAULT_INPUT_PATH));
}

std::string MapGenerator::read_srs(fs::path xml_file)
{
    bpt::ptree tree;
    bpt::xml_parser::read_xml(xml_file, tree);
    return tree.get<std::string>("Map.<xmlattr>.srs");
}

static mapnik::Map read_map(fs::path xml_file, cv::Size size)
{
    mapnik::Map retval{size.width, size.height};
    mapnik::logger::set_severity(mapnik::logger::severity_type::none);
    mapnik::load_map(retval, xml_file.string());
    return retval;
}

MapGenerator::MapGenerator(fs::path xml_file, fs::path tile_cache, int size_pxl)
    : tile_cache_(tile_cache)
    , size_(size_pxl, size_pxl)
    , last_tile_pos_(-1, -1)
    , osm_map_(read_map(xml_file, size_))
    , srs_(osm_map_.srs())
    , projection_(srs_)
{
    initialize_tile_cache();
}

std::string_view MapGenerator::srs() const
{
    return srs_;
}

cv::Point2d MapGenerator::deg2num(cv::Point2d deg, int zoom) const
{
    auto rad = deg * pi / 180.;
    rad.y = -std::asinh(std::tan(rad.y));
    auto retval = (.5 + rad / tau) * std::pow(2., zoom);
    return retval;
}

cv::Point2d MapGenerator::num2deg(cv::Point2d num, int zoom) const
{
    auto rad = (num / std::pow(2., zoom)) * tau - pi;
    rad.y = -std::atan(std::sinh(rad.y));
    return rad * 360. / tau;
}

void MapGenerator::initialize_tile_cache()
{
    for (const auto &p : fs::directory_iterator(tile_cache_))
    {
        auto iterator  = tile_files_.emplace(p.path(), p.last_write_time()).first;
        tile_files_by_date_.emplace(iterator);
    }
}

void MapGenerator::update_tile_cache(const fs::path &path)
{
    auto iterator = tile_files_.find(path);
    auto now = std::chrono::file_clock::now();

    if (iterator != tile_files_.end())
    {
        tile_files_by_date_.erase(iterator);
        tile_files_.erase(iterator);
    }

    iterator  = tile_files_.emplace(path, now).first;
    tile_files_by_date_.emplace(iterator);
}

void MapGenerator::drop_oldest()
{
    if (tile_files_.size() < thres_cleaning_max)
        return;

    auto to_clean = tile_files_.size() - thres_cleaning_min;

    for (std::size_t i = 0; i < to_clean; i++)
    {
        auto iterator = tile_files_.begin();
        auto path = iterator->first;
        std::cout << "Removing " << path << std::endl;
        fs::remove(path);
        tile_files_by_date_.erase(iterator);
        tile_files_.erase(iterator);
    }
}

cv::Mat MapGenerator::get_tile(cv::Point2i tile_pos, int zoom)
{
    std::ostringstream sstr;
    sstr << "tile_" << size_.width << "_" << size_.height << "_" << zoom << "_" << tile_pos.x << "_" << tile_pos.y << ".png";

    auto pathname = tile_cache_ / sstr.str();
    update_tile_cache(pathname);
    drop_oldest();

    std::cout << "Fetch " << pathname << std::endl;
    if (!fs::exists(pathname))
    {
        std::cout << "Compute " << pathname << std::endl;
        auto NW = num2deg(tile_pos, zoom);
        auto SE = num2deg(tile_pos + I + J, zoom);
        mapnik::box2d box{NW.x, NW.y, SE.x, SE.y};
        box = projection_.from_lonlat(box);
        osm_map_.zoom_to_box(box);
        mapnik::image_rgba8 mapnik_image(size_.width, size_.height);
        mapnik::agg_renderer<mapnik::image_rgba8> mapnik_render(osm_map_, mapnik_image);

        mapnik_render.apply();
        mapnik::save_to_file(mapnik_image, pathname.string(), "png24"); 
    }
    return cv::imread(pathname.string(), cv::IMREAD_COLOR);
}

cv::Mat MapGenerator::operator()(double latitude, double longitude, int zoom)
{
    cv::Point2d lon_lat{longitude, latitude};
    auto tile_pos_d = deg2num(lon_lat, zoom);
    cv::Point2i tile_pos = tile_pos_d;

    if (tile_pos_d.x - tile_pos.x < 0.5)
        tile_pos.x -= 1;
    if (tile_pos_d.y - tile_pos.y < 0.5)
        tile_pos.y -= 1;

    if (last_tile_pos_ != tile_pos)
    {
        cv::Mat left;
        cv::Mat right;
        cv::hconcat(get_tile(tile_pos, zoom), get_tile(tile_pos + J, zoom), left);
        cv::hconcat(get_tile(tile_pos + I, zoom), get_tile(tile_pos + I + J, zoom), right);
        cv::vconcat(left, right, last_tile_);

        last_tile_pos_ = tile_pos;
    }

    auto NW = num2deg(tile_pos, zoom);
    auto SE = num2deg(tile_pos + 2 * I + 2 * J, zoom);
    cv::Point2d pos_in_tile{(longitude - NW.x) / (SE.x - NW.x), (NW.y - latitude) / (NW.y - SE.y)};
    cv::Point2i xy = pos_in_tile * size_ * 2. - size_ / 2;

    return last_tile_(cv::Rect(xy, size_));
}

// class Generator(object):
//     def __init__(self, xml_mapnik, tile_cache, size_pxl):
//         self.tile_cache = tile_cache
//         self.size_pxl = Size(size_pxl, size_pxl)
//         self.osm_map = mapnik.Map(size_pxl, size_pxl)
//         mapnik.load_map(self.osm_map, xml_mapnik)

//         EPSG_4326 = '+proj=longlat +ellps=WGS84 +datum=WGS84 +no_defs'
//         self.proj_lonlat = mapnik.Projection(EPSG_4326)
//         self.proj_dest = mapnik.Projection(self.osm_map.srs)
//         self.transform_lonlat_osm = mapnik.ProjTransform(
//             self.proj_lonlat, self.proj_dest)
//         self.thres_cleaning_max = 200
//         self.thres_cleaning_min = 150

//         self.last_tile = None
//         self.last_tile_pos = None
//         self.last_image = None
//         self.last_position = None

//     def srs(self):
//         return self.osm_map.srs

//     def deg2num(self, lat_deg, lon_deg, zoom):
//         lat_rad = math.radians(lat_deg)
//         n = 2.0 ** zoom
//         xtile = (lon_deg + 180.0) / 360.0 * n
//         ytile = (1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n
//         return xtile, ytile

//     def num2deg(self, xtile, ytile, zoom):
//         n = 2.0 ** zoom
//         lon_deg = xtile / n * 360.0 - 180.0
//         lat_rad = math.atan(math.sinh(math.pi * (1 - 2 * ytile / n)))
//         lat_deg = math.degrees(lat_rad)
//         return (lat_deg, lon_deg)

//     def drop_oldest(self):
//         files = glob.glob(os.path.join(self.tile_cache, '*'))
//         files.sort(reverse=True, key=os.path.getctime)
//         if len(files) > self.thres_cleaning_max:
//             drop = files[self.thres_cleaning_min:]
//             print (f'Deleting {drop}')
//             for file in drop:
//                 os.remove(file)

//     def get_tile(self, xtile, ytile, zoom):
//         self.drop_oldest()
//         tilepath = os.path.join(
//             self.tile_cache,
//             f'tile_{self.size_pxl.w}_{self.size_pxl.h}_{zoom}_{xtile}_{ytile}.png')
//         print(f'Fetch {tilepath}')
//         if (not os.path.isfile(tilepath)):
//             print(f'Compute {tilepath}')
//             ymin, xmin = self.num2deg(xtile, ytile, zoom)
//             ymax, xmax = self.num2deg(xtile + 1, ytile + 1, zoom)
//             long_lat_bbox = mapnik.Box2d(xmin, ymin, xmax, ymax)
//             bbox = self.transform_lonlat_osm.forward(long_lat_bbox)
//             self.osm_map.zoom_to_box(bbox)
//             mapnik.render_to_file(self.osm_map, tilepath, 'png24')

//         return cv.imread(tilepath)

//     def produce(self, lat_deg, lon_deg, zoom):
//         xtile, ytile = self.deg2num(lat_deg, lon_deg, zoom)
//         if xtile % 1 < 0.5:
//             xtile -= 1
//         if ytile % 1 < 0.5:
//             ytile -= 1
//         tile_pos = Point(int(xtile), int(ytile), zoom)
//         if self.last_tile_pos != tile_pos:
//             self.last_tile_pos = tile_pos
//             self.last_tile = np.concatenate(
//                     tuple(
//                         np.concatenate(
//                             tuple(
//                                 self.get_tile(
//                                     tile_pos.x + x,
//                                     tile_pos.y + y,
//                                     zoom)
//                                 for x in (0,1)),
//                             axis=1)
//                         for y in (0,1)),
//                     axis = 0)
//             self.last_position = None
//             self.last_image = None

//         w, h = self.size_pxl
//         lat_max, lon_min = self.num2deg(tile_pos.x, tile_pos.y, zoom)
//         lat_min, lon_max = self.num2deg(tile_pos.x + 2, tile_pos.y + 2, zoom)
//         x = (lon_deg - lon_min) / (lon_max - lon_min)
//         xmin = int(x * w * 2 - w / 2)
//         xmax = xmin + w
//         y = (lat_max - lat_deg) / (lat_max - lat_min)
//         ymin = int(y * h * 2 - h / 2)
//         ymax = ymin + h

//         position = Point(xmin, ymin, zoom)
//         if position != self.last_position:
//             self.last_image = self.last_tile[ymin:ymax, xmin:xmax]
//         return self.last_image.copy()
