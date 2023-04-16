#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <set>
#include <sstream>
#include "configuration.h"
#include "map_generator.h"

namespace fs = std::filesystem;
namespace bpt = boost::property_tree;

static fs::path read_path(const bpt::ptree &pt, const std::string &path)
{
    fs::path retval = fs::absolute(pt.get<std::string>(path).data());
    if (!fs::exists(retval))
        throw std::runtime_error("Impossible to find " + retval.string());
    return retval;
}

Configuration::Configuration(fs::path conf_file)
{
    bpt::ptree pt;
    bpt::read_json(conf_file, pt);
    layout_json_ = read_path(pt, "layout");
    osm_xml_ = read_path(pt, "osm.xml");
    osm_srs_ = MapGenerator::read_srs(osm_xml_);
    osm_cache_ = read_path(pt, "osm.cache");
    evenement_json_ = read_path(pt, "evenement");
    elevation_ = read_path(pt, "height_map");

    int id_video = pt.get<int>("video.id");
    auto video_path = read_path(pt, "video.location");


    std::set<fs::path> videos;
    std::string extension;

    {
        std::ostringstream sstr;
        sstr << std::setfill('0') << std::setw(4) << id_video << ".MP4";
        extension = sstr.str();
    }

    for (const auto &video_file : fs::directory_iterator(video_path))
        if (video_file.path().string().ends_with(extension))
            videos.emplace(video_file.path());

    videos_.insert(videos_.end(), videos.begin(), videos.end());
}