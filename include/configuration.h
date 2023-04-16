#pragma once

#include <filesystem>
#include <vector>
#include <string>

struct Configuration
{
    Configuration(std::filesystem::path conf_file);

    std::filesystem::path layout_json_;
    std::filesystem::path osm_xml_;
    std::string osm_srs_;
    std::filesystem::path osm_cache_;
    std::vector<std::filesystem::path> videos_;
    std::filesystem::path evenement_json_;
    std::filesystem::path elevation_;
};
