#pragma once

#include <set>
#include <filesystem>

std::set<std::filesystem::path> list_videos(const std::filesystem::path &directory, int id_video);
