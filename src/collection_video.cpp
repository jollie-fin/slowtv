#include <string>
#include <sstream>
#include <filesystem>
#include <set>

#include "collection_video.h"

namespace fs = std::filesystem;

std::set<fs::path> list_videos(const fs::path &directory, int id_video)
{
    std::set<fs::path> retval;
    std::string extension;

    {
        std::ostringstream sstr;
        sstr << std::setfill('0') << std::setw(4) << id_video << ".MP4";
        extension = sstr.str();
    }

    for (const auto &video_file : fs::directory_iterator(directory))
        if (video_file.path().string().ends_with(extension))
            retval.emplace(video_file.path());
    return retval;
}
