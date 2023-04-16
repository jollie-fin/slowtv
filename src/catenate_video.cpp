#include <iostream>
#include <algorithm>
#include <cmath>
#include <tuple>

#include "configuration.h"
#include "catenate_video.h"

namespace fs = std::filesystem;
namespace bp = boost::process;

static std::tuple<int, int, int, int, int> get_info(const fs::path &video)
{
    bp::ipstream pipe_stream;
    bp::child c{
        bp::search_path("ffprobe"), "-v", "error", "-select_streams", "v", "-of", "default=noprint_wrappers=1:nokey=1", "-show_entries", "stream=width,height,r_frame_rate,nb_frames", video.string(),
        bp::std_out > pipe_stream, bp::std_err > bp::null};
    c.wait();

    int width;
    int height;
    std::string num_denom;
    int count;

    pipe_stream >> width >> height >> num_denom >> count;
    auto pos = num_denom.find("/");
    if (pos == std::string::npos)
    {
        return {width, height, std::stoi(num_denom), 1, count};
    }
    else
    {
        auto num = std::stod(num_denom.substr(0, pos));
        auto denom = std::stod(num_denom.substr(pos + 1));
        return {width, height, num, denom, count};
    }
}

CatenateVideo::CatenateVideo(const Configuration &conf)
{
    int cumul_count = 0;
    for (const auto &video : conf.videos_)
    {
        auto [width, height, num, denom, ticks] = get_info(video);
        if (!num_)
            num_ = num;
        else if (num_ != num)
            throw std::runtime_error("Inconsistent num");

        if (!denom_)
            denom_ = denom;
        else if (denom_ != denom)
            throw std::runtime_error("Inconsistent num");

        if (!width_)
            width_ = width;
        else if (width_ != width)
            throw std::runtime_error("Inconsistent width");

        if (!height_)
            height_ = height;
        else if (height_ != height)
            throw std::runtime_error("Inconsistent height");

        cumul_count += ticks;
        list_videos_.emplace_back(video.string(), cumul_count);
    }
    buffer_.resize(height_ * width_ * 3);
    last_frame_ = cv::Mat{height_, width_, CV_8UC3, buffer_.data()};
    next_frame();
}

int CatenateVideo::count() const
{
    return list_videos_.rbegin()->second;
}

int CatenateVideo::fps() const
{
    return std::lround(fps_d());
}

double CatenateVideo::fps_d() const
{
    return static_cast<double>(num_)/ static_cast<double>(denom_);
}

void CatenateVideo::move_to(int sec, bool fast)
{
    sec = std::max(sec, 0);
    position_ = sec * fps();
    int id_video = 0;
    for (const auto &[path, cumul] : list_videos_)
    {
        (void)path;
        std::cout << sec << "->" << position_ << " file " << path << " at " << cumul << std::endl;
        if (cumul <= position_)
            id_video++;
        else
            break;
    }
    if (id_video > 0)
        sec -= list_videos_[id_video - 1].second / fps();
    set_video(id_video, sec, fast);
    next_frame();
}

void CatenateVideo::set_video(int current_video, int offset_sec, bool fast)
{
    std::cout << "Switch to " << current_video << " at " << offset_sec << std::endl;
    current_video_ = current_video;
    fast_ = fast;
    if (current_video >= 0 && current_video < static_cast<int>(list_videos_.size()))
    {
        auto num = num_;
        auto denom = denom_;
        if (fast)
            denom *= fps();
        auto num_denom = std::to_string(num) + "/" + std::to_string(denom);
        auto path = list_videos_[current_video_].first;
        subprocess_str_ = std::make_unique<bp::ipstream>();
        subprocess_ = std::make_unique<bp::child>(
            bp::search_path("ffmpeg"),
            "-ss", std::to_string(offset_sec),
            "-discard", fast_ ? "nokey" : "none",
            "-i",  path.string(),
            "-r", num_denom,
            // "-filter:v", "select='eq(pict_type,I)'",
            "-c:v", "rawvideo", "-pix_fmt", "bgr24", "-f", "rawvideo",
            "-",
            bp::std_out > *subprocess_str_, bp::std_err > bp::null);
    }
    else
    {
        subprocess_str_.reset();
        subprocess_.reset();
    }
}

void CatenateVideo::fast(bool fast)
{
    move_to(position_ / fps(), fast);
}

cv::Mat CatenateVideo::fetch_frame(int position)
{
    if (position < position_ || position >= position_ + 2 * step())
        move_to(position / fps(), fast_);

    if (position >= position_ + step() && position < position_ + 2 * step())
    {
        next_frame();
        position_ += step();
    }
    return last_frame_;
}

void CatenateVideo::next_frame()
{
    if (current_video_ == -1)
    {
        set_video(0, 0, fast_);
        next_frame();
    }

    if (!subprocess_)
        return;

    if (!*subprocess_str_)
    {
        set_video(current_video_ + 1, 0, fast_);
        return;
    }

    subprocess_str_->read(buffer_.data(), width_ * height_ * 3);
}

std::vector<std::filesystem::path> CatenateVideo::what_videos() const
{
    std::vector<std::filesystem::path> retval;
    for (const auto &p : list_videos_)
        retval.emplace_back(p.first);
    return retval;
}