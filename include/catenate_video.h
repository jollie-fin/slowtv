#pragma once
#include <opencv2/imgproc.hpp>
#include <boost/process.hpp>
#include <filesystem>
#include <memory>
#include <vector>
#include "configuration.h"

class CatenateVideo
{
    public:
    CatenateVideo(const Configuration &);
    int count() const;
    int fps() const;
    double fps_d() const;

    bool fast() const {return fast_;}
    void fast(bool);
    cv::Mat fetch_frame(int position);
    int step() const {return fast_ ? fps() : 1;}

    std::vector<std::filesystem::path> what_videos() const;
    int width() const {return width_;}
    int height() const {return height_;}

    protected:
    void next_frame();
    void move_to(int sec, bool fast = false);
    void set_video(int current_video, int offset_sec = 0, bool fast = false);

    std::vector<std::pair<std::filesystem::path, int>> list_videos_;
    std::unique_ptr<boost::process::child> subprocess_;
    std::unique_ptr<boost::process::ipstream> subprocess_str_;
    int num_{0};
    int denom_{0};
    int width_{0};
    int height_{0};
    int current_video_{-1};
    bool fast_{false};

    std::vector<char> buffer_;
    cv::Mat last_frame_;
    int position_{0};
};
