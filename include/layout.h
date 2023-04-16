#pragma once

#include <opencv2/core/types.hpp>
#include <filesystem>
#include <variant>
#include <vector>
#include <map>
#include <functional>

struct ImageGenerator
{
    using Generator = std::function<cv::Mat(int)>;
    Generator generator_;
    std::string internal_;
    std::vector<std::string> parameters_;
    cv::Size size_;
    cv::Mat operator()(int position) const;
    bool empty() const {return !generator_;}
};

struct Box
{
    cv::Point dest_center_;
    double scaling_{1.};

    ImageGenerator image_;
    ImageGenerator mask_;

    void render(cv::Mat output, int time, double angle_deg) const;
};

struct Layout
{
    Layout() = default;
    Layout(std::filesystem::path path);

    cv::Size output_bounds_;

    cv::Mat render(int time, std::map<std::string_view, int> angles_deg = {}) const;
    std::vector<const ImageGenerator *> internal_images(std::string_view name) const;
    std::vector<ImageGenerator *> internal_images(std::string_view name);

    protected:
        std::vector<Box> boxs_;
};

