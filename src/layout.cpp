#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/matx.hpp>
#include <iostream>
#include "layout.h"
#include "constants.h"
#include "utilities.h"
#include <opencv2/highgui.hpp>

namespace bpt = boost::property_tree;

static const cv::Size invalid_size{-1,-1};

static cv::Matx33d translate(cv::Size direction)
{
    auto retval = cv::Matx33d::eye();
    retval(0,2) = direction.width;
    retval(1,2) = direction.height;
    return retval;
}

static cv::Matx33d rotate(double angle)
{
    angle *= pi / 180.;
    auto c = std::cos(angle);
    auto s = std::sin(angle);
    auto retval = cv::Matx33d::eye();
    retval(0,0) = c;
    retval(1,1) = c;
    retval(1,0) = s;
    retval(0,1) = -s;
    return retval;
}

static cv::Matx33d scale(double scale)
{
    auto retval = cv::Matx33d::eye();
    retval(0,0) = scale;
    retval(1,1) = scale;
    return retval;
}

// static cv::Matx23d extract_affine(const cv::Matx33d &mat)
// {
//     cv::Matx23d retval;
//     for (int row = 0; row != 2; row++)
//         for (int col = 0; col != 3; col++)
//             retval(row, col) = mat(row, col);
//     return retval;
// }

static cv::Rect centered_rect(cv::Point center, cv::Size bounds)
{
    return {center - bounds / 2, bounds};
}

static cv::Rect bounding_rect(cv::Size size)
{
    return cv::Rect{{0,0}, size};
}

static cv::Rect bounding_rect(const cv::Mat &img)
{
    return bounding_rect(img.size());
}

static cv::Point rect_center(cv::Size size)
{
    return cv::Point{} + size / 2;
}

// static cv::Point rect_center(const cv::Rect &rect)
// {
//     return rect.tl() + rect_center(rect.size());
// }

static cv::Point rect_center(const cv::Mat &img)
{
    return rect_center(img.size());
}

static bool is_valid(cv::Size size)
{
    return size.width > 0  && size.height > 0;
}

static bool is_invalid(cv::Size size)
{
    return !is_valid(size);
}

static int rect_right(const cv::Rect &rect)
{
    return rect.width + rect.x;
}

static int rect_bottom(const cv::Rect &rect)
{
    return rect.height + rect.y;
}

static void bounds_roi(const std::vector<std::pair<const cv::Mat *, cv::Rect *>> &input)
{
    int trim_left = 0;
    int trim_top = 0;
    int trim_right = 0;
    int trim_bottom = 0;

    for (auto [mat, roi] : input)
    {
        auto new_roi = *roi & bounding_rect(*mat);
        trim_left   = std::max(trim_left   , new_roi.x   - roi->x        );
        trim_top    = std::max(trim_top    , new_roi.y   - roi->y        );
        trim_right  = std::max(trim_right  , rect_right (*roi) - rect_right (new_roi));
        trim_bottom = std::max(trim_bottom , rect_bottom(*roi) - rect_bottom(new_roi));
    }

    for (auto [mat, roi] : input)
    {
        roi->x      += trim_left;
        roi->y      += trim_top;
        roi->width  -= trim_right + trim_left;
        roi->height -= trim_top + trim_bottom;
        roi->width = std::max(roi->width, 0);
        roi->height = std::max(roi->height, 0);
    }
}


static int get_px(const bpt::ptree &tree, auto name, int span, int default_value)
{
    auto patch = [&](int value)->int {if (value < 0) value += span; return value;};

    if (auto property = tree.get_optional<int>(name); property)
        return patch(*property);

    if (auto property = tree.get_optional<double>(name); property)
        return patch(*property * span);

    if (auto property = tree.get_optional<std::string>(name); property)
    {
        if (property->back() != '%')
            throw std::runtime_error("Impossible to interpret '" + *property + "' as a pixel position");

        *property = property->substr(0, property->size() - 1);
        return patch(std::stod(*property) / 100. * span);
    }

    return default_value;
}

static cv::Size read_size(const bpt::ptree &tree, cv::Size span, cv::Size default_size)
{
    cv::Size retval;
    retval.width = get_px(tree, "width", span.width, default_size.width);
    retval.height = get_px(tree, "height", span.height, default_size.height);
    return retval;
}

static cv::Size read_size_opt(const bpt::ptree &tree, const char *name, cv::Size span, cv::Size default_size)
{
    auto size_block_opt = tree.get_child_optional(name);
    if (!size_block_opt)
        return default_size;

    return read_size(*size_block_opt, span, default_size);
}

static std::vector<std::string_view> split_tokens(std::string_view input, std::size_t max_nb_tokens = std::string::npos)
{
    std::vector<std::string_view> retval;
    std::size_t position = 0;
    
    while (position != std::string::npos)
    {
        auto next_sep = input.find(':', position);
        if (retval.size() == max_nb_tokens - 1)
            next_sep = std::string::npos;
        auto token = input.substr(position, next_sep);
        position = next_sep;
        if (position != std::string::npos)
            position++;
        retval.emplace_back(token);
    }
    return retval;
}

static std::pair<std::vector<cv::Mat>, int> read_image_seq(std::string_view path)
{
    std::vector<cv::Mat> retval;
    int step = 1;

    auto tokens = split_tokens(path);
    if (retval.size() == 1)
    {
        retval.emplace_back(cv::imread(std::string(tokens.back())));
        return {retval, step};
    }

    step = std::stoi(std::string(tokens.front()));
    std::transform(
        tokens.begin() + 1,
        tokens.end(),
        std::back_inserter(retval),
        [&](std::string_view name)
            {
                return cv::imread(std::string(name));
            });

    return {retval, step};
}

static ImageGenerator read_image(const bpt::ptree &tree, const char *name, cv::Size output_bounds, cv::Size default_size, bool is_mask)
{
    ImageGenerator retval;
    retval.size_ = default_size;
    auto image_block_opt = tree.get_child_optional(name);
    if (!image_block_opt)
        return retval;

    const bpt::ptree &image_block = *image_block_opt;

    retval.size_ = read_size(image_block, output_bounds, default_size);

    std::string source = image_block.get<std::string>("src");
    auto type_dest = split_tokens(source, 2);

    if (type_dest.size() != 2)
        throw std::runtime_error("Imposible to find ':' in " + source);
    auto type = type_dest.front();
    auto dest = type_dest.back();

    if (type == "shape")
    {
        bool is_circle = dest == "circle";
        bool is_rectangle = dest == "rectangle";

        if (retval.size_.width < 0 || retval.size_.height < 0)
            throw std::runtime_error("Missing width, height for shape : '" + source + "'");
        if (!is_circle && !is_rectangle)
            throw std::runtime_error("Invalid shape : '" + source + "'");
        if (retval.size_.width != retval.size_.height && is_circle)
            throw std::runtime_error("Non square shape : '" + source + "'");

        cv::Mat tmp_img{retval.size_, is_mask ? CV_8U : CV_8UC3, cv::Scalar(0)};
        if (is_circle)
            cv::circle(tmp_img, rect_center(retval.size_), retval.size_.width / 2, cv::Scalar(255), cv::FILLED);
        else
            tmp_img = cv::Scalar(255);

        retval.generator_ = [=](int){return tmp_img;};
        return retval;
    }
    if (type == "internal")
    {
        auto tokens = split_tokens(dest);
        retval.internal_ = tokens.front();
        retval.parameters_.insert(retval.parameters_.end(), tokens.begin() + 1, tokens.end());
        return retval;
    }
    if (type == "image")
    {
        auto [tmp_imgs, step] = read_image_seq(dest);
        if (is_valid(retval.size_) && retval.size_ != tmp_imgs.front().size())
        {
            std::ostringstream sstr;
            sstr << tmp_imgs.front().size() << " and " << retval.size_ << " are incompatble";
            throw std::runtime_error(sstr.str());
        }

        retval.generator_ = [=](int position){return tmp_imgs[(position / step) % tmp_imgs.size()];};

        return retval;
    }
    throw std::runtime_error("Unknown type : '" + source + "'");
}
  
static Box make_box(const bpt::ptree &tree, cv::Size output_bounds)
{
    Box retval;
    retval.image_ = read_image(tree, "image", output_bounds, {-1, -1}, false);
    retval.mask_ = read_image(tree, "mask", output_bounds, retval.image_.size_, true);

    cv::Size dest_size = read_size_opt(tree, "destination", output_bounds, retval.mask_.size_);

    if (is_valid(retval.mask_.size_) && is_valid(dest_size) && retval.mask_.size_ != dest_size)
        throw std::runtime_error("Impossible to mask");

    retval.mask_.size_  = dest_size;

    auto image_center = rect_center(output_bounds);
    retval.scaling_ = get_px(tree, "destination.scaling", 1., 1.);
    retval.dest_center_.x = get_px(tree, "destination.centerx", output_bounds.width, image_center.x);
    retval.dest_center_.y = get_px(tree, "destination.centery", output_bounds.height, image_center.y);

    return retval;
}

// static cv::Matx23d operator*(cv::Matx23d first, cv::Matx23d second)
// {
//     auto mul1 = cv::Matx33d::eye();
//     auto mul2 = cv::Matx33d::eye();
    
//     for (int row = 0; row != 2; row++)
//         for (int col = 0; col != 3; col++)
//         {
//             mul1(row, col) = first(row, col);
//             mul2(row, col) = second(row, col);
//         }

//     cv::Matx33d mul_r = mul1 * mul2;
//     cv::Matx23d retval;

//     for (int row = 0; row != 2; row++)
//         for (int col = 0; col != 3; col++)
//             retval(row, col) = mul_r(row, col);
//     retval *= 1. / mul_r(2,2);
//     return retval;
// }

void Box::render(cv::Mat output, int time, double angle_deg) const
{
    if (image_.empty())
        throw std::runtime_error("Uninitialized ImageGenerator:" + image_.internal_);
    auto input = image_.generator_(time);
    auto image_size = image_.size_;
    if (is_invalid(image_size))
        image_size = input.size();
    else if (image_size != input.size())
            throw std::runtime_error("Mismatch size in Box::copy");

    auto dest_size = mask_.size_;
    if (is_invalid(dest_size))
        dest_size = image_size;

    if (angle_deg == 0. && scaling_ == 1.)
    {
        auto extract_from_input = centered_rect(rect_center(image_size), dest_size);
        auto extract_from_mask = bounding_rect(dest_size);
        auto extract_from_output = centered_rect(dest_center_, dest_size);

        if (mask_.empty())
        {
            bounds_roi({{&input, &extract_from_input}, {&output, &extract_from_output}});
            input(extract_from_input).copyTo(output(extract_from_output));
        }
        else
        {
            auto mask = mask_.generator_(time);
            bounds_roi({{&input, &extract_from_input}, {&output, &extract_from_output}, {&mask, &extract_from_mask}});
            input(extract_from_input).copyTo(output(extract_from_output), mask(extract_from_mask));
        }
    }
    else
    {
        auto rot = scale(scaling_) * rotate(angle_deg) * translate(image_size / -2);
        auto extract_from_output = centered_rect(dest_center_, dest_size);

        if (mask_.empty())
        {
            auto new_extract_from_output = extract_from_output & bounding_rect(output);
            auto offset = new_extract_from_output.tl() - extract_from_output.tl();
            extract_from_output = new_extract_from_output;
            rot = translate(dest_size / 2 - offset) * rot;
            cv::warpPerspective(input, output(extract_from_output), rot, extract_from_output.size(), cv::INTER_CUBIC, cv::BORDER_TRANSPARENT);
        }
        else
        {
            auto mask = mask_.generator_(time);
            auto rotated_bounds = cv::RotatedRect({0, 0}, image_size, angle_deg);
            rotated_bounds.size *= scaling_;
            auto rotated_size = rotated_bounds.boundingRect().size();
            cv::Mat rotated_input;

            rot = translate(rotated_size / 2) * rot;
            cv::Mat rotated_mask;
            cv::Mat white{image_size, CV_8U, cv::Scalar(255)};
            cv::warpPerspective(input, rotated_input, rot, rotated_size, cv::INTER_CUBIC, cv::BORDER_TRANSPARENT);
            cv::warpPerspective(white, rotated_mask, rot, rotated_size, cv::INTER_NEAREST, cv::BORDER_TRANSPARENT);

            auto extract_from_input = centered_rect(rect_center(rotated_input), dest_size);
            auto extract_from_mask  = bounding_rect(dest_size);

            bounds_roi({{&rotated_input, &extract_from_input}, {&mask, &extract_from_mask}, {&output, &extract_from_output}});
            cv::bitwise_and(rotated_mask(extract_from_input), mask(extract_from_mask), rotated_mask(extract_from_input));

            rotated_input(extract_from_input).copyTo(output(extract_from_output), rotated_mask(extract_from_input));
        }
    }
}

Layout::Layout(std::filesystem::path path)
{
    bpt::ptree tree;
    bpt::read_json(path, tree);
    output_bounds_ = make_box(tree.get_child("output"), {}).image_.size_;
    if (is_invalid(output_bounds_))
        throw std::runtime_error("Output size has not been filled");

    for (const auto &child : tree.get_child("objects"))
        boxs_.emplace_back(make_box(child.second, output_bounds_));

    for (const auto *img : internal_images("map"))
        if (img->size_.width != img->size_.height)
            throw std::runtime_error("Only square maps can be generated. Please use mask instead");
}

cv::Mat Layout::render(int time, std::map<std::string_view, int> angles_deg) const
{
    cv::Mat output{output_bounds_, CV_8UC3, cv::Scalar(255, 255, 0)};
    for (const auto &box : boxs_)
    {
        auto angle_deg = 0.;
        if (auto it = angles_deg.find(box.image_.internal_)
            ; it != angles_deg.end())
            angle_deg = it->second;
        box.render(output, time, angle_deg);
    }
    return output;
}

std::vector<ImageGenerator *> Layout::internal_images(std::string_view name)
{
    assert(!name.empty());
    std::vector<ImageGenerator *> retval;
    for (auto &box : boxs_)
        for (auto *img : {&box.image_, &box.mask_})
            if (img->internal_ == name)
                retval.emplace_back(img);
    return retval;

}

std::vector<const ImageGenerator *> Layout::internal_images(std::string_view name) const
{
    assert(!name.empty());
    std::vector<const ImageGenerator *> retval;
    for (auto &box : boxs_)
        for (auto *img : {&box.image_, &box.mask_})
            if (img->internal_ == name)
                retval.emplace_back(img);
    return retval;

}
