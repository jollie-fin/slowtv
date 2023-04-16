#pragma once

#include <iosfwd>
#include <filesystem>
#include <optional>
#include <opencv2/core.hpp>
#include <span>
#include "token_type.h"
#include "utilities.h"

using MemoryArea = std::span<const char>;
using Pointer = MemoryArea::iterator;

struct Packet
{
    Packet() = default;
    Packet(MemoryArea);
    cv::Mat accelerometer_;
    cv::Mat gps_position_;
    cv::Mat gyroscope_;
    std::vector<std::vector<std::tuple<Fourcc, std::array<float, 22>>>> faces_;
    enum Value {No_Fix, Fix_2d = 2, fix_3d = 3, LAST};
    uint32_t gps_fix_{No_Fix};
    uint32_t gps_accuracy_{0};
    std::string gps_timestamp_;
    float temperature_{0};
};

class Gpmf
{
    public:
    Gpmf(std::istream &is);
    class Iterator
    {
        public:
        Iterator (std::istream &is);
        Iterator () = default;
        Iterator& operator++();
        Iterator operator++(int);
        bool operator==(const Iterator &) const;
        bool operator!=(const Iterator &) const;
        const Packet &operator*() const;

        protected:
        void increment();
        std::istream *is_{nullptr};
        std::optional<Packet> data_;
    };

    Iterator begin() const;
    Iterator end() const;
    protected:
        std::istream &is_;
};
