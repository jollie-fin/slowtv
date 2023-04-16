#ifndef UTILITIES_H
#define UTILITIES_H

#include <concepts>
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>

template <typename T> concept arithmetic = std::integral<T> || std::floating_point<T>; 

template<typename T>
struct CvType
{
};

template<>
struct CvType<uint8_t>
{
    static constexpr int value = CV_8U;
};

template<>
struct CvType<int8_t>
{
    static constexpr int value = CV_8S;
};

template<>
struct CvType<uint16_t>
{
    static constexpr int value = CV_16U;
};

template<>
struct CvType<int16_t>
{
    static constexpr int value = CV_16S;
};

template<>
struct CvType<int32_t>
{
    static constexpr int value = CV_32S;
};

template<>
struct CvType<float>
{
    static constexpr int value = CV_32F;
};

template<>
struct CvType<double>
{
    static constexpr int value = CV_64F;
};

template<typename T>
constexpr int cv_type = CvType<T>::value;

template<arithmetic T, arithmetic A>
static cv::Point_<T> operator+(cv::Point_<T> retval, A other)
{
    retval.x += other;
    retval.y += other;
    return retval;
}

template<arithmetic T, arithmetic A>
static cv::Point_<T> operator+(A other, cv::Point_<T> retval)
{
    retval.x += other;
    retval.y += other;
    return retval;
}

template<arithmetic T, arithmetic A>
static cv::Point_<T> operator-(cv::Point_<T> retval, A other)
{
    retval.x -= other;
    retval.y -= other;
    return retval;
}

template<arithmetic T, arithmetic A>
static cv::Point_<T> operator-(A other, cv::Point_<T> retval)
{
    retval.x = other - retval.x;
    retval.y = other - retval.y;
    return retval;
}

template<arithmetic T, arithmetic A>
static cv::Point_<T> operator*(cv::Point_<T> retval, cv::Size_<A> other)
{
    retval.x *= other.width;
    retval.y *= other.height;
    return retval;
}

template<arithmetic T, arithmetic A>
static cv::Point_<T> operator*(cv::Size_<A> other, cv::Point_<T> retval)
{
    retval.x *= other.width;
    retval.y *= other.height;
    return retval;
}

template<arithmetic T, arithmetic A>
static cv::Size_<T> &operator*=(cv::Size_<T> &retval, A other)
{
    retval.width *= other;
    retval.height *= other;
    return retval;
}

// template<arithmetic T, arithmetic A>
// static cv::Size_<T> operator/(cv::Size_<T> retval, A other)
// {
//     retval.width /= other;
//     retval.height /= other;
//     return retval;
// }

template<arithmetic T, arithmetic A>
static cv::Point_<T> operator+(cv::Point_<T> retval, cv::Size_<A> other)
{
    retval.x += other.width;
    retval.y += other.height;
    return retval;
}

template<arithmetic T, arithmetic A>
static cv::Point_<T> operator+(cv::Size_<A> other, cv::Point_<T> retval)
{
    retval.x += other.width;
    retval.y += other.height;
    return retval;
}

template<arithmetic T, arithmetic A>
static cv::Point_<T> operator-(cv::Point_<T> retval, cv::Size_<A> other)
{
    retval.x -= other.width;
    retval.y -= other.height;
    return retval;
}

template<arithmetic T, arithmetic A>
static cv::Point_<T> operator-(cv::Size_<A> other, cv::Point_<T> retval)
{
    retval.x = other.width - retval.x;
    retval.y = other.height - retval.y;
    return retval;
}

#endif
