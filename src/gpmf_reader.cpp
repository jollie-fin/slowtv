#include <stdexcept>
#include <iostream>
#include <cassert>
#include <limits>
#include <algorithm>
#include <optional>
#include <cstring>
#include <tuple>
#include <type_traits>
#include <variant>
#include <iomanip>
#include <variant>
#include <cstdint>
#include <vector>
#include <string>
#include <string_view>
#include <array>
#include <charconv>

#include "gpmf_reader.h"
#include "utilities.h"
namespace fs = std::filesystem;

struct Token
{
    Token(MemoryArea area);
    static std::tuple<TokenType, char, uint32_t, uint32_t, uint32_t>
        read_header(MemoryArea area);
    static uint32_t
        payload_size(MemoryArea area);
    uint32_t memory_usage() const;

    static constexpr int header_size_ = 8;
    TokenType type_;
    char format_;
    uint32_t size_;
    uint32_t length_;
    uint32_t payload_size_;
    MemoryArea payload_;
};

struct Payload;

template<typename T, TokenType TYPE>
struct Content
{
    using PayloadType = std::vector<T>;
    static constexpr TokenType token_type_ = TYPE;
    std::vector<PayloadType> payload_;
};

using FaceContent = Content<std::tuple<Fourcc, std::array<float, 22>>, make_tokentype("FACE")>;
using GpsPosContent = Content<std::array<double, 5>, make_tokentype("GPS5")>;
using AcclContent = Content<std::array<double, 3>, make_tokentype("ACCL")>;
using GyroContent = Content<std::array<double, 3>, make_tokentype("GYRO")>;
using GpsFixContent = Content<int, make_tokentype("GPSF")>;
using GpsAccContent = Content<int, make_tokentype("GPSP")>;
using TmpcContent = Content<double, make_tokentype("TMPC")>;
using TimestampContent = Content<std::string_view, make_tokentype("GPSU")>;
using AnyContent = std::tuple<
        AcclContent,
        FaceContent,
        GyroContent,
        GpsFixContent,
        GpsAccContent,
        GpsPosContent,
        TmpcContent,
        TimestampContent>;

struct Payload
{
    Payload(const Token &main_tok);

    int64_t position_{-1};
    int64_t empty_{-1};
    std::array<double, 9> matrix_{};
    std::string_view name_;
    std::string_view unit_;
    std::string_view type_;
    std::string_view orin_;
    std::string_view orio_;
    uint64_t timo_{0};
    std::vector<float> scaling_;
    AnyContent content_;
};

static void split(MemoryArea input, auto&& fonctor)
{
    while (!input.empty())
    {
        Token tok(input);
        input = input.subspan(tok.memory_usage());
        fonctor(tok);
    }
}

static void reverse_memcpy(void *dest, const void *src, std::size_t count)
{
    char *dest_c = reinterpret_cast<char *>(dest);
    const char *src_c = reinterpret_cast<const char *>(src);
    for (std::size_t i = 0; i != count; i++)
        dest_c[count - 1 - i] = src_c[i];
}

template <typename INT>
static INT big_endian(const auto &area)
{
    INT dest = 0;
    reverse_memcpy(&dest, &area[0], sizeof(INT));
    // auto size = sizeof(INT);
    // for (int i = 0; i < sizeof(INT); i++)
    // {
    //     //Thanks to C++20, no UB here \o/
    //     dest |= static_cast<unsigned char>(area[i]) << ((size - i - 1) * 8);
    // }

    return dest;
}

uint32_t Token::payload_size(MemoryArea area)
{
    return std::get<4>(read_header(area));
}

std::tuple<TokenType, char, uint32_t, uint32_t, uint32_t>
    Token::read_header(MemoryArea area)
{
    assert (area.size() >= header_size_);
    auto start = area.begin();
    auto type = make_tokentype(area.data());
    auto format = start[4];
    uint32_t size = big_endian<uint8_t>(start + 5);
    uint32_t length = big_endian<uint16_t>(start + 6);
    return {type, format, size, length, size * length};
}

Token::Token(MemoryArea area)
{
    std::tie(
        type_,
        format_,
        size_,
        length_,
        payload_size_) = read_header(area);

    if (area.size() < memory_usage())
        throw std::runtime_error("Inconsistent file1");

    payload_ = MemoryArea(area.data() + header_size_,
                        payload_size_);
}

uint32_t Token::memory_usage() const
{
    return ((payload_.size() + 3) / 4) * 4 + header_size_;
}

namespace Internal
{
    std::size_t size_from_char(char c)
    {
        switch (c)
        {
            case '\0':
                return 0;
            case 'G':
            case 'U':
                return 16;
            case 'b':
            case 'B':
                return 1;
            case 'j':
            case 'J':
                return 8;
            case 'l':
            case 'L':
                return 4;
            case 's':
            case 'S':
                return 2;
            case 'q':
                return 4;
            case 'Q':
                return 8;
            case 'c':
                return 1;
            case 'd':
                return 8;
            case 'f':
                return 4;
            case 'F':
                return 4;
            default:
                return 0;
        }
    }

    std::size_t size_from_format(std::string_view format)
    {
        std::size_t retval = 0;
        for (auto c : format)
            retval += size_from_char(c);
        return retval;
    }

    template <typename T>
    struct Convert
    {
        T operator()(auto big_payload, std::string_view format, std::size_t &index, int &offset)
        {
            auto payload = big_payload.subspan(offset);
            index++;
            if (index >= format.size())
                index = 0;
            if (payload.empty())
                throw std::runtime_error("Empty payload");
            if constexpr (std::is_same_v<std::string, T> || std::is_same_v<std::string_view, T>)
            {
                offset = big_payload.size();
                switch (format[index])
                {
                    case 'G':
                    case 'U':
                        assert(payload.size() >= 16);
                        return T{payload.data(), 16};
                    case 'c':
                    {
                        //if last byte is null, discard it
                        auto size = payload.size() - !payload.back();
                        return T{payload.data(), size};
                    }
                    default :
                        throw std::runtime_error("Trying to extract string from non string payload");
                }               
            }
            else if constexpr (std::is_scalar_v<T>)
            {
                switch(format[index])
                {
                    case 'b':
                        assert(payload.size() >= 1);
                        offset++;
                        return big_endian<int8_t>(payload);
                    case 'B':
                        assert(payload.size() >= 1);
                        offset++;
                        return big_endian<uint8_t>(payload);
                    case 'j':
                        assert(payload.size() >= 8);
                        offset+=8;
                        return big_endian<int64_t>(payload);
                    case 'J':
                        assert(payload.size() >= 8);
                        offset+=8;
                        return big_endian<uint64_t>(payload);
                    case 'l':
                        assert(payload.size() >= 4);
                        offset+=4;
                        return big_endian<int32_t>(payload);
                    case 'L':
                        assert(payload.size() >= 4);
                        offset+=4;
                        return big_endian<uint32_t>(payload);
                    case 's':
                        assert(payload.size() >= 2);
                        offset+=2;
                        return big_endian<int16_t>(payload);
                    case 'S':
                        assert(payload.size() >= 2);
                        offset+=2;
                        return big_endian<uint16_t>(payload);
                    case 'q':
                        {
                            assert(payload.size() >= 4);
                            offset++;
                            double d = big_endian<int32_t>(payload);
                            d /= (1 << 16);
                            return d;
                        }
                    case 'Q':
                        {
                            assert(payload.size() >= 8);
                            offset++;
                            double d = big_endian<int64_t>(payload);
                            d /= (1L << 32);
                            return d;
                        }
                    case 'c':
                        offset++;
                        return big_endian<char>(payload);
                    case 'd':
                        {
                            double d;
                            assert(payload.size() >= sizeof(d));
                            offset += sizeof(d);
                            reverse_memcpy(&d, payload.data(), sizeof(d));
                            return d;
                        }
                    case 'f':
                        {
                            float d;
                            assert(payload.size() >= sizeof(d));
                            offset += sizeof(d);
                            reverse_memcpy(&d, payload.data(), sizeof(d));
                            return d;
                        }
                    case 'F':
                        assert(payload.size() >= 4);
                        offset += 4;
                        return make_fourcc(payload.data());
                    default:
                    {
                        char char_format{format[index]};
                        throw std::runtime_error("Impossible to read " + std::string(1, char_format) + " as " + std::string(typeid(T).name()));
                    }
                }                
            }
            else
            {
                T t{};
                int a = t;
            }
            return {};
        }
    };

    template <typename T, std::size_t N>
    struct Convert<std::array<T, N>>
    {
        std::array<T, N> operator()(auto payload, std::string_view format, std::size_t &index, int &offset)
        {
            std::array<T, N> retval;
            for (auto &dest : retval)
                dest = Convert<T>{}(payload, format, index, offset);
            return retval;
        }
    };

    template <typename T>
    struct Convert<std::vector<T>>
    {
        std::vector<T>
            operator()(auto payload, std::string_view format, std::size_t &index, int &offset)
        {
            std::vector<T> retval;
            auto index_bound = format.size() - 1;
            auto min_size = size_from_format(format);
            while (offset + min_size <= payload.size())
            {
                index = index_bound;
                auto value = Convert<T>{}(payload, format, index, offset);
                retval.emplace_back(value);
                if (index != index_bound)
                    throw std::runtime_error("Inconsistent type with " + std::string(format));
            }
            return retval;
        }
    };

    template <>
    struct Convert<std::tuple<>>
    {
        std::tuple<> operator()(auto, auto, auto&, auto&)
        {
            return {};
        }
    };

    template <typename Arg, typename... Args>
    struct Convert<std::tuple<Arg, Args...>>
    {
        std::tuple<Arg, Args...>
            operator()(auto payload, std::string_view format, std::size_t &index, int &offset)
        {
            auto value =
                Convert<Arg>{}(payload, format, index, offset);

            auto tuple =
                Convert<std::tuple<Args...>>{}
                    (payload, format, index, offset);

            return std::tuple_cat(std::make_tuple(value), tuple);
        }
    };
}

template <typename T>
static T convert(const Token &tok, std::string_view alternative_format = "")
{
    std::size_t index = -1;
    int offset = 0;
    std::string_view format(&tok.format_, 1);
    if (tok.format_ == '?')
        format = alternative_format;
    if (format.empty())
        throw std::runtime_error("Invalid type");
    return Internal::Convert<T>{}(tok.payload_, format, index, offset);
}

namespace Internal
{
    template <typename T>
    struct Normalize
    {
        void operator()(T &content, const auto &scaling, std::string_view, std::size_t &index)
        {
            if constexpr (std::is_floating_point_v<T>)
                if (!scaling.empty())
                {
                    content /= scaling[index];
                    index++;
                    if (index >= scaling.size())
                        index = 0;
                }
        }
    };

    template <typename T, std::size_t N>
    struct Normalize<std::array<T, N>>
    {
        void operator()(std::array<T, N> &dest, const auto &scaling, std::string_view permutation, std::size_t &index)
        {
            for (auto &elt : dest)
                Normalize<T>{}(elt, scaling, permutation, index);
            if (!permutation.empty())
            {
                if (permutation.size() != N)
                    throw std::runtime_error("Invalid Orientation");
                std::array<T, N> temp;
                for (std::size_t i = 0; i < N; i++)
                {
                    int position = std::tolower(permutation[i]) - 'a';
                    temp[position] = std::islower(permutation[i]) ? -dest[i] : dest[i];
                }
                dest = temp;
            }
        }
    };

    template <typename... Args>
    struct Normalize<std::tuple<Args...>>
    {
        void operator()(std::tuple<Args...> &dest, const auto &scaling, std::string_view permutation, std::size_t &index)
        {
            auto normalize_wrapper = [&](auto &x)
            {
                using Type = std::remove_cvref_t<decltype(x)>;
                Normalize<Type>{}(x, scaling, permutation, index);
            };

            std::apply([&](auto& ...x){(..., normalize_wrapper(x));}, dest);
        }
    };

    template <typename T>
    struct Normalize<std::vector<T>>
    {
        void operator()(std::vector<T> &content, const auto &scaling, std::string_view permutation, std::size_t &index)
        {
            for (auto &elt : content)
            {
                index = 0;
                Normalize<T>{}(elt, scaling, permutation, index);
            }
        }
    };
}

static void normalize(auto &dest, const auto &scaling, std::string_view orin, std::string_view orio)
{
    std::string permutation;

    assert(orin.size() == orio.size());
    for (auto ci : orin)    {
        if (!ci)
            continue;
        bool found = false;
        for (std::size_t i = 0; i != orio.size(); i++)
        {
            auto co = orio[i];
            if (!co)
                continue;
            auto new_char = 'a' + i;
            if (std::isupper(ci))
                new_char = std::toupper(new_char);
            if (std::tolower(ci) == std::tolower(co))
            {
                found = true;
                permutation += new_char;
            }
        }
        if (!found)
        {
            throw std::runtime_error("Impossible to permute " + std::string(orin) + " in " + std::string(orio));
        }
    }

    std::size_t index = 0;
    std::string_view orin_view(permutation.data(), permutation.size());
    using Type = std::remove_cvref_t<decltype(dest)>;
    Internal::Normalize<Type>{}(dest, scaling, orin_view, index);
}

static void convert_and_assign(const auto &obj, const auto &child, auto &dest)
{
    dest = convert<std::remove_cvref_t<decltype(dest)>>(child, obj.type_);
}

template <typename T>
static void convert_and_assign_tuple(const auto &obj, const auto &child, const auto &scaling, const auto &orin, const auto &orio, AnyContent &dest)
{
    if (child.type_ != T::token_type_)
        throw std::runtime_error("Internal error");

    auto &pval = std::get<T>(dest);
    auto tmpval = convert<typename T::PayloadType>(child, obj.type_);
    normalize(tmpval, scaling, orin, orio);
    pval.payload_.emplace_back(tmpval);
}

#define MAKE_CASE(name, dest) \
case make_tokentype(name): \
    convert_and_assign(*this, tok, dest); \
    break;

#define MAKE_CASE_IGNORE(name) \
case make_tokentype(name): \
    break;

#define MAKE_CASE_CONTENT(dest, scaling, orin, orio, type) \
case type::token_type_: \
    convert_and_assign_tuple<type>(*this, tok, scaling, orin, orio, dest); \
    orin = ""; \
    orio = ""; \
    scaling.clear(); \
    break;

static void patch_orio(std::string_view &orio)
{
    static bool is_xyz[256];
    is_xyz['\0'] = true;
    is_xyz['x'] = true;
    is_xyz['y'] = true;
    is_xyz['z'] = true;
    is_xyz['X'] = true;
    is_xyz['Y'] = true;
    is_xyz['Z'] = true;

    for (auto c : orio)
        if (!is_xyz[static_cast<unsigned char>(c)])
            return;
    orio = "XYZ";
}

Payload::Payload(const Token &main_tok)
{
    split(main_tok.payload_, [this](const Token &tok)
        {
            switch (tok.type_)
            {
                MAKE_CASE("EMPT", position_);
                MAKE_CASE("TSMP", empty_);
                MAKE_CASE("MTRX", matrix_);
                MAKE_CASE("STNM", name_);
                MAKE_CASE("SIUN", unit_);
                MAKE_CASE("UNIT", unit_);
                MAKE_CASE("TYPE", type_);
                MAKE_CASE("SCAL", scaling_);
                MAKE_CASE("TIMO", timo_);
                MAKE_CASE("ORIN", orin_);
                MAKE_CASE("ORIO", orio_);
                MAKE_CASE_CONTENT(content_, scaling_, orin_, orio_, AcclContent);
                MAKE_CASE_CONTENT(content_, scaling_, orin_, orio_, FaceContent);
                MAKE_CASE_CONTENT(content_, scaling_, orin_, orio_, GyroContent);
                MAKE_CASE_CONTENT(content_, scaling_, orin_, orio_, GpsPosContent);
                MAKE_CASE_CONTENT(content_, scaling_, orin_, orio_, GpsFixContent);
                MAKE_CASE_CONTENT(content_, scaling_, orin_, orio_, GpsAccContent);
                MAKE_CASE_CONTENT(content_, scaling_, orin_, orio_, TmpcContent);
                MAKE_CASE_CONTENT(content_, scaling_, orin_, orio_, TimestampContent);
            case TokenType::INVALID:
                throw std::runtime_error("Error decoding token");
                break;
            default:
                break;
            }
            if (tok.type_ == make_tokentype("ORIO"))
                patch_orio(orio_);
        });
}

template<class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
template<class... Ts> overloaded(Ts...) -> overloaded<Ts...>;

template <typename T>
struct SizeArray
{
};

template <typename T, std::size_t N>
struct SizeArray<std::array<T, N>>
{
    static constexpr std::size_t size = N;
};

template<typename T>
static constexpr std::size_t sizeof_array = SizeArray<T>::size;

Packet::Packet(MemoryArea area)
{
    split(area,
        [&, this](const Token &tok)
        {
            if (tok.format_ != '\0')
                return;

            Payload payload(tok);

            auto append_container = [&](auto &dest, const auto &data)
            {
                dest.insert(dest.end(), data.begin(), data.end());
            };

            auto matricize = [&](cv::Mat &retval, const auto &data)
            {
                using T = std::remove_cvref_t<decltype(data[0][0])>;
                constexpr std::size_t N = sizeof_array<T>;
                int length = 0;
                for (const auto &t : data)
                    length += t.size();

                cv::Mat tmp(N, length, cv_type<double>, cv::Scalar(0.));
                for (int i = 0; i != N; i++)
                {
                    auto *ptr = tmp.ptr<double>(i);
                    std::size_t j = 0;
                    for (const auto &t : data)
                        for (const auto &v : t)
                            ptr[j++] = v[i];
                }

                if (retval.empty())
                    retval = tmp;
                else if (!tmp.empty())
                    cv::hconcat(retval, tmp, retval);

                return retval;
            };

            auto extract_value = [&](auto &dest, const auto &data)
            {
                if (!data.empty() && !data.back().empty())
                    dest = data.back().back();
            };

            auto visitor =
                overloaded(
                    [&](const GpsPosContent& content){
                        matricize(gps_position_, content.payload_);},
                    [&](const AcclContent& content){
                        matricize(accelerometer_, content.payload_);},
                    [&](const GyroContent& content){
                        matricize(gyroscope_, content.payload_);},
                    [&](const GpsFixContent& content){
                        extract_value(gps_fix_, content.payload_);},
                    [&](const GpsAccContent& content){
                        extract_value(gps_accuracy_, content.payload_);},
                    [&](const TmpcContent& content){
                        extract_value(temperature_, content.payload_);},
                    [&](const TimestampContent& content){
                        extract_value(gps_timestamp_, content.payload_);},
                    [&](const FaceContent& content){
                        append_container(faces_, content.payload_);},
                    [&](auto){}
                );

            std::apply([&](auto& ...x){(..., visitor(x));}, payload.content_);
        }
    );
}


Gpmf::Gpmf(std::istream &is)
    : is_(is)
{
}

Gpmf::Iterator Gpmf::begin() const
{
    return {is_};
}

Gpmf::Iterator Gpmf::end() const
{
    return {};
}

Gpmf::Iterator::Iterator(std::istream &is)
    : is_{&is}
{
    increment();
}

void Gpmf::Iterator::increment()
{
    assert(is_);
    constexpr auto header_size = Token::header_size_;
    char header[header_size];
    MemoryArea header_span(header, header_size);
    if (!is_->read(header, header_size)) {
        is_ = nullptr;
        data_ = {};
        return;
    }

    auto payload_size = Token::payload_size(header_span);
    payload_size = ((payload_size + 3) / 4) * 4;
    std::vector<char> area(header_size + payload_size);
    std::copy(header_span.begin(), header_span.end(), area.begin());

    if (!is_->read(area.data() + header_size, payload_size)) {
        is_ = nullptr;
        data_ = {};
        return;
    }

    Token tok(MemoryArea(area.data(), area.size()));
    if (tok.type_ != make_tokentype("DEVC"))
        throw std::runtime_error("Inconsistent file");
    data_ = Packet{tok.payload_};
}

const Packet &Gpmf::Iterator::operator*() const
{
    assert(data_);
    return *data_;
}

Gpmf::Iterator &Gpmf::Iterator::operator++()
{
    increment();
    return *this;
}

Gpmf::Iterator Gpmf::Iterator::operator++(int)
{
    auto previous = *this;
    increment();
    return previous;
}

bool Gpmf::Iterator::operator==(const Gpmf::Iterator &other) const
{
    if (!data_ && !other.data_)
        return true;
    if (!data_ || !other.data_)
        return false;
    return is_ == other.is_;
}

bool Gpmf::Iterator::operator!=(const Gpmf::Iterator &other) const
{
    return !(*this == other);
}

