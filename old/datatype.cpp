#include <iostream>
#include <string>
#include <cstring>
#include <map>
#include <array>
#include <exception>
#include <limits>
#include <istream>
#include <array>
#include <cctype>
#include <iomanip>
#include "datatype.h"

using namespace OldGPParser;

template <typename ITERATOR>
static bool is_valid_four_cc(ITERATOR it)
{
    for (int i = 0; i < 4; i++)
        if (!std::isalnum(it[i]) && it[i] != ' ')
            return false;

    return true;
}

template <int N>
static std::array<char, N> read_buffer(std::istream &stream)
{
    std::array<char, N> buffer;
    if (!stream.read(buffer.data(), N))
        buffer.fill(0);
    return buffer;
}

template <typename INT, typename ITERATOR>
static INT decode_big_endian(ITERATOR it)
{
    int64_t dest = 0;
    for (int i = 0; i != sizeof(INT); i++)
    {
        dest <<= CHAR_BIT;
        dest |= static_cast<unsigned char>(*it++);
    }

    if (dest > std::numeric_limits<INT>::max())
        dest += 2 * static_cast<int64_t>(std::numeric_limits<INT>::min());

    return dest;
}

template <typename INT>
static INT read_big_endian(std::istream &stream)
{
    auto buffer = read_buffer<sizeof(INT)>(stream);
    return decode_big_endian<INT>(buffer.begin());
}

std::string OldGPParser::label_to_string(Klv::Label label)
{
        char key[4];
        std::memcpy(key, &label, 4);
        std::string out(key, 4);
        std::reverse(out.begin(), out.end());
        return out;
}

std::istream &operator>>(std::istream &stream, Klv &klv)
{
    auto buffer = read_buffer<8>(stream);
    klv.label = static_cast<Klv::Label>(str_to_four_cc(buffer.begin()));
    if (!is_valid_four_cc(buffer.begin()))
        throw std::runtime_error(label_to_string(klv.label) + " is not a valid 4CC");

    klv.format = buffer[4];
    klv.size = buffer[5];
    klv.len = decode_big_endian<uint16_t>(buffer.begin() + 6);
 
    klv.memory_usage = static_cast<uint32_t>(klv.len) * static_cast<uint32_t>(klv.size);
    return stream;
}

std::ostream &operator<<(std::ostream &stream, const Klv &klv)
{
    return stream << "Klv(" << label_to_string(klv.label)
                  << "," << klv.format << "," << static_cast<int>(klv.size)
                  << "," << klv.len << ")";
}

Divisor::Divisor(const char *buffer, int size)
{
    switch(size)
    {
        case 2:
            divisor = decode_big_endian<uint16_t>(buffer);
            break;
        case 4:
            divisor = decode_big_endian<uint32_t>(buffer);
            break;
        default:
            throw std::runtime_error(std::to_string(size) + " is not an acceptable scaling size");

    }
}

Accelerometer::Accelerometer(const char *buffer, const ScalFactor &sf)
{
    for (auto *dest : {&x, &y, &z})
    {
        *dest = static_cast<double>(decode_big_endian<int16_t>(buffer)) / sf[0].divisor;
        buffer += 2;
    }
}

std::ostream &operator<<(std::ostream &out, const Accelerometer &accel)
{
    return out << std::setprecision(8) << accel.x << "," << accel.y << "," << accel.z;
}

Gyroscope::Gyroscope(const char *buffer, const ScalFactor &sf)
{
    for (auto *dest : {&x, &y, &z})
    {
        *dest = static_cast<double>(decode_big_endian<int16_t>(buffer)) / sf[0].divisor;
        buffer += 2;
    }
}

std::ostream &operator<<(std::ostream &out, const Gyroscope &gyro)
{
    return out << std::setprecision(8) << gyro.x << "," << gyro.y << "," << gyro.z;
}

GpsPos::GpsPos(const char *buffer, const ScalFactor &sf)
{
    int index = 0;
    for (auto *dest : {&lat, &lont, &alt, &speed_2d, &speed_3d})
    {
        *dest = static_cast<double>(decode_big_endian<int32_t>(buffer)) / sf[index++].divisor;
        buffer += 4;
    }
}

std::ostream &operator<<(std::ostream &out, const GpsPos &pos)
{
    return out << std::setprecision(8) << pos.lat << "," << pos.lont << "," << pos.alt << "," << pos.speed_2d << "," << pos.speed_3d;
}

GpsFix::GpsFix(const char *buffer)
{
    auto value = decode_big_endian<uint32_t>(buffer);
    if (value >= LAST)
        throw std::runtime_error("Invalid fix");
}

std::ostream &operator<<(std::ostream &out, const GpsFix &fix)
{
    return out << std::setprecision(1) << fix.gps_fix;
}

GpsAccuracy::GpsAccuracy(const char *buffer)
{
    accuracy = decode_big_endian<uint16_t>(buffer);
}

std::ostream &operator<<(std::ostream &out, const GpsAccuracy &accu)
{
    return out << std::setprecision(1) << accu.accuracy;
}

GpsTimestamp::GpsTimestamp(const char *buffer)
{
    time = std::string(buffer, 16);
}

std::ostream &operator<<(std::ostream &out, const GpsTimestamp &timestamp)
{
    return out << timestamp.time;
}

Temperature::Temperature(const char *buffer)
{
    auto tmp = decode_big_endian<uint32_t>(buffer);
    memcpy(&temp, &tmp, 4);
}

std::ostream &operator<<(std::ostream &out, const Temperature &temp)
{
    return out << temp.temp;
}

template <typename CLASS>
std::ostream &operator<<(std::ostream &out, const std::vector<CLASS> &container)
{
    bool first = true;
    for (const auto &elt : container)
    {
        if (!first)
            out << ";";
        first = false;
        out << elt;
    }
    return out;
}

std::ostream &operator<<(std::ostream &out, const Telemetry &telem)
{
    return out << telem.accelerometer << "|" << telem.gyroscope << "|"
               << telem.gps_position << "|" << telem.gps_accuracy << "|"
               << telem.gps_fix << "|" << telem.gps_timestamp << "|"
               << telem.temperature;
    return out;
}
