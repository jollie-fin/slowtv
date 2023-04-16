#pragma once

#include <cstdint>
#include <algorithm>
#include <istream>
#include <ostream>
#include <string_view>
#include <vector>
#include <climits>

namespace OldGPParser
{
    using DynBuffer = std::vector<char>;

    constexpr uint32_t str_to_four_cc(const char *str)
    {
        uint32_t four_cc = 0;
        for (int i = 0; i < 4; i++)
        {
            four_cc <<= CHAR_BIT;
            four_cc |= static_cast<uint8_t>(str[i]);
        }
        return four_cc;
    }

    // KLV - Go-Pro-Metadata-Format Key-Length-Value
    struct Klv
    {
        char format;   // Format of the data
        uint8_t size;  // Size of the object
        uint16_t len; // Number of object
        uint32_t memory_usage;

        enum
        {
            Empty = str_to_four_cc("EMPT"),
            StreamName = str_to_four_cc("STNM"),
            ScalFactor = str_to_four_cc("SCAL"),
            Dvid = str_to_four_cc("DVID"),
            GpsPos = str_to_four_cc("GPS5"),
            Accelerometer = str_to_four_cc("ACCL"),
            Gyroscope = str_to_four_cc("GYRO"),
            GpsFix = str_to_four_cc("GPSF"),
            GpsAccuracy = str_to_four_cc("GPSP"),
            GpsTimestamp = str_to_four_cc("GPSU"),
            Temperature = str_to_four_cc("TMPC"),
            Invalid = 0xFFFFFFFF
        };

        using Label = uint32_t;
        Label label;

        friend std::istream &operator>>(std::istream &, Klv &);
        friend std::ostream &operator<<(std::ostream &, const Klv &);
    };

    namespace Internal
    {
        template<typename CLASS, int ELEMENT_SIZE, typename... Args>
        struct ReadableFromStream
        {
            static CLASS create_one(const DynBuffer &buffer, const Klv &klv, Args&& ... args)
            {
                if (klv.size != ELEMENT_SIZE && ELEMENT_SIZE)
                    throw std::runtime_error("Invalid size");
                if (buffer.size() != klv.size)
                    throw std::runtime_error("Invalid size");
                if (klv.len != 1)
                    throw std::runtime_error("Expected one element");
                return CLASS(buffer.data(), std::forward<Args>(args)...);
            }

            static std::vector<CLASS> create_array(const DynBuffer &buffer, const Klv &klv, Args&&... args)
            {
                if (klv.size != ELEMENT_SIZE && ELEMENT_SIZE)
                    throw std::runtime_error("Invalid size");
                if (buffer.size() != klv.memory_usage)
                    throw std::runtime_error("Invalid size");

                std::vector<CLASS> ret;
                for (int i = 0; i < klv.len; i++)
                    ret.emplace_back(buffer.data() + i * klv.size,
                                    std::forward<Args>(args)...);
                return ret;
            }
        };
    }

    template <typename CLASS, int ELEMENT_SIZE, typename... Args>
    using ReadableFromStream = Internal::ReadableFromStream<CLASS, ELEMENT_SIZE, Args...>;

    struct Divisor : public ReadableFromStream<Divisor, 0, int>
    {
        Divisor(const char *, int);
        double divisor;
    };

    using ScalFactor = std::vector<Divisor>;

    template <typename CLASS, int ELEMENT_SIZE, int SF_LENGTH>
    struct ReadableFromStreamWithScaling
        : Internal::ReadableFromStream<CLASS, ELEMENT_SIZE, const ScalFactor &>
    {
        using Parent = Internal::ReadableFromStream<CLASS, ELEMENT_SIZE, const ScalFactor &>;
        static CLASS create_one(const DynBuffer &buffer, const Klv &klv, const ScalFactor &sf)
        {
            if (sf.size() != SF_LENGTH)
                throw std::runtime_error("Invalid number of scalor " + std::to_string(sf.size()) + "," + std::to_string(SF_LENGTH));
            return Parent::create_one(buffer, klv, sf);
        }

        static std::vector<CLASS> create_array(const DynBuffer &buffer, const Klv &klv, const ScalFactor &sf)
        {
            if (sf.size() != SF_LENGTH)
                throw std::runtime_error("Invalid number of scalor " + std::to_string(sf.size()) + "," + std::to_string(SF_LENGTH));
            return Parent::create_array(buffer, klv, sf);
        }
    };
    std::string label_to_string(Klv::Label label);

    // ACCL - 3-axis accelerometer measurements (meters/sec^2)
    struct Accelerometer : public ReadableFromStreamWithScaling<Accelerometer, 6, 1>
    {
        Accelerometer() = default;
        Accelerometer(const char *, const ScalFactor &);
        double x{0.}, y{0.}, z{0.};
        friend std::ostream &operator<<(std::ostream &, const Accelerometer &);
    };

    // GYRO - 3-axis gyroscope measurement (radians/sec)
    struct Gyroscope : public ReadableFromStreamWithScaling<Gyroscope, 6, 1>
    {
        Gyroscope() = default;
        Gyroscope(const char *, const ScalFactor &);
        double x{0.}, y{0.}, z{0.};
        friend std::ostream &operator<<(std::ostream &, const Gyroscope &);
    };

    struct GpsPos : public ReadableFromStreamWithScaling<GpsPos, 20, 5>
    {
        GpsPos() = default;
        GpsPos(const char *, const ScalFactor &);
        double lat{0.};  ///< Latitude (Degrees)
        double lont{0.}; ///< Longitude (Degrees)
        double alt{0.};  ///< Altitude
        double speed_2d{0.}; ///< m/s
        double speed_3d{0.}; ///< m/s
        friend std::ostream &operator<<(std::ostream &, const GpsPos &);
    };

    // GPSF - GPS fix (0: No Fix, 2: 2D, 3: 3D)
    struct GpsFix : public ReadableFromStream<GpsFix, 4>
    {
        GpsFix() = default;
        GpsFix(const char *);
        enum Value {No_Fix, Fix_2d = 2, fix_3d = 3, LAST};
        Value gps_fix{No_Fix};
        friend std::ostream &operator<<(std::ostream &, const GpsFix &);
    };

    // GPSP - GPS position accuracy (centimeters)
    struct GpsAccuracy : public ReadableFromStream<GpsAccuracy, 2>
    {
        GpsAccuracy() = default;
        GpsAccuracy(const char *);
        uint16_t accuracy{0}; ///< Accuracy (centimeters)
        friend std::ostream &operator<<(std::ostream &, const GpsAccuracy &);
    };

    // GPSU - GPS acquired UTC timestamp
    struct GpsTimestamp : public ReadableFromStream<GpsTimestamp, 16>
    {
        GpsTimestamp() = default;
        GpsTimestamp(const char *);

        std::string time;
        friend std::ostream &operator<<(std::ostream &, const GpsTimestamp &);
    };

    // TMPC - Temperature (Degrees C)
    struct Temperature : public ReadableFromStream<Temperature, 4>
    {
        Temperature() = default;
        Temperature(const char *);
        float temp{0.}; ///< Degrees C
        friend std::ostream &operator<<(std::ostream &, const Temperature &);
    };

    struct Telemetry
    {
        std::vector<Accelerometer> accelerometer;
        std::vector<GpsPos> gps_position;
        std::vector<Gyroscope> gyroscope;
        GpsFix gps_fix;
        GpsAccuracy gps_accuracy;
        GpsTimestamp gps_timestamp;
        Temperature temperature;

        friend std::ostream &operator<<(std::ostream &, const Telemetry &);
    };
}
