#include <sstream>
#include <exception>
#include <optional>
#include <iostream>
#include "telemetry.h"
//#include "datatype.h"
#include "gpmf_reader.h"

template <typename CONTAINER1, typename CONTAINER2>
static void append(CONTAINER1 &a, CONTAINER2 &&b)
{
    a.insert(std::end(a), std::begin(b), std::end(b));
}
#if 0
using namespace OldGPParser;
static std::optional<Telemetry> read_packet(std::istream &is)
{
    Telemetry telemetry;
    Klv klv;
    ScalFactor sf;
    bool is_empty = true;

    while (is >> klv)
    {
        auto indent = (klv.label == str_to_four_cc("STRM")) ? "" : "  ";
        std::cerr << indent << label_to_string(klv.label) << "," << static_cast<int>(klv.size) << "," << klv.len << "," << klv.format;

        if (klv.format == 0)
        {
            std::cerr << std::endl;
            continue;
        }

        DynBuffer buffer(klv.memory_usage);
        is.read(buffer.data(), klv.memory_usage);

        if (!is)
            return {};
//            throw std::runtime_error("Unexpected EOF");

        if (klv.format == 'c')
            std::cerr << "," << std::string(buffer.begin(), buffer.end());
        std::cerr << std::endl;

        switch (klv.label)
        {
            case Klv::ScalFactor:
                sf = Divisor::create_array(buffer, klv, klv.size);
                is_empty = false;
                break;
            case Klv::Dvid:
                if (is_empty)
                    return {};
                return telemetry;
            case Klv::GpsPos:
                append(telemetry.gps_position, GpsPos::create_array(buffer, klv, sf));
                is_empty = false;
                break;
            case Klv::Accelerometer:
                append(telemetry.accelerometer, Accelerometer::create_array(buffer, klv, sf));
                is_empty = false;
                break;
            case Klv::Gyroscope:
                append(telemetry.gyroscope, Gyroscope::create_array(buffer, klv, sf));
                is_empty = false;
                break;
            case Klv::GpsFix:
                telemetry.gps_fix = GpsFix::create_one(buffer, klv);
                is_empty = false;
                break;
            case Klv::GpsAccuracy:
                telemetry.gps_accuracy = GpsAccuracy::create_one(buffer, klv);
                is_empty = false;
                break;
            case Klv::GpsTimestamp:
                telemetry.gps_timestamp = GpsTimestamp::create_one(buffer, klv);
                is_empty = false;
                break;
            case Klv::Temperature:
                telemetry.temperature = Temperature::create_one(buffer, klv);
                is_empty = false;
                break;
            default:
                break;
        }

        int padding = klv.memory_usage % 4;
		if (padding)
            is.ignore(4 - padding);
    }
    if (is_empty)
        return {};
    return telemetry;
}

#else

static std::optional<Telemetry> read_packet(std::istream &is)
{
    constexpr auto header_size = Token::header_size_;
    char header[header_size];
    MemoryArea header_span(header, header_size);
    if (!is.read(header, header_size))
        return {};

    auto payload_size = Token::payload_size(header_span);
    payload_size = ((payload_size + 3) / 4) * 4;
    std::vector<char> area(header_size + payload_size);
    std::copy(header_span.begin(), header_span.end(), area.begin());

    if (!is.read(area.data() + header_size, payload_size))
        return {};
    Token tok(MemoryArea(area.data(), area.size()));
    if (tok.type_ != make_tokentype("DEVC"))
        throw std::runtime_error("Inconsistent file");
    return Telemetry(tok.payload_);
}
#endif

void decode(std::ostream &ostream, std::istream &stream)
{
    while (stream)
    {
        auto telemetry = read_packet(stream);
        if (telemetry)
        {
        }
    }
}

