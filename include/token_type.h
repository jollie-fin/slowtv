#pragma once

#include <cstdint>
#include <array>
#include <algorithm>

//update the array
enum class TokenType
{
    AALP,
    ACCL,
    ALLD,
    CORI,
    DEVC,
    DISP,
    DVID,
    DVNM,
    EMPT,
    FACE,
    FCNM,
    GPS5,
    GPSF,
    GPSP,
    GPSU,
    GRAV,
    GYRO,
    HUES,
    IORI,
    ISOE,
    ISOG,
    MAGN,
    MWET,
    ORIN,
    ORIO,
    RMRK,
    SCAL,
    SCEN,
    SHUT,
    SIUN,
    SROT,
    STMP,
    STNM,
    STPS,
    STRM,
    SUIN,
    TICK,
    TIMO,
    TMPC,
    TOCK,
    TSMP,
    TYPE,
    UNIF,
    UNIT,
    UUID,
    WBAL,
    WNDM,
    WRGB,
    YAVG,
    SIZE,
    INVALID
};

using Fourcc = uint32_t;
constexpr Fourcc make_fourcc(const auto &name)
{
    Fourcc fourcc = 0;
    for (int i = 0; i < 4; i++)
    {
        fourcc <<= 8;
        fourcc |= static_cast<uint8_t>(name[i]);
    }
    return fourcc;
}

namespace Internal
{
    //Please keep it sorted
    static constexpr std::array<std::pair<Fourcc, TokenType>,
                                static_cast<std::size_t>(TokenType::SIZE)> fourcc_token_equivalence =
    {
        std::make_pair(make_fourcc("AALP"),TokenType::AALP),
        std::make_pair(make_fourcc("ACCL"),TokenType::ACCL),
        std::make_pair(make_fourcc("ALLD"),TokenType::ALLD),
        std::make_pair(make_fourcc("CORI"),TokenType::CORI),
        std::make_pair(make_fourcc("DEVC"),TokenType::DEVC),
        std::make_pair(make_fourcc("DISP"),TokenType::DISP),
        std::make_pair(make_fourcc("DVID"),TokenType::DVID),
        std::make_pair(make_fourcc("DVNM"),TokenType::DVNM),
        std::make_pair(make_fourcc("EMPT"),TokenType::EMPT),
        std::make_pair(make_fourcc("FACE"),TokenType::FACE),
        std::make_pair(make_fourcc("FCNM"),TokenType::FCNM),
        std::make_pair(make_fourcc("GPS5"),TokenType::GPS5),
        std::make_pair(make_fourcc("GPSF"),TokenType::GPSF),
        std::make_pair(make_fourcc("GPSP"),TokenType::GPSP),
        std::make_pair(make_fourcc("GPSU"),TokenType::GPSU),
        std::make_pair(make_fourcc("GRAV"),TokenType::GRAV),
        std::make_pair(make_fourcc("GYRO"),TokenType::GYRO),
        std::make_pair(make_fourcc("HUES"),TokenType::HUES),
        std::make_pair(make_fourcc("IORI"),TokenType::IORI),
        std::make_pair(make_fourcc("ISOE"),TokenType::ISOE),
        std::make_pair(make_fourcc("ISOG"),TokenType::ISOG),
        std::make_pair(make_fourcc("MAGN"),TokenType::MAGN),
        std::make_pair(make_fourcc("MTRX"),TokenType::MAGN),
        std::make_pair(make_fourcc("MWET"),TokenType::MWET),
        std::make_pair(make_fourcc("ORIN"),TokenType::ORIN),
        std::make_pair(make_fourcc("ORIO"),TokenType::ORIO),
        std::make_pair(make_fourcc("RMRK"),TokenType::RMRK),
        std::make_pair(make_fourcc("SCAL"),TokenType::SCAL),
        std::make_pair(make_fourcc("SCEN"),TokenType::SCEN),
        std::make_pair(make_fourcc("SHUT"),TokenType::SHUT),
        std::make_pair(make_fourcc("SIUN"),TokenType::SIUN),
        std::make_pair(make_fourcc("SROT"),TokenType::SROT),
        std::make_pair(make_fourcc("STMP"),TokenType::STMP),
        std::make_pair(make_fourcc("STNM"),TokenType::STNM),
        std::make_pair(make_fourcc("STPS"),TokenType::STPS),
        std::make_pair(make_fourcc("STRM"),TokenType::STRM),
        std::make_pair(make_fourcc("SUIN"),TokenType::SUIN),
        std::make_pair(make_fourcc("TICK"),TokenType::TICK),
        std::make_pair(make_fourcc("TIMO"),TokenType::TIMO),
        std::make_pair(make_fourcc("TMPC"),TokenType::TMPC),
        std::make_pair(make_fourcc("TOCK"),TokenType::TICK),
        std::make_pair(make_fourcc("TSMP"),TokenType::TSMP),
        std::make_pair(make_fourcc("TYPE"),TokenType::TYPE),
        std::make_pair(make_fourcc("UNIF"),TokenType::UNIF),
        std::make_pair(make_fourcc("UNIT"),TokenType::UNIT),
        std::make_pair(make_fourcc("WBAL"),TokenType::WBAL),
        std::make_pair(make_fourcc("WNDM"),TokenType::WNDM),
        std::make_pair(make_fourcc("WRGB"),TokenType::WRGB),
        std::make_pair(make_fourcc("YAVG"),TokenType::YAVG)
    };

    constexpr TokenType fourcc_to_token(Fourcc fourcc)
    {
        auto it = std::lower_bound(
            fourcc_token_equivalence.begin(),
            fourcc_token_equivalence.end(),
            fourcc,
            [&](const auto &t1, const auto &t2)
            {
                return t1.first < t2;
            });
        if (it == fourcc_token_equivalence.end() ||
            it->first != fourcc)
            return TokenType::INVALID;
        return it->second;
    }
}

constexpr TokenType make_tokentype(const auto &name)
{
    return Internal::fourcc_to_token(make_fourcc(name));
}
