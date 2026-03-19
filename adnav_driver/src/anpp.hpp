#pragma once
#include <cstdint>
#include <string>

enum class FilterStatusBit : uint16_t {
    OrientationFilterInitialised = (1u << 0),
    NavigationFilterInitialised  = (1u << 1),
    HeadingInitialised           = (1u << 2),
    UtcTimeInitialised           = (1u << 3),
    // Bits 4–6: GNSS fix status — NOT individual flags, use gnss_fix_status()
    Event1Occurred               = (1u << 7),
    Event2Occurred               = (1u << 8),
    InternalGnssEnabled          = (1u << 9),
    DualAntennaHeadingActive     = (1u << 10),
    VelocityHeadingEnabled       = (1u << 11),
    AtmosphericAltitudeEnabled   = (1u << 12),
    ExternalPositionActive       = (1u << 13),
    ExternalVelocityActive       = (1u << 14),
    ExternalHeadingActive        = (1u << 15),
};

enum class GnssFixStatus : uint8_t {
    NoFix          = 0,
    Fix2D          = 1,
    Fix3D          = 2,
    SbasFix        = 3,
    DifferentialFix = 4,
    PppFix         = 5,
    RtkFloat       = 6,
    RtkFixed       = 7,
};

inline std::string to_string(const FilterStatusBit bit) {
    switch (bit) {
        case FilterStatusBit::OrientationFilterInitialised: return "Orientation Filter Initialised";
        case FilterStatusBit::NavigationFilterInitialised: return "Navigation Filter Initialised";
        case FilterStatusBit::HeadingInitialised: return "Heading Initialised";
        case FilterStatusBit::UtcTimeInitialised: return "UTC Time Initialised";
        case FilterStatusBit::Event1Occurred: return "Event 1 Occurred";
        case FilterStatusBit::Event2Occurred: return "Event 2 Occurred";
        case FilterStatusBit::InternalGnssEnabled: return "Internal GNSS Enabled";
        case FilterStatusBit::DualAntennaHeadingActive: return "Dual Antenna Heading Active";
        case FilterStatusBit::VelocityHeadingEnabled: return "Velocity Heading Enabled";
        case FilterStatusBit::AtmosphericAltitudeEnabled: return "Atmospheric Altitude Enabled";
        case FilterStatusBit::ExternalPositionActive: return "External Position Active";
        case FilterStatusBit::ExternalVelocityActive: return "External Velocity Active";
        case FilterStatusBit::ExternalHeadingActive: return "External Heading Active";
    }
    return "Unknown Bit";
}

inline std::string to_string(const GnssFixStatus status) {
    switch (status) {
        case GnssFixStatus::NoFix: return "No Fix";
        case GnssFixStatus::Fix2D: return "2D Fix";
        case GnssFixStatus::Fix3D: return "3D Fix";
        case GnssFixStatus::SbasFix: return "SBAS Fix";
        case GnssFixStatus::DifferentialFix: return "Differential Fix";
        case GnssFixStatus::PppFix: return "PPP Fix";
        case GnssFixStatus::RtkFloat: return "RTK Float";
        case GnssFixStatus::RtkFixed: return "RTK Fixed";
    }
    return "Unknown Status";
}