#pragma once
// How the readouts show lengths, masses and the simulated date (V1.1 settings, Display & units). No OpenGL, no ImGui.
//
// One process-wide preference that formatDistance, formatSimDate (format.h) and formatQuantity with no forced unit (units.h)
// follow, so every readout and every editor field changes with one switch. The defaults (Auto, DayClock) are the automatic
// units and "Day 123  04:05:06" the readouts always had. Parsing is unaffected: every unit is still accepted in every field.
namespace cosmos {
namespace app {

enum class LengthUnit { Auto, Metres, Kilometres, AU };
enum class MassUnit { Auto, Kilograms, EarthMasses, SolarMasses };
enum class DateFormat {
    DayClock,   // "Day 123  04:05:06"
    DayOnly,    // "Day 123"
    YearDay,    // "Year 2, Day 14  04:05:06" (365-day years counted from the epoch, like the day count)
};

struct DisplayPrefs {
    LengthUnit length = LengthUnit::Auto;
    MassUnit mass = MassUnit::Auto;
    DateFormat date = DateFormat::DayClock;
};

// The preference in use. Set it from the UI thread; the readouts read it on the same thread.
DisplayPrefs& displayPrefs();

} // namespace app
} // namespace cosmos
