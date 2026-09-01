#pragma once

#include <stdint.h>

// Lightweight metadata for a deterministic fly-by camera trigger.
// This type deliberately contains only integer fields so it can be inspected
// from the step ISR without floating-point work or dynamic allocation.
struct FlyByTrigger {
    enum Flags : uint8_t {
        ENABLED        = 1 << 0,
        CAMERA_TRIGGER = 1 << 1,
        LED_STROBE     = 1 << 2
    };

    uint32_t trigger_tick;
    uint32_t trigger_distance_um;
    uint16_t trigger_id;
    uint8_t nozzle_id;
    uint8_t flags;

    FlyByTrigger()
        : trigger_tick(0), trigger_distance_um(0), trigger_id(0), nozzle_id(0), flags(0)
    {
    }

    void clear()
    {
        trigger_tick = 0;
        trigger_distance_um = 0;
        trigger_id = 0;
        nozzle_id = 0;
        flags = 0;
    }

    bool enabled() const
    {
        return (flags & ENABLED) != 0;
    }
};
