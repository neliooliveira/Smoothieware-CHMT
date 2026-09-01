#pragma once

#include <stdint.h>

// Fly-by vision host protocol.
// Command numbers are intentionally centralized here so they can be remapped
// without touching the motion/ISR implementation.
namespace FlyByProtocol {

static const uint16_t M_MODE   = 950; // M950 S0=LIVE, S1=TRIGGER, S2=AUTO
static const uint16_t M_ARM    = 951; // Arm next motion: I<id> N<nozzle> D<mm> [C0/1] [L0/1]
static const uint16_t M_STATUS = 952; // Report current mode and pending trigger
static const uint16_t M_TIMING = 953; // Pulse timing: C<camera_us> L<strobe_us>

enum Mode : uint8_t {
    LIVE = 0,
    TRIGGER = 1,
    AUTO = 2
};

}
