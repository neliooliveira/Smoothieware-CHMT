#pragma once

#include "libs/Module.h"

class Gcode;

// Runtime Pick & Place motion policy.
// G61 selects exact-path mode for pick/place/fiducial/stationary vision.
// G64 [P<mm>] selects high-speed continuous mode with bounded junction deviation.
class PnPMotion : public Module {
public:
    void on_module_loaded() override;
    void on_gcode_received(void *argument) override;

private:
    void report(Gcode *gcode) const;
};
