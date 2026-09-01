#include "PnPMotion.h"

#include "libs/Kernel.h"
#include "Gcode.h"
#include "StreamOutput.h"
#include "modules/robot/Planner.h"

void PnPMotion::on_module_loaded()
{
    register_for_event(ON_GCODE_RECEIVED);
}

void PnPMotion::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);
    if(!gcode->has_g || THEKERNEL->planner == nullptr) return;

    switch(gcode->g) {
        case 61: // exact path for pick/place/fiducial/stationary vision
            THEKERNEL->planner->set_exact_path(true);
            report(gcode);
            break;

        case 64: // continuous path; P is maximum accepted corner deviation in mm
            if(gcode->has_letter('P')) {
                THEKERNEL->planner->set_path_tolerance(gcode->get_value('P'));
            } else if(THEKERNEL->planner->get_path_tolerance() > 0.0F) {
                THEKERNEL->planner->set_exact_path(false);
            }
            report(gcode);
            break;

        default:
            break;
    }
}

void PnPMotion::report(Gcode *gcode) const
{
    gcode->stream->printf("pnp path=%s tolerance=%.4f scurve=%d jerk=%.1f\n",
        THEKERNEL->planner->is_path_blending_enabled() ? "blend" : "exact",
        THEKERNEL->planner->get_path_tolerance(),
        THEKERNEL->planner->is_s_curve_enabled() ? 1 : 0,
        THEKERNEL->planner->get_max_jerk());
}
