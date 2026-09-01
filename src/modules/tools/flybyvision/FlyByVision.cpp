#include "FlyByVision.h"

#include "libs/Kernel.h"
#include "Config.h"
#include "ConfigValue.h"
#include "checksumm.h"
#include "Gcode.h"
#include "StepTicker.h"
#include "StreamOutput.h"

#include <algorithm>

#define flyby_enable_checksum               CHECKSUM("flyby_vision_enable")
#define flyby_camera_pin_checksum           CHECKSUM("flyby_camera_trigger_pin")
#define flyby_strobe_pin_checksum           CHECKSUM("flyby_strobe_pin")
#define flyby_camera_width_checksum         CHECKSUM("flyby_trigger_width_us")
#define flyby_strobe_width_checksum         CHECKSUM("flyby_strobe_width_us")

FlyByVision *FlyByVision::instance = nullptr;

FlyByVision::FlyByVision()
{
    instance = this;
}

void FlyByVision::on_module_loaded()
{
    load_config();
    register_for_event(ON_GCODE_RECEIVED);
    register_for_event(ON_IDLE);
    THEKERNEL->step_ticker->set_flyby_hook(&FlyByVision::trigger_from_isr);
}

void FlyByVision::load_config()
{
    enabled = THEKERNEL->config->value(flyby_enable_checksum)->by_default(false)->as_bool();
    camera_pin.from_string(THEKERNEL->config->value(flyby_camera_pin_checksum)->by_default("nc")->as_string())->as_output();
    strobe_pin.from_string(THEKERNEL->config->value(flyby_strobe_pin_checksum)->by_default("nc")->as_string())->as_output();
    camera_pin.set(false);
    strobe_pin.set(false);

    const float tick_us = 5.0F; // StepTicker is 200 kHz in this branch.
    float camera_us = THEKERNEL->config->value(flyby_camera_width_checksum)->by_default(20.0F)->as_number();
    float strobe_us = THEKERNEL->config->value(flyby_strobe_width_checksum)->by_default(40.0F)->as_number();
    camera_width_ticks = std::max<uint32_t>(1, (uint32_t)(camera_us / tick_us + 0.5F));
    strobe_width_ticks = std::max<uint32_t>(1, (uint32_t)(strobe_us / tick_us + 0.5F));
}

void FlyByVision::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);
    if(!gcode->has_m) return;

    switch(gcode->m) {
        case 950: arm(gcode); break;
        case 951: cancel(); break;
        case 952: status(gcode); break;
        case 953:
            enabled = gcode->has_letter('S') ? (gcode->get_int('S') != 0) : enabled;
            gcode->stream->printf("flyby enabled=%d\n", enabled ? 1 : 0);
            break;
        default: break;
    }
}

void FlyByVision::arm(Gcode *gcode)
{
    if(!enabled) {
        gcode->stream->printf("error: flyby vision disabled\n");
        return;
    }

    pending.clear();
    pending.trigger_id = gcode->has_letter('I') ? (uint16_t)gcode->get_uint('I') : 0;
    pending.nozzle_id = gcode->has_letter('N') ? (uint8_t)gcode->get_uint('N') : 0;
    pending.flags = FlyByTrigger::ENABLED | FlyByTrigger::CAMERA_TRIGGER | FlyByTrigger::LED_STROBE;

    pending_distance_mm = gcode->has_letter('D') ? gcode->get_value('D') : 0.0F;
    pending_fraction = gcode->has_letter('P') ? gcode->get_value('P') : -1.0F;
    if(pending_fraction > 1.0F) pending_fraction *= 0.01F;
    if(pending_fraction >= 0.0F) pending_fraction = std::max(0.0F, std::min(1.0F, pending_fraction));

    gcode->stream->printf("flyby armed I%u N%u\n", pending.trigger_id, pending.nozzle_id);
}

void FlyByVision::cancel()
{
    pending.clear();
    pending_distance_mm = 0.0F;
    pending_fraction = -1.0F;
}

void FlyByVision::status(Gcode *gcode)
{
    gcode->stream->printf("flyby enabled=%d pending=%d I%u N%u\n", enabled ? 1 : 0,
        pending.enabled() ? 1 : 0, pending.trigger_id, pending.nozzle_id);
}

bool FlyByVision::consume_pending(FlyByTrigger& trigger, float block_mm)
{
    if(!enabled || !pending.enabled() || block_mm <= 0.0F) return false;

    trigger = pending;
    float fraction = pending_fraction;
    if(fraction < 0.0F) fraction = pending_distance_mm / block_mm;
    fraction = std::max(0.0F, std::min(1.0F, fraction));

    // Temporary spatial representation. Planner finalizes this into a real
    // trajectory tick after total_move_ticks is known.
    trigger.trigger_tick = (uint32_t)(fraction * 4294967295.0F);
    cancel();
    return true;
}

void FlyByVision::trigger_from_isr(const FlyByTrigger& trigger)
{
    FlyByVision *self = instance;
    if(self == nullptr || !self->enabled) return;

    if((trigger.flags & FlyByTrigger::CAMERA_TRIGGER) && self->camera_pin.connected()) self->camera_pin.set(true);
    if((trigger.flags & FlyByTrigger::LED_STROBE) && self->strobe_pin.connected()) self->strobe_pin.set(true);

    self->camera_off_tick = self->camera_width_ticks;
    self->strobe_off_tick = self->strobe_width_ticks;
    self->fired_id = trigger.trigger_id;
    self->fired_nozzle = trigger.nozzle_id;
    self->fired_report_pending = true;
}

void FlyByVision::on_idle(void *)
{
    if(fired_report_pending) {
        fired_report_pending = false;
        THEKERNEL->streams->printf("// FLYBY I%u N%u\n", fired_id, fired_nozzle);
    }
}
