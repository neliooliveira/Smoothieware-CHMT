#include "FlyByVision.h"

#include "libs/Kernel.h"
#include "Config.h"
#include "ConfigValue.h"
#include "checksumm.h"
#include "Gcode.h"
#include "StepTicker.h"
#include "StreamOutput.h"
#include "StreamOutputPool.h"

#include <algorithm>
#include <math.h>

#define flyby_enable_checksum               CHECKSUM("flyby_vision_enable")
#define flyby_camera_pin_checksum           CHECKSUM("flyby_camera_trigger_pin")
#define flyby_strobe_pin_checksum           CHECKSUM("flyby_strobe_pin")
#define flyby_camera_width_checksum         CHECKSUM("flyby_trigger_width_us")
#define flyby_strobe_width_checksum         CHECKSUM("flyby_strobe_width_us")

FlyByVision *FlyByVision::instance = nullptr;

FlyByVision::FlyByVision() { instance = this; }

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
    const float tick_us = 1000000.0F / THEKERNEL->step_ticker->get_frequency();
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
        case FlyByProtocol::M_MODE: set_mode(gcode); break;
        case FlyByProtocol::M_ARM: arm(gcode); break;
        case FlyByProtocol::M_STATUS: status(gcode); break;
        case FlyByProtocol::M_TIMING: set_timing(gcode); break;
        default: break;
    }
}

void FlyByVision::set_mode(Gcode *gcode)
{
    if(gcode->has_letter('S')) {
        int value = gcode->get_int('S');
        if(value >= FlyByProtocol::LIVE && value <= FlyByProtocol::AUTO) {
            mode = static_cast<FlyByProtocol::Mode>(value);
            if(mode == FlyByProtocol::LIVE) cancel();
        }
    }
    gcode->stream->printf("flyby mode=%u enabled=%d\n", (unsigned)mode, enabled ? 1 : 0);
}

void FlyByVision::arm(Gcode *gcode)
{
    if(!enabled) { gcode->stream->printf("error: flyby vision disabled\n"); return; }
    if(mode == FlyByProtocol::LIVE) { gcode->stream->printf("error: flyby mode is LIVE\n"); return; }
    pending.clear();
    pending.trigger_id = gcode->has_letter('I') ? (uint16_t)gcode->get_uint('I') : 0;
    pending.nozzle_id = gcode->has_letter('N') ? (uint8_t)gcode->get_uint('N') : 0;
    pending.flags = FlyByTrigger::ENABLED;
    if(!gcode->has_letter('C') || gcode->get_int('C') != 0) pending.flags |= FlyByTrigger::CAMERA_TRIGGER;
    if(!gcode->has_letter('L') || gcode->get_int('L') != 0) pending.flags |= FlyByTrigger::LED_STROBE;
    pending_distance_mm = gcode->has_letter('D') ? gcode->get_value('D') : 0.0F;
    pending_fraction = gcode->has_letter('P') ? gcode->get_value('P') : -1.0F;
    if(pending_fraction > 1.0F) pending_fraction *= 0.01F;
    if(pending_fraction >= 0.0F) pending_fraction = std::max(0.0F, std::min(1.0F, pending_fraction));
    gcode->stream->printf("flyby armed I%u N%u\n", pending.trigger_id, pending.nozzle_id);
}

void FlyByVision::set_timing(Gcode *gcode)
{
    const float tick_us = 1000000.0F / THEKERNEL->step_ticker->get_frequency();
    if(gcode->has_letter('C')) camera_width_ticks = std::max<uint32_t>(1, (uint32_t)(gcode->get_value('C') / tick_us + 0.5F));
    if(gcode->has_letter('L')) strobe_width_ticks = std::max<uint32_t>(1, (uint32_t)(gcode->get_value('L') / tick_us + 0.5F));
    gcode->stream->printf("flyby timing C%lu L%lu ticks\n", camera_width_ticks, strobe_width_ticks);
}

void FlyByVision::cancel()
{
    pending.clear();
    pending_distance_mm = 0.0F;
    pending_fraction = -1.0F;
}

void FlyByVision::status(Gcode *gcode)
{
    gcode->stream->printf("flyby enabled=%d mode=%u pending=%d I%u N%u\n", enabled ? 1 : 0,
        (unsigned)mode, pending.enabled() ? 1 : 0, pending.trigger_id, pending.nozzle_id);
}

bool FlyByVision::consume_pending(FlyByTrigger& trigger, float block_mm)
{
    if(!enabled || mode == FlyByProtocol::LIVE || !pending.enabled() || block_mm <= 0.0F) return false;
    trigger = pending;
    float distance_mm = pending_fraction >= 0.0F ? pending_fraction * block_mm : pending_distance_mm;
    distance_mm = std::max(0.0F, std::min(block_mm, distance_mm));
    trigger.trigger_distance_um = (uint32_t)(distance_mm * 1000.0F + 0.5F);
    trigger.trigger_tick = 0;
    cancel();
    return true;
}

void FlyByVision::finalize_trigger_tick(FlyByTrigger& trigger, float block_mm,
                                        float entry_speed, float exit_speed,
                                        float maximum_rate_steps_s, uint32_t steps_event_count,
                                        uint32_t accelerate_until, uint32_t decelerate_after,
                                        uint32_t total_move_ticks, float tick_frequency)
{
    if(!trigger.enabled() || block_mm <= 0.0F || steps_event_count == 0 || total_move_ticks == 0 || tick_frequency <= 0.0F) return;

    double distance_mm = (double)trigger.trigger_distance_um / 1000.0;
    if(distance_mm < 0.0) distance_mm = 0.0;
    if(distance_mm > block_mm) distance_mm = block_mm;
    const double target_steps = distance_mm * (double)steps_event_count / (double)block_mm;

    const double t_acc = (double)accelerate_until / tick_frequency;
    const double t_dec_start = (double)decelerate_after / tick_frequency;
    const double t_total = (double)total_move_ticks / tick_frequency;
    const double t_dec = t_total - t_dec_start;
    const double v0 = (double)entry_speed * (double)steps_event_count / (double)block_mm;
    const double vf = (double)exit_speed * (double)steps_event_count / (double)block_mm;
    const double vmax = (double)maximum_rate_steps_s;
    const double a_acc = t_acc > 0.0 ? (vmax - v0) / t_acc : 0.0;
    const double a_dec = t_dec > 0.0 ? (vmax - vf) / t_dec : 0.0;
    const double s_acc = t_acc > 0.0 ? (v0 + vmax) * 0.5 * t_acc : 0.0;
    const double t_plateau = std::max(0.0, t_dec_start - t_acc);
    const double s_plateau = vmax * t_plateau;

    double t = 0.0;
    if(target_steps <= s_acc && a_acc > 0.0) {
        t = (-v0 + sqrt(v0 * v0 + 2.0 * a_acc * target_steps)) / a_acc;
    } else if(target_steps <= s_acc + s_plateau || a_dec <= 0.0) {
        t = t_acc + (vmax > 0.0 ? (target_steps - s_acc) / vmax : 0.0);
    } else {
        const double s2 = target_steps - s_acc - s_plateau;
        double radicand = vmax * vmax - 2.0 * a_dec * s2;
        if(radicand < 0.0) radicand = 0.0;
        const double td = (vmax - sqrt(radicand)) / a_dec;
        t = t_dec_start + td;
    }

    uint32_t tick = (uint32_t)floor(t * tick_frequency);
    if(tick >= total_move_ticks) tick = total_move_ticks - 1;
    trigger.trigger_tick = tick;
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

void FlyByVision::service_pulses_from_isr()
{
    FlyByVision *self = instance;
    if(self == nullptr) return;
    if(self->camera_off_tick != 0 && --self->camera_off_tick == 0 && self->camera_pin.connected()) self->camera_pin.set(false);
    if(self->strobe_off_tick != 0 && --self->strobe_off_tick == 0 && self->strobe_pin.connected()) self->strobe_pin.set(false);
}

void FlyByVision::on_idle(void *)
{
    if(fired_report_pending) {
        fired_report_pending = false;
        THEKERNEL->streams->printf("// FLYBY I%u N%u\n", fired_id, fired_nozzle);
    }
}
