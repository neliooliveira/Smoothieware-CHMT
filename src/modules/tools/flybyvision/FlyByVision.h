#pragma once

#include "libs/Module.h"
#include "Pin.h"
#include "FlyByTrigger.h"
#include "FlyByProtocol.h"

class Gcode;

class FlyByVision : public Module {
public:
    FlyByVision();
    void on_module_loaded() override;
    void on_gcode_received(void *argument) override;
    void on_idle(void *argument) override;

    static FlyByVision *getInstance() { return instance; }
    static void trigger_from_isr(const FlyByTrigger& trigger);
    static void service_pulses_from_isr();

    bool consume_pending(FlyByTrigger& trigger, float block_mm);
    static void finalize_trigger_tick(FlyByTrigger& trigger, float block_mm,
                                      float entry_speed, float exit_speed,
                                      float maximum_rate_steps_s, uint32_t steps_event_count,
                                      uint32_t accelerate_until, uint32_t decelerate_after,
                                      uint32_t total_move_ticks, float tick_frequency);

private:
    static FlyByVision *instance;

    void load_config();
    void set_mode(Gcode *gcode);
    void arm(Gcode *gcode);
    void set_timing(Gcode *gcode);
    void cancel();
    void status(Gcode *gcode);

    Pin camera_pin;
    Pin strobe_pin;
    bool enabled{false};
    FlyByProtocol::Mode mode{FlyByProtocol::AUTO};
    uint32_t camera_width_ticks{4};
    uint32_t strobe_width_ticks{8};

    FlyByTrigger pending;
    float pending_distance_mm{0.0F};
    float pending_fraction{-1.0F};

    volatile uint32_t camera_off_tick{0};
    volatile uint32_t strobe_off_tick{0};
    volatile uint16_t fired_id{0};
    volatile uint8_t fired_nozzle{0};
    volatile bool fired_report_pending{false};
};
