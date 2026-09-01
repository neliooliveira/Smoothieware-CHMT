#pragma once

#include "libs/Module.h"
#include "Pin.h"
#include "FlyByTrigger.h"

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

private:
    static FlyByVision *instance;

    void load_config();
    void arm(Gcode *gcode);
    void cancel();
    void status(Gcode *gcode);

    Pin camera_pin;
    Pin strobe_pin;
    bool enabled{false};
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
