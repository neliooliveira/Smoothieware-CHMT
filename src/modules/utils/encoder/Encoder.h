#pragma once

#include "Module.h"
#include <stdint.h>

class StreamOutput;

class Encoder : public Module {
    public:
        Encoder();

        void on_module_loaded();
        void on_gcode_received(void *argument);
        void on_idle(void *argument);
        void on_halt(void *argument);

        int32_t get_x_count();
        int32_t get_y_count();
        void set_x_count(int32_t count);
        void set_y_count(int32_t count);

    private:
        void init_encoders();
        void init_output_compare();
        void report_encoder_position(StreamOutput *stream);
        void report_stepper_position(StreamOutput *stream);
        void auto_calibrate(StreamOutput *stream, float distance);
        void arm_x_target(int32_t target);
        void arm_y_target(int32_t target);
        void disarm_x();
        void disarm_y();

        float x_counts_per_mm;
        float y_counts_per_mm;
        float x_encoder_offset;
        float y_encoder_offset;
        float x_move_distance_mm;
        float y_move_distance_mm;
        uint32_t x_arm_time_us;
        uint32_t y_arm_time_us;
        volatile bool x_move_armed;
        volatile bool y_move_armed;
        bool encoder_enabled;
};
