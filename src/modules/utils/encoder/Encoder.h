#pragma once

#include "Module.h"
#include <stdint.h>

class StreamOutput;

class Encoder : public Module {
    public:
        Encoder();

        void on_module_loaded();
        void on_gcode_received(void *argument);

        int32_t get_x_count();
        int32_t get_y_count();
        void set_x_count(int32_t count);
        void set_y_count(int32_t count);

    private:
        void init_encoders();
        void report_encoder_position(StreamOutput *stream);
        void report_stepper_position(StreamOutput *stream);
        void auto_calibrate(StreamOutput *stream, float distance);

        float x_counts_per_mm;
        float y_counts_per_mm;
        bool encoder_enabled;
};
