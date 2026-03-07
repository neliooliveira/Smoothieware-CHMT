#include "Encoder.h"
#include "libs/Kernel.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "Config.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "Gcode.h"
#include "libs/StreamOutput.h"
#include "Robot.h"
#include "StepperMotor.h"
#include "Conveyor.h"

#include "stm32f4xx_hal.h"

#define encoder_enable_checksum          CHECKSUM("encoder_enable")
#define encoder_x_counts_per_mm_checksum CHECKSUM("encoder_x_counts_per_mm")
#define encoder_y_counts_per_mm_checksum CHECKSUM("encoder_y_counts_per_mm")
#define alpha_max_travel_checksum        CHECKSUM("alpha_max_travel")
#define beta_max_travel_checksum         CHECKSUM("beta_max_travel")

static TIM_HandleTypeDef htim2;
static TIM_HandleTypeDef htim5;

Encoder::Encoder()
{
    encoder_enabled = false;
    x_counts_per_mm = 0;
    y_counts_per_mm = 0;
}

void Encoder::on_module_loaded()
{
    encoder_enabled = THEKERNEL->config->value(encoder_enable_checksum)->by_default(false)->as_bool();
    if (!encoder_enabled) {
        delete this;
        return;
    }

    x_counts_per_mm = THEKERNEL->config->value(encoder_x_counts_per_mm_checksum)->by_default(0)->as_number();
    y_counts_per_mm = THEKERNEL->config->value(encoder_y_counts_per_mm_checksum)->by_default(0)->as_number();

    init_encoders();

    this->register_for_event(ON_GCODE_RECEIVED);
}

void Encoder::init_encoders()
{
    // Configure GPIO pins for TIM2 encoder (X axis): PA15 = CH1, PB3 = CH2
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_HIGH;

    gpio.Pin = GPIO_PIN_15;
    gpio.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = GPIO_PIN_3;
    gpio.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOB, &gpio);

    // Configure GPIO pins for TIM5 encoder (Y axis): PA0 = CH1, PA1 = CH2
    gpio.Pin = GPIO_PIN_0;
    gpio.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = GPIO_PIN_1;
    gpio.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(GPIOA, &gpio);

    // Configure TIM2 in encoder mode (X axis)
    __HAL_RCC_TIM2_CLK_ENABLE();

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 0xFFFFFFFF; // 32-bit timer, full range
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

    TIM_Encoder_InitTypeDef encoder_config = {0};
    encoder_config.EncoderMode = TIM_ENCODERMODE_TI12; // count on both edges
    encoder_config.IC1Polarity = TIM_ICPOLARITY_RISING;
    encoder_config.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    encoder_config.IC1Prescaler = TIM_ICPSC_DIV1;
    encoder_config.IC1Filter = 0x0F; // max filtering for noise rejection
    encoder_config.IC2Polarity = TIM_ICPOLARITY_RISING;
    encoder_config.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    encoder_config.IC2Prescaler = TIM_ICPSC_DIV1;
    encoder_config.IC2Filter = 0x0F;

    HAL_TIM_Encoder_Init(&htim2, &encoder_config);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

    // Configure TIM5 in encoder mode (Y axis)
    __HAL_RCC_TIM5_CLK_ENABLE();

    htim5.Instance = TIM5;
    htim5.Init.Prescaler = 0;
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.Period = 0xFFFFFFFF; // 32-bit timer, full range
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

    HAL_TIM_Encoder_Init(&htim5, &encoder_config);
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
}

int32_t Encoder::get_x_count()
{
    return (int32_t)TIM2->CNT;
}

int32_t Encoder::get_y_count()
{
    return (int32_t)TIM5->CNT;
}

void Encoder::set_x_count(int32_t count)
{
    TIM2->CNT = (uint32_t)count;
}

void Encoder::set_y_count(int32_t count)
{
    TIM5->CNT = (uint32_t)count;
}

void Encoder::report_encoder_position(StreamOutput *stream)
{
    stream->printf("ok EX:%ld EY:%ld\n", get_x_count(), get_y_count());
}

void Encoder::report_stepper_position(StreamOutput *stream)
{
    int32_t sx = THEKERNEL->robot->actuators[0]->get_current_step();
    int32_t sy = THEKERNEL->robot->actuators[1]->get_current_step();
    stream->printf("ok SX:%ld SY:%ld\n", sx, sy);
}

void Encoder::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);

    if (gcode->has_m) {
        switch (gcode->m) {
            case 918: // report encoder positions
                report_encoder_position(gcode->stream);
                break;

            case 919: // set encoder counters
                if (gcode->has_letter('X')) set_x_count((int32_t)gcode->get_value('X'));
                if (gcode->has_letter('Y')) set_y_count((int32_t)gcode->get_value('Y'));
                gcode->stream->printf("ok\n");
                break;

            case 921: // report stepper step counts
                report_stepper_position(gcode->stream);
                break;

            case 922: // set stepper step counters (only when idle)
                if (!THEKERNEL->conveyor->is_idle()) {
                    gcode->stream->printf("error: machine is moving\n");
                } else {
                    if (gcode->has_letter('X'))
                        THEKERNEL->robot->actuators[0]->current_position_steps = (int32_t)gcode->get_value('X');
                    if (gcode->has_letter('Y'))
                        THEKERNEL->robot->actuators[1]->current_position_steps = (int32_t)gcode->get_value('Y');
                    gcode->stream->printf("ok\n");
                }
                break;

            case 923: // set encoder counts per mm
                if (gcode->has_letter('X')) x_counts_per_mm = gcode->get_value('X');
                if (gcode->has_letter('Y')) y_counts_per_mm = gcode->get_value('Y');
                gcode->stream->printf("ok X:%.4f Y:%.4f\n", x_counts_per_mm, y_counts_per_mm);
                break;

            case 924: { // auto-calibrate encoder counts per mm
                if (!THEKERNEL->conveyor->is_idle()) {
                    gcode->stream->printf("error: machine is moving\n");
                    break;
                }
                float cal_distance = 0;
                if (gcode->has_letter('D')) {
                    cal_distance = gcode->get_value('D');
                } else {
                    float x_travel = THEKERNEL->config->value(alpha_max_travel_checksum)->by_default(500)->as_number();
                    float y_travel = THEKERNEL->config->value(beta_max_travel_checksum)->by_default(500)->as_number();
                    cal_distance = (x_travel < y_travel ? x_travel : y_travel) / 2.0f;
                }
                auto_calibrate(gcode->stream, cal_distance);
                break;
            }
        }
    }
}

void Encoder::auto_calibrate(StreamOutput *stream, float distance)
{
    // Zero encoder counts at current (home) position
    set_x_count(0);
    set_y_count(0);

    float cal_speed = 10.0f; // mm/s — slow for accuracy, minimizes lost steps

    THEROBOT->push_state();

    // Move toward origin (negative direction from home at back-right)
    float delta[3] = {-distance, -distance, 0};
    THEROBOT->delta_move(delta, cal_speed, 3);
    THECONVEYOR->wait_for_idle();

    // Capture encoder counts after calibration move
    int32_t ex = get_x_count();
    int32_t ey = get_y_count();

    // Move back to starting position
    float delta_back[3] = {distance, distance, 0};
    THEROBOT->delta_move(delta_back, cal_speed, 3);
    THECONVEYOR->wait_for_idle();

    THEROBOT->pop_state();

    // Calculate counts per mm from absolute encoder counts
    int32_t abs_ex = ex < 0 ? -ex : ex;
    int32_t abs_ey = ey < 0 ? -ey : ey;
    x_counts_per_mm = (float)abs_ex / distance;
    y_counts_per_mm = (float)abs_ey / distance;

    // Report raw counts (sign indicates encoder direction) and calibrated values
    stream->printf("ok EX:%ld EY:%ld X:%.4f Y:%.4f\n", ex, ey, x_counts_per_mm, y_counts_per_mm);
}
