#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pid_controller.h"

#define SERIAL_MONITOR 1
#define SERIAL_PLOTTER 0

#define PIN_CURRENT_GPIO 32
#define PIN_PWM_GPIO 18

#define CURRENT_ADC_UNIT ADC_UNIT_1
#define CURRENT_ADC_CHANNEL ADC_CHANNEL_4
#define CURRENT_ADC_ATTEN ADC_ATTEN_DB_11

#define PWM_FREQ_HZ 20000
#define PWM_RES_BITS 8
#define PWM_DUTY_MAX ((1U << PWM_RES_BITS) - 1U)
#define PWM_MODE LEDC_LOW_SPEED_MODE
#define PWM_TIMER LEDC_TIMER_0
#define PWM_CHANNEL LEDC_CHANNEL_0

/*
 * Tinh toan cho motor 12V va mach do dong.
 *
 * Motor:
 *   - Motor dung nguon ngoai 12V qua mach driver/MOSFET.
 *   - Chan GPIO18 chi tao tin hieu PWM dieu khien driver, KHONG cap 12V truc tiep.
 *   - Dien ap trung binh uoc tinh tren motor:
 *       V_motor_avg = duty / 255 * MOTOR_SUPPLY_V
 *     Vi du duty=128 voi motor 12V:
 *       V_motor_avg = 128 / 255 * 12 = 6.02V
 *
 * ADC ESP32:
 *   - ADC_VREF = 3.3V la dien ap quy doi ADC cua ESP32, khong phai dien ap motor.
 *   - Khong dua 12V vao GPIO/ADC. Ngo ra ADC chi duoc nhan V_sense an toan < 3.3V.
 *
 * Cam bien dong low-side shunt + LM358 non-inverting:
 *   V_shunt = I_motor * R_SHUNT
 *   Gain    = 1 + VR1 / R1
 *   V_sense = V_shunt * Gain = I_motor * R_SHUNT * Gain
 *   I_motor = (V_adc - offset) / (R_SHUNT * Gain)
 *
 * Voi R_SHUNT=0.1 ohm va GAIN_ACTUAL=20:
 *   SENSOR_SENS_V = 0.1 * 20 = 2.0 V/A
 *   Neu I_motor=0.20A:
 *       V_sense = 0.20 * 2.0 = 0.40V
 *   Dong lon nhat de V_sense < 3.3V:
 *       I_adc_safe = (3.3 - offset) / 2.0 ~= 1.65A
 *
 * CURRENT_LIMIT_A la nguong bao ve theo dong motor 12V thuc te.
 * Hay chinh lai theo dong dinh muc/stall current cua motor va kha nang driver.
 */
#define MOTOR_SUPPLY_V 12.0f

#define ADC_VREF 3.3f
#define ADC_BITS 4096.0f

#define R_SHUNT 0.1f
#define GAIN_ACTUAL 20.0f
#define SENSOR_SENS_V (R_SHUNT * GAIN_ACTUAL)

#define CALIB_ON_BOOT 1
#define OFFSET_MANUAL 0.0f
#define CALIB_SAMPLES 200

#define CURRENT_LIMIT_A 0.30f
#define CURRENT_RESUME_A 0.10f
#define FAULT_COOLDOWN_MS 2000UL

#define FILTER_N 16
#define PID_SAMPLE_TIME_MS 5U
#define LOG_INTERVAL_MS 100U

static const char *TAG = "pwm_pid";

static adc_oneshot_unit_handle_t adc_handle;
static float sensor_offset_v = OFFSET_MANUAL;

static bool fault_active;
static uint32_t fault_time_ms;

static int filter_buf[FILTER_N];
static int filter_idx;
static int32_t filter_sum;

static pid_controller_t motor_pid;

static uint32_t millis_u32(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static double clamp_double(double value, double min_value, double max_value)
{
    if (value > max_value) {
        return max_value;
    }
    if (value < min_value) {
        return min_value;
    }
    return value;
}

static int adc_read_raw(void)
{
    int raw = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, CURRENT_ADC_CHANNEL, &raw));
    return raw;
}

static void pwm_write(uint32_t duty)
{
    if (duty > PWM_DUTY_MAX) {
        duty = PWM_DUTY_MAX;
    }

    ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, PWM_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, PWM_CHANNEL));
}

static void adc_init(void)
{
    const adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = CURRENT_ADC_UNIT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &adc_handle));

    const adc_oneshot_chan_cfg_t channel_cfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = CURRENT_ADC_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, CURRENT_ADC_CHANNEL, &channel_cfg));
}

static void pwm_init(void)
{
    const ledc_timer_config_t timer_cfg = {
        .speed_mode = PWM_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = PWM_TIMER,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    const ledc_channel_config_t channel_cfg = {
        .gpio_num = PIN_PWM_GPIO,
        .speed_mode = PWM_MODE,
        .channel = PWM_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = PWM_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_cfg));
}

static void filter_warm_up(void)
{
    filter_sum = 0;
    filter_idx = 0;

    for (int i = 0; i < FILTER_N; i++) {
        filter_buf[i] = adc_read_raw();
        filter_sum += filter_buf[i];
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

static int adc_read_filtered(void)
{
    filter_sum -= filter_buf[filter_idx];
    filter_buf[filter_idx] = adc_read_raw();
    filter_sum += filter_buf[filter_idx];
    filter_idx = (filter_idx + 1) % FILTER_N;

    return (int)(filter_sum / FILTER_N);
}

static float adc_to_current(int adc_raw)
{
    const float voltage = ((float)adc_raw / ADC_BITS) * ADC_VREF;
    float current = (voltage - sensor_offset_v) / SENSOR_SENS_V;

    if (current < 0.0f) {
        current = 0.0f;
    }
    return current;
}

static float duty_to_motor_voltage(uint32_t duty)
{
    if (duty > PWM_DUTY_MAX) {
        duty = PWM_DUTY_MAX;
    }

    return ((float)duty / (float)PWM_DUTY_MAX) * MOTOR_SUPPLY_V;
}

static void calibrate_offset(void)
{
#if CALIB_ON_BOOT
#if SERIAL_MONITOR
    ESP_LOGI(TAG, "[CALIB] Measuring offset at 0 A load...");
#endif
    vTaskDelay(pdMS_TO_TICKS(100));

    int32_t sum = 0;
    for (int i = 0; i < CALIB_SAMPLES; i++) {
        sum += adc_read_raw();
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    const int adc_zero = (int)(sum / CALIB_SAMPLES);
    const float v_zero = ((float)adc_zero / ADC_BITS) * ADC_VREF;

    if (v_zero > 0.2f) {
#if SERIAL_MONITOR
        ESP_LOGW(TAG, "[CALIB] Voffset = %.4f V > 0.2 V; using 0.0 V default", v_zero);
#endif
        sensor_offset_v = 0.0f;
    } else {
        sensor_offset_v = v_zero;
#if SERIAL_MONITOR
        ESP_LOGI(TAG, "[CALIB] ADC@0A = %d, Voffset = %.4f V", adc_zero, sensor_offset_v);
#endif
    }

#if SERIAL_MONITOR
    ESP_LOGI(TAG, "[CALIB] R_shunt=%.3f Ohm, Gain=%.1f, Sens=%.4f V/A",
             R_SHUNT, GAIN_ACTUAL, SENSOR_SENS_V);
    ESP_LOGI(TAG, "[CALIB] Motor supply=%.1f V, ADC max=%.1f V",
             MOTOR_SUPPLY_V, ADC_VREF);
    ESP_LOGI(TAG, "[CALIB] I_adc_safe (V_sense<%.1fV) = %.2f A",
             ADC_VREF, (ADC_VREF - sensor_offset_v) / SENSOR_SENS_V);
    ESP_LOGI(TAG, "[CALIB] Done");
#endif
#endif
}

static void print_status(float current, int adc_raw, uint32_t duty)
{
    const float duty_pct = ((float)duty / (float)PWM_DUTY_MAX) * 100.0f;
    const float motor_voltage = duty_to_motor_voltage(duty);

#if SERIAL_MONITOR
    ESP_LOGI(TAG, "----------------------------------------");
    ESP_LOGI(TAG, "  Setpoint : %.2f A", motor_pid.setpoint);
    ESP_LOGI(TAG, "  Current  : %.2f A", current);
    ESP_LOGI(TAG, "  V_sense  : %.3f V", current * SENSOR_SENS_V);
    ESP_LOGI(TAG, "  V_motor  : %.2f V avg est", motor_voltage);
    ESP_LOGI(TAG, "  ADC raw  : %d", adc_raw);
    ESP_LOGI(TAG, "  Duty     : %lu/%u (%.1f%%)", (unsigned long)duty, PWM_DUTY_MAX, duty_pct);
    ESP_LOGI(TAG, "  PID out  : %.2f", motor_pid.output);
    ESP_LOGI(TAG, "  Fault    : %s", fault_active ? "YES" : "NO");
#endif

#if SERIAL_PLOTTER
    printf("%.2f\t%.2f\t%.1f\t%.2f\t%d\n",
           motor_pid.setpoint, current, duty_pct, motor_voltage, fault_active ? 1 : 0);
#endif
}

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(300));
    ESP_LOGI(TAG, "[INIT] PWM-PID | 12V motor | Shunt+LM358 Current Sense");

    adc_init();
    filter_warm_up();

    pwm_init();
    pwm_write(0);

    calibrate_offset();

    pid_controller_init(&motor_pid, 80.0, 15.0, 0.2, 0.20, PID_SAMPLE_TIME_MS);
    pid_controller_set_output_limits(&motor_pid, 0.0, (double)PWM_DUTY_MAX);
    pid_controller_set_mode(&motor_pid, true, 0.0, millis_u32());

#if SERIAL_PLOTTER
    printf("Setpoint(A)\tMeasured(A)\tDuty%%\tMotorV(avg)\tFault\n");
#endif

    ESP_LOGI(TAG, "[INIT] Ready");

    uint32_t last_log_ms = 0;

    while (true) {
        const int adc_raw = adc_read_filtered();
        const float current = adc_to_current(adc_raw);

        if (!fault_active && current >= CURRENT_LIMIT_A) {
            fault_active = true;
            fault_time_ms = millis_u32();
            pwm_write(0);
#if SERIAL_MONITOR
            ESP_LOGE(TAG, "[FAULT] Overcurrent! %.2f A >= %.1f A; PWM OFF",
                     current, CURRENT_LIMIT_A);
#endif
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        if (fault_active) {
            const uint32_t now_ms = millis_u32();
            if ((now_ms - fault_time_ms >= FAULT_COOLDOWN_MS) &&
                (current < CURRENT_RESUME_A)) {
                fault_active = false;
#if SERIAL_MONITOR
                ESP_LOGI(TAG, "[RESUME] %.2f A; PID restarting", current);
#endif
                pid_controller_set_mode(&motor_pid, false, current, now_ms);
                motor_pid.output = 0.0;
                pwm_write(0);
                pid_controller_set_mode(&motor_pid, true, current, now_ms);
            } else {
                pwm_write(0);
                vTaskDelay(pdMS_TO_TICKS(1));
                continue;
            }
        }

        pid_controller_compute(&motor_pid, (double)current, millis_u32());

        const uint32_t duty = (uint32_t)clamp_double(motor_pid.output, 0.0, (double)PWM_DUTY_MAX);
        pwm_write(duty);

        const uint32_t now_ms = millis_u32();
        if (now_ms - last_log_ms >= LOG_INTERVAL_MS) {
            last_log_ms = now_ms;
            print_status(current, adc_raw, duty);
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
