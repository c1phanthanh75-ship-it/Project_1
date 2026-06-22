# ESP-IDF PWM PID Current Controller

This is an ESP-IDF 5.x port of 

## Hardware Map

- Current sense input: GPIO32, ADC1 channel 4
- PWM output: GPIO18, LEDC channel 0
- Motor supply: external 12 V through a driver/MOSFET stage
- PWM frequency: 20 kHz
- PWM resolution: 8 bit, duty `0..255`

GPIO32 is hard-coded as `ADC1_CHANNEL_4`. If you change the GPIO, update both `PIN_CURRENT_GPIO` and `CURRENT_ADC_CHANNEL` in `main/main.c`.

Do not connect 12 V to the ESP32 ADC or GPIO. The motor is 12 V, but the ADC input still reads the LM358 sense voltage in the ESP32-safe range below about 3.3 V.

## Calculation Notes

The code keeps the current-sense calculation comments in `main/main.c`.

Motor voltage estimate:

```text
V_motor_avg = duty / 255 * 12V
```

Current sense:

```text
V_shunt = I_motor * R_SHUNT
V_sense = V_shunt * GAIN
I_motor = (V_adc - offset) / (R_SHUNT * GAIN)
```

With `R_SHUNT=0.1 ohm` and `GAIN_ACTUAL=20`, sensitivity is `2.0 V/A`. So `0.20 A` produces about `0.40 V` at the ADC sense input.

## Build and Flash

```powershell
cd esp_idf_pwm_pid
idf.py set-target esp32
idf.py build
idf.py -p COMx flash monitor
```

Replace `COMx` with your ESP32 serial port.

