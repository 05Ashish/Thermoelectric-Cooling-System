# Temperature Control System with Rotary Encoder, OLED Display, and PWM

This project implements a temperature control system for an ESP32 that uses a rotary encoder to set a target temperature. The system reads temperature data from a thermistor, displays the current and target temperatures on an OLED screen, and adjusts a Peltier cooler and fan based on a PID control system. REFER TO [final_code.ino](https://github.com/05Ashish/Thermoelectric-Cooling-System/blob/main/final_code.ino)

## Features
- **Rotary Encoder Control**: Adjust the temperature setpoint (range: 10°C to 25°C).
- **PID-Controlled Cooling**: Manages Peltier and fan duty cycles to maintain the target temperature.
- **OLED Display**: Real-time display of set temperature, current temperature, and PWM duty cycles for fan and Peltier.
- **Temperature Reading**: Reads and filters data from a thermistor for stable temperature values.
- **PWM Management**: Controls the duty cycles of fan and Peltier based on temperature.

## Hardware Components
- **ESP32 Microcontroller**
- **Rotary Encoder**
- **SSD1306 OLED Display (128x64)**
- **Thermistor** for temperature measurement
- **Peltier Module** with fan for active cooling
- **BTS7960 Motor Driver Module** for PWM control of the Peltier and fan

## Pin Configuration
- **Rotary Encoder**:  
  - A Pin: GPIO 23
  - B Pin: GPIO 18
  - Button Pin: GPIO 19

- **Thermistor**: GPIO 34
- **BTS7960 Motor Driver**:  
  - RPWM (Peltier): GPIO 26
  - LPWM (Fan): GPIO 27
  - R_EN (Enable Peltier): GPIO 14
  - L_EN (Enable Fan): GPIO 12

- **OLED Display**:  
  - SDA: GPIO 21
  - SCL: GPIO 22

## Dependencies
- [AiEsp32RotaryEncoder](https://github.com/igorantolic/ai-esp32-rotary-encoder)
- [Adafruit GFX Library](https://github.com/adafruit/Adafruit-GFX-Library)
- [Adafruit SSD1306](https://github.com/adafruit/Adafruit_SSD1306)

Install these libraries in the Arduino IDE Library Manager.

## Code Details

### PID Control
The PID controller adjusts the duty cycles for the Peltier cooler and fan based on the temperature error, calculated as the difference between the setpoint and current temperature.

### Temperature Sampling
A median filter is applied to reduce noise from the thermistor readings. The temperature is calculated using the Steinhart-Hart equation.

### Rotary Encoder
- Sets the target temperature between 10°C and 25°C.
- Resets the target temperature to 20°C when the encoder button is pressed.

### OLED Display
Displays:
- Target temperature
- Current temperature
- Peltier and fan duty cycles as percentages

## Usage

1. **Wiring**: Connect components as per the pin configuration above.
2. **Set Temperature**: Rotate the encoder to set the target temperature.
3. **Start Control**: Once powered, the system maintains the set temperature, adjusting Peltier and fan duty cycles as needed.

