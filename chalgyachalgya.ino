#include "AiEsp32RotaryEncoder.h"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Pin Definitions
#define ROTARY_ENCODER_A_PIN 23
#define ROTARY_ENCODER_B_PIN 18
#define ROTARY_ENCODER_BUTTON_PIN 19
#define ROTARY_ENCODER_VCC_PIN -1
#define ROTARY_ENCODER_STEPS 2

#define THERMISTOR_PIN 34
#define BTS_RPWM 26     // PWM pin for Peltier
#define BTS_LPWM 27     // PWM pin for Fan
#define BTS_R_EN 14     // Enable pin for Peltier side
#define BTS_L_EN 12     // Enable pin for Fan side

// OLED Display settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
#define I2C_SDA 21
#define I2C_SCL 22

// Thermistor settings
#define SERIES_RESISTOR 100000
#define NOMINAL_RESISTANCE 60000
#define NOMINAL_TEMPERATURE 26
#define B_COEFFICIENT 3950

// PID Control Constants
#define KP 35.0
#define KI 0.8
#define KD 15.0
#define PID_INTERVAL 100  // PID calculation interval in ms

// Temperature sampling
#define NUM_SAMPLES 10
#define SAMPLE_DELAY 10

// PWM Limits
#define MIN_FAN_DUTY 128        // 50% minimum fan speed
#define MAX_FAN_DUTY 255        // 100% maximum fan speed
#define MIN_PELTIER_DUTY 64     // 25% minimum Peltier duty cycle
#define MAX_PELTIER_DUTY 255    // 100% maximum Peltier duty cycle
#define MIN_TEMP_DIFFERENCE 0.4  // Minimum temperature difference

// Fan control parameters
#define FAN_TEMP_THRESHOLD 2.0  // Temperature difference where fan reaches maximum
#define FAN_RESPONSE_FACTOR 0.7 // How aggressively fan responds to temperature difference

// Objects
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS);

// Global variables
volatile int currentSetpoint = 20;
int lastValue = 20;
bool buttonPressed = false;
float lastError = 0;
float integral = 0;
unsigned long lastPID = 0;
uint8_t currentPeltierDuty = 0;
uint8_t currentFanDuty = 0;

// Function to calculate fan speed based on temperature difference
uint8_t calculateFanDuty(float tempDifference) {
    if (tempDifference <= 0) {
        return MIN_FAN_DUTY;
    }
    
    // Calculate fan duty cycle proportional to temperature difference
    float fanRange = MAX_FAN_DUTY - MIN_FAN_DUTY;
    float normalizedDiff = min(tempDifference / FAN_TEMP_THRESHOLD, 1.0);
    float fanOutput = MIN_FAN_DUTY + (fanRange * normalizedDiff * FAN_RESPONSE_FACTOR);
    
    return (uint8_t)constrain(fanOutput, MIN_FAN_DUTY, MAX_FAN_DUTY);
}

// Function to update PWM duty cycle
void updatePWM(uint8_t pin, uint8_t duty) {
    if (pin == BTS_RPWM) {
        currentPeltierDuty = duty;
        analogWrite(pin, duty);
    } else if (pin == BTS_LPWM) {
        currentFanDuty = duty;
        analogWrite(pin, duty);
    }
}

// Function to read PWM duty cycle
uint8_t getPWMDuty(uint8_t pin) {
    return (pin == BTS_RPWM) ? currentPeltierDuty : currentFanDuty;
}

// Improved temperature reading function with median filtering
float readTemperature() {
    float samples[NUM_SAMPLES];
    
    for(int i = 0; i < NUM_SAMPLES; i++) {
        int analogValue = analogRead(THERMISTOR_PIN);
        float resistance = SERIES_RESISTOR / ((4095.0 / analogValue) - 1.0);
        
        float steinhart = resistance / NOMINAL_RESISTANCE;
        steinhart = log(steinhart);
        steinhart /= B_COEFFICIENT;
        steinhart += 1.0 / (NOMINAL_TEMPERATURE + 273.15);
        steinhart = 1.0 / steinhart;
        
        samples[i] = steinhart - 273.15;
        delay(SAMPLE_DELAY);
    }
    
    // Calculate median
    for(int i = 0; i < NUM_SAMPLES-1; i++) {
        for(int j = i+1; j < NUM_SAMPLES; j++) {
            if(samples[j] < samples[i]) {
                float temp = samples[i];
                samples[i] = samples[j];
                samples[j] = temp;
            }
        }
    }
    
    return samples[NUM_SAMPLES/2];
}

// Revised PID controller function with variable fan speed
void updatePID(float currentTemp) {
    unsigned long now = millis();
    if (now - lastPID < PID_INTERVAL) return;
    
    float dt = (now - lastPID) / 1000.0;
    lastPID = now;
    
    // Calculate error (positive error means cooling is needed)
    float error = currentTemp - currentSetpoint;
    
    // Calculate fan duty cycle based on temperature difference
    uint8_t fanDuty = calculateFanDuty(error);
    updatePWM(BTS_LPWM, fanDuty);
    
    // If temperature is above setpoint, increase cooling
    if (error > MIN_TEMP_DIFFERENCE) {
        // Calculate PID terms
        float proportional = KP * error;
        integral += KI * error * dt;
        float derivative = KD * (error - lastError) / dt;
        
        // Anti-windup for integral term
        integral = constrain(integral, 0, MAX_PELTIER_DUTY);
        
        // Calculate output
        float output = proportional + integral + derivative;
        
        // Map the output to the new range (MIN_PELTIER_DUTY to MAX_PELTIER_DUTY)
        output = map(constrain(output, 0, MAX_PELTIER_DUTY), 
                    0, MAX_PELTIER_DUTY,
                    MIN_PELTIER_DUTY, MAX_PELTIER_DUTY);
        
        // Update Peltier PWM
        updatePWM(BTS_RPWM, (uint8_t)output);
        
    } else if (error < -MIN_TEMP_DIFFERENCE) {
        // If temperature is too low, set Peltier to minimum duty cycle
        updatePWM(BTS_RPWM, MIN_PELTIER_DUTY);
        integral = 0;
    } else {
        // Within acceptable range - maintain minimum cooling
        updatePWM(BTS_RPWM, MIN_PELTIER_DUTY);
    }
    
    lastError = error;
    
    // Debug output
    Serial.print("Temp: "); Serial.print(currentTemp);
    Serial.print(" Set: "); Serial.print(currentSetpoint);
    Serial.print(" Error: "); Serial.print(error);
    Serial.print(" Peltier: "); Serial.print(getPWMDuty(BTS_RPWM));
    Serial.print(" Fan: "); Serial.println(getPWMDuty(BTS_LPWM));
}

void rotary_onButtonClick() {
    static unsigned long lastTimePressed = 0;
    unsigned long currentTime = millis();
    
    if (currentTime - lastTimePressed < 500) {
        return;
    }
    
    lastTimePressed = currentTime;
    currentSetpoint = 20;
    lastValue = 20;
    rotaryEncoder.setEncoderValue(20);
    buttonPressed = true;
    
    Serial.println("Button pressed: Value reset to 20");
}

void updateDisplay(float currentTemp) {
    display.clearDisplay();
    
    // Display Set Temperature
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print("Set: ");
    display.print(currentSetpoint);
    display.println("C");
    
    // Display Current Temperature
    display.setCursor(0, 16);
    display.print("Current: ");
    display.print(currentTemp, 1);
    display.println("C");
    
    // Display PWM values
    display.setCursor(0, 32);
    display.print("PEL: ");
    display.print((getPWMDuty(BTS_RPWM) * 100) / 255);
    display.print("%");
    
    display.setCursor(0, 48);
    display.print("Fan: ");
    display.print((getPWMDuty(BTS_LPWM) * 100) / 255);
    display.println("%");
    
    display.display();
}

void rotary_loop() {
    if (rotaryEncoder.encoderChanged()) {
        int value = rotaryEncoder.readEncoder();
        
        if (value < 10) {
            value = 10;
            rotaryEncoder.setEncoderValue(10);
        } else if (value > 25) {
            value = 25;
            rotaryEncoder.setEncoderValue(25);
        }
        
        if (value != lastValue || buttonPressed) {
            currentSetpoint = value;
            lastValue = value;
            buttonPressed = false;
            Serial.print("New setpoint: ");
            Serial.println(value);
        }
    }
    
    if (rotaryEncoder.isEncoderButtonClicked()) {
        rotary_onButtonClick();
    }
}

void IRAM_ATTR readEncoderISR() {
    rotaryEncoder.readEncoder_ISR();
}

void setup() {
    Serial.begin(115200);
    
    // Configure PWM pins
    pinMode(BTS_RPWM, OUTPUT);
    pinMode(BTS_LPWM, OUTPUT);
    
    // Configure BTS7960 enable pins
    pinMode(BTS_R_EN, OUTPUT);
    pinMode(BTS_L_EN, OUTPUT);
    digitalWrite(BTS_R_EN, HIGH);
    digitalWrite(BTS_L_EN, HIGH);
    
    // Set initial PWM values - both start at minimum duty cycle
    analogWrite(BTS_RPWM, MIN_PELTIER_DUTY);
    analogWrite(BTS_LPWM, MIN_FAN_DUTY);
    
    Serial.println("Starting setup...");

    Wire.begin(I2C_SDA, I2C_SCL);
    
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        for(;;);
    }
    
    display.clearDisplay();
    
    rotaryEncoder.begin();
    rotaryEncoder.setup(readEncoderISR);
    rotaryEncoder.setBoundaries(10, 25, false);
    rotaryEncoder.disableAcceleration();
    rotaryEncoder.setEncoderValue(20);
    
    analogReadResolution(12);
    
    // Show initial values
    float initialTemp = readTemperature();
    updateDisplay(initialTemp);
    
    Serial.println("Setup complete");
}

void loop() {
    static unsigned long lastDisplay = 0;
    unsigned long currentMillis = millis();
    
    rotary_loop();
    
    // Update temperature reading and control system
    if (currentMillis - lastDisplay >= 500) {
        float currentTemp = readTemperature();
        updatePID(currentTemp);
        updateDisplay(currentTemp);
        lastDisplay = currentMillis;
    }
    
    delay(50);
}