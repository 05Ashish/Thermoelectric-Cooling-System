// ESP32 Register Base Addresses
#define DR_REG_GPIO_BASE             0x3FF44000
#define DR_REG_IO_MUX_BASE          0x3FF49000
#define DR_REG_RTCCNTL_BASE         0x3FF48000
#define DR_REG_SENS_BASE            0x3FF48800
#define DR_REG_PCNT_BASE            0x3FF57000
#define DR_REG_MCPWM_BASE           0x3FF6E000
#define DR_REG_DPORT_BASE           0x3FF00000

// GPIO Registers
#define GPIO_ENABLE_W1TS_REG        (DR_REG_GPIO_BASE + 0x0020)
#define GPIO_ENABLE_W1TC_REG        (DR_REG_GPIO_BASE + 0x0024)
#define GPIO_OUT_W1TS_REG           (DR_REG_GPIO_BASE + 0x0008)
#define GPIO_OUT_W1TC_REG           (DR_REG_GPIO_BASE + 0x000C)
#define GPIO_IN_REG                 (DR_REG_GPIO_BASE + 0x003C)
#define GPIO_PIN_REG(n)             (DR_REG_GPIO_BASE + 0x0088 + (n) * 4)

// IO MUX Registers
#define IO_MUX_PIN_REG(n)           (DR_REG_IO_MUX_BASE + 0x0004 + (n) * 4)

// ADC Registers
#define SENS_SAR_START_FORCE_REG    (DR_REG_SENS_BASE + 0x002C)
#define SENS_SAR_READ_CTRL_REG      (DR_REG_SENS_BASE + 0x0030)
#define SENS_SAR_MEAS_START1_REG    (DR_REG_SENS_BASE + 0x0034)
#define SENS_SAR_MEAS_START2_REG    (DR_REG_SENS_BASE + 0x0038)
#define SENS_SAR_ATTEN1_REG         (DR_REG_SENS_BASE + 0x003C)
#define SENS_SAR_ATTEN2_REG         (DR_REG_SENS_BASE + 0x0040)

// PWM Registers
#define MCPWM_CLK_CFG_REG          (DR_REG_MCPWM_BASE + 0x0000)
#define MCPWM_TIMER_CFG0_REG       (DR_REG_MCPWM_BASE + 0x0004)
#define MCPWM_TIMER_CFG1_REG       (DR_REG_MCPWM_BASE + 0x0008)
#define MCPWM_GEN0_CFG0_REG        (DR_REG_MCPWM_BASE + 0x000C)
#define MCPWM_GEN1_CFG0_REG        (DR_REG_MCPWM_BASE + 0x0010)

// Clock and Reset Registers
#define DPORT_PERIP_CLK_EN_REG     (DR_REG_DPORT_BASE + 0x00AC)
#define DPORT_PERIP_RST_EN_REG     (DR_REG_DPORT_BASE + 0x00B0)

// Pin Definitions
#define PIN_THERMISTOR              34
#define PIN_BTS_RPWM               26
#define PIN_BTS_LPWM               27
#define PIN_BTS_R_EN               14
#define PIN_BTS_L_EN               12
#define PIN_ROTARY_A               23
#define PIN_ROTARY_B               18
#define PIN_ROTARY_BTN             19

// Control Parameters
#define TEMP_SAMPLES               10
#define PWM_FREQUENCY             5000
#define MIN_PELTIER_DUTY          64
#define MAX_PELTIER_DUTY          255
#define MIN_FAN_DUTY              128
#define MAX_FAN_DUTY              255

// Function Prototypes
static void init_clocks(void);
static void init_gpio(void);
static void init_adc(void);
static void init_pwm(void);
static void delay_cycles(uint32_t cycles);
static uint32_t get_ccount(void);
static void write_reg(uint32_t addr, uint32_t value);
static uint32_t read_reg(uint32_t addr);

// Register access functions
static inline void write_reg(uint32_t addr, uint32_t value) {
    *((volatile uint32_t *)addr) = value;
}

static inline uint32_t read_reg(uint32_t addr) {
    return *((volatile uint32_t *)addr);
}

// Cycle counting for timing
static inline uint32_t get_ccount(void) {
    uint32_t ccount;
    asm volatile("rsr.ccount %0" : "=r" (ccount));
    return ccount;
}

static void delay_cycles(uint32_t cycles) {
    uint32_t start = get_ccount();
    while (get_ccount() - start < cycles) {
        asm volatile("nop");
    }
}

// Initialize system clocks
static void init_clocks(void) {
    // Enable peripheral clocks
    uint32_t clock_en = read_reg(DPORT_PERIP_CLK_EN_REG);
    clock_en |= (1 << 2);  // GPIO
    clock_en |= (1 << 3);  // ADC
    clock_en |= (1 << 13); // MCPWM
    write_reg(DPORT_PERIP_CLK_EN_REG, clock_en);
    
    // Clear peripheral resets
    uint32_t reset = read_reg(DPORT_PERIP_RST_EN_REG);
    reset &= ~(1 << 2);  // GPIO
    reset &= ~(1 << 3);  // ADC
    reset &= ~(1 << 13); // MCPWM
    write_reg(DPORT_PERIP_RST_EN_REG, reset);
}

// Initialize GPIO pins
static void init_gpio(void) {
    // Configure GPIO function for each pin
    write_reg(IO_MUX_PIN_REG(PIN_BTS_RPWM), (2 << 12) | (1 << 9));  // Set as GPIO
    write_reg(IO_MUX_PIN_REG(PIN_BTS_LPWM), (2 << 12) | (1 << 9));
    write_reg(IO_MUX_PIN_REG(PIN_BTS_R_EN), (2 << 12) | (1 << 9));
    write_reg(IO_MUX_PIN_REG(PIN_BTS_L_EN), (2 << 12) | (1 << 9));
    
    // Set GPIO direction
    uint32_t gpio_enable = read_reg(GPIO_ENABLE_W1TS_REG);
    gpio_enable |= (1 << PIN_BTS_RPWM) | (1 << PIN_BTS_LPWM) |
                   (1 << PIN_BTS_R_EN) | (1 << PIN_BTS_L_EN);
    write_reg(GPIO_ENABLE_W1TS_REG, gpio_enable);
    
    // Set initial output states
    write_reg(GPIO_OUT_W1TS_REG, (1 << PIN_BTS_R_EN) | (1 << PIN_BTS_L_EN));
}

// Initialize ADC for temperature reading
static void init_adc(void) {
    // Configure ADC attenuation (11dB) for wider voltage range
    uint32_t atten_val = read_reg(SENS_SAR_ATTEN1_REG);
    atten_val |= (3 << (PIN_THERMISTOR * 2));  // 11dB attenuation
    write_reg(SENS_SAR_ATTEN1_REG, atten_val);
    
    // Enable ADC1
    write_reg(SENS_SAR_READ_CTRL_REG, 0x00000000);
    write_reg(SENS_SAR_MEAS_START1_REG, 0xFFFFFFFF);
}

// Initialize PWM for Peltier and Fan control
static void init_pwm(void) {
    // Configure MCPWM clock
    write_reg(MCPWM_CLK_CFG_REG, (80 << 8));  // 80MHz / 80 = 1MHz base clock
    
    // Configure Timer 0
    write_reg(MCPWM_TIMER_CFG0_REG, 
        (1 << 30) |                   // Enable timer
        ((1000000/PWM_FREQUENCY) << 0) // Set period for desired frequency
    );
    
    // Configure PWM generators
    write_reg(MCPWM_GEN0_CFG0_REG, 
        (1 << 0) |     // Enable generator
        (2 << 1) |     // Active high output
        (0 << 4)       // Initial duty cycle
    );
    
    write_reg(MCPWM_GEN1_CFG0_REG,
        (1 << 0) |     // Enable generator
        (2 << 1) |     // Active high output
        (0 << 4)       // Initial duty cycle
    );
}

// Read ADC value for temperature
static uint32_t read_adc(void) {
    // Start ADC conversion
    write_reg(SENS_SAR_START_FORCE_REG, 1);
    
    // Wait for conversion to complete (typically takes about 10μs)
    delay_cycles(1000);  // Approximate 10μs at 100MHz
    
    // Read ADC value
    return (read_reg(SENS_SAR_MEAS_START1_REG) >> PIN_THERMISTOR) & 0xFFF;
}

// Set PWM duty cycle (0-255)
static void set_pwm_duty(uint8_t channel, uint8_t duty) {
    uint32_t reg = (channel == 0) ? MCPWM_GEN0_CFG0_REG : MCPWM_GEN1_CFG0_REG;
    uint32_t cfg = read_reg(reg);
    cfg &= ~(0xFF << 4);  // Clear duty cycle bits
    cfg |= (duty << 4);   // Set new duty cycle
    write_reg(reg, cfg);
}

// Arduino setup function
void setup() {
    // Disable watchdog
    write_reg(DR_REG_RTCCNTL_BASE + 0x8C, 0);
    
    // Initialize peripherals
    init_clocks();
    init_gpio();
    init_adc();
    init_pwm();
    
    // Initial PWM settings
    set_pwm_duty(0, MIN_PELTIER_DUTY);
    set_pwm_duty(1, MIN_FAN_DUTY);
}

// Arduino loop function
void loop() {
    uint32_t temp_raw = 0;
    
    // Average multiple temperature readings
    for (int i = 0; i < TEMP_SAMPLES; i++) {
        temp_raw += read_adc();
        delay_cycles(100000);  // ~1ms between samples
    }
    temp_raw /= TEMP_SAMPLES;
    
    // Convert to temperature (simplified calculation)
    float temp = (float)temp_raw * 3.3f / 4096.0f;  // Convert to voltage
    temp = (temp - 0.5f) * 100.0f;  // Approximate conversion to Celsius
    
    // Basic temperature control
    if (temp > 25.0f) {
        set_pwm_duty(0, MAX_PELTIER_DUTY);  // Max cooling
        set_pwm_duty(1, MAX_FAN_DUTY);      // Max fan
    } else if (temp < 20.0f) {
        set_pwm_duty(0, MIN_PELTIER_DUTY);  // Min cooling
        set_pwm_duty(1, MIN_FAN_DUTY);      // Min fan
    }
    
    delay_cycles(5000000);  // ~50ms delay at 100MHz
}