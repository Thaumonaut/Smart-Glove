#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// SMART GLOVE PIN CONFIGURATION
// ============================================================================
// IdeaSpark ESP32 Pin Mapping for all sensors and peripherals
// Note: Power via 5V + GND pins (USB data works, power IC fried)

// ============================================================================
// I2S AUDIO PINS (MAX98357A Speaker + INMP441 Microphone)
// ============================================================================
// Shared clock signals between mic and speaker
#define I2S_BCK_IO      27  // Bit Clock (shared)
#define I2S_WS_IO       26  // Word Select / LRC (shared)

// Speaker output (MAX98357A on I2S port 0)
#define I2S_SPKR_DO_IO  25  // Data Out to speaker amp
#define I2S_GAIN_PIN    5   // GAIN control pin (HIGH=15dB, LOW=9dB, Float=12dB)

// Microphone input (INMP441 on I2S port 1)
#define I2S_MIC_DI_IO   32  // Data In from microphone (changed from 33)

// I2S Port assignments
#define I2S_PORT_SPEAKER    I2S_NUM_0
#define I2S_PORT_MIC        I2S_NUM_1

// Audio configuration
#define I2S_SAMPLE_RATE     16000   // 16kHz for voice (can go up to 44.1kHz)
#define I2S_BITS_PER_SAMPLE 16      // 16-bit audio
#define I2S_CHANNELS        1       // Mono

// ============================================================================
// I2C BUS (Shared by multiple sensors)
// ============================================================================
#define I2C_SDA_IO      21  // Data line
#define I2C_SCL_IO      22  // Clock line
#define I2C_FREQ_HZ     100000  // 100kHz standard mode (reduced for reliability)

// I2C Device Addresses (verify with i2c scanner if needed)
#define APDS9960_ADDR   0x39  // Gesture/proximity sensor
#define MAX30102_ADDR   0x57  // Heart rate/O2 sensor
#define MPU6050_ADDR    0x68  // Gyro/Accelerometer (HW-123) - can be 0x69 if AD0 is HIGH

// ============================================================================
// ANALOG INPUTS (Flex Sensors & Pressure Pads)
// ============================================================================
// ADC1 channels (ADC1 can be used with WiFi/BT, ADC2 cannot)
#define FLEX_INDEX_PIN      36  // ADC1_CH0 - Index finger
#define FLEX_MIDDLE_PIN     39  // ADC1_CH3 - Middle/Ring finger
#define FLEX_PINKY_PIN      34  // ADC1_CH6 - Pinky finger

#define PRESSURE_PAD_PIN    35  // ADC1_CH7 - Main pressure pad (gesture activation)
#define PRESSURE_PAD2_PIN   -1  // Disabled (GPIO 33 used for mic)

// ADC configuration
#define ADC_ATTEN       ADC_ATTEN_DB_11  // 0-3.3V range
#define ADC_WIDTH       ADC_WIDTH_BIT_12 // 12-bit resolution (0-4095)

// Flex sensor calibration thresholds (to be determined experimentally)
#define FLEX_THRESHOLD_CURLED   2000  // Value when finger is fully curled
#define FLEX_THRESHOLD_STRAIGHT 1000  // Value when finger is straight

// Pressure pad thresholds
#define PRESSURE_THRESHOLD_ACTIVE 500  // Activate gesture sensor when exceeded

// ============================================================================
// DIGITAL GPIO PINS
// ============================================================================
#define VIBRATION_MOTOR_PIN     19  // Haptic feedback motor (PWM capable)

// Future expansion pins (available if needed)
#define GPIO_SPARE_1    16
#define GPIO_SPARE_2    17

// ============================================================================
// DISPLAY CONFIGURATION (IdeaSpark integrated display)
// ============================================================================
// IdeaSpark ESP32 with ST7789 1.14" TFT display (135x240)
#define TFT_MOSI        23
#define TFT_SCLK        18
#define TFT_CS          15
#define TFT_DC          2
#define TFT_RST         4
#define TFT_BL          32  // Backlight

// Display resolution for ST7789 1.14" display
#define TFT_WIDTH       135
#define TFT_HEIGHT      240

// ============================================================================
// BLUETOOTH CONFIGURATION
// ============================================================================
#define BT_DEVICE_NAME  "SmartGlove"
#define BT_PIN_CODE     "1234"

// ============================================================================
// POWER MANAGEMENT
// ============================================================================
#define BATTERY_ADC_PIN     -1  // Not currently monitored (using external battery)
#define LOW_POWER_MODE_TIMEOUT  30000  // 30 seconds idle before power save

// ============================================================================
// TIMING CONSTANTS
// ============================================================================
#define SENSOR_READ_INTERVAL_MS     50   // Read sensors every 50ms (20Hz)
#define GESTURE_TIMEOUT_MS          3000 // Reset gesture detection after 3s
#define VIBRATION_DURATION_MS       200  // Haptic feedback duration
#define DISPLAY_SLEEP_TIMEOUT_MS    10000 // Display sleep after 10s idle

// ============================================================================
// GESTURE DETECTION CONSTANTS
// ============================================================================
// "Call Me" gesture: Index/Middle/Ring curled, Pinky + Thumb extended
#define GESTURE_CALL_ME_CONFIDENCE  0.85  // 85% confidence threshold

// APDS9960 gesture directions
#define GESTURE_UP      0
#define GESTURE_DOWN    1
#define GESTURE_LEFT    2
#define GESTURE_RIGHT   3

// ============================================================================
// DEBUG CONFIGURATION
// ============================================================================
#define DEBUG_SERIAL_ENABLED    1
#define DEBUG_BAUD_RATE         115200

// Debug verbosity levels
#define DEBUG_LEVEL_ERROR   1
#define DEBUG_LEVEL_WARN    2
#define DEBUG_LEVEL_INFO    3
#define DEBUG_LEVEL_DEBUG   4
#define DEBUG_LEVEL_VERBOSE 5

#define DEBUG_LEVEL DEBUG_LEVEL_INFO

// ============================================================================
// COMPILE-TIME FEATURE FLAGS
// ============================================================================
#define ENABLE_BLUETOOTH_AUDIO  1
#define ENABLE_GESTURE_DETECT   1
#define ENABLE_HEART_RATE       1  // Set to 0 if using Colmi ring instead
#define ENABLE_GYRO_ACCEL       1
#define ENABLE_VIBRATION        1
#define ENABLE_LVGL_UI          1

#endif // CONFIG_H
