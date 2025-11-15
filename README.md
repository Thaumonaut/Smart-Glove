# Smart Glove

Assistive wearable device for people with Periodic Paralysis - enables emergency alerts and health monitoring when large muscle groups are weak but finger movement is still possible.

## Project Timeline
- **Total:** 3 weeks (class project)
- **Status:** Week 2 of 3
- **Goal:** Working prototype with core features

## Core Features
- 🚨 Emergency alert system via gestures
- 📞 Bluetooth hands-free calling (A2DP/HFP)
- ❤️ Health monitoring (HR, O2, future: ECG for K+ level indicators)
- 🎯 Menu navigation via thumb gestures (APDS9960)
- 📳 Haptic feedback for user confirmation

## Hardware

### Main Board
- **IdeaSpark ESP32** (original ESP32 with Bluetooth Classic)
  - Note: Power via 5V + GND pins (power IC damaged)

### Currently Connected (Breadboard)
- MAX98357A I2S amplifier + 3W speaker
- INMP441 I2S MEMS microphone
- HW-123 gyro/accelerometer (MPU6050)

### On Order
- **APDS9960** gesture/proximity sensor (Amazon - order ASAP!)
- **3x Flex sensors** (AliExpress - arriving next week)
- **MAX30102** HR/O2 sensor (AliExpress - arriving next week)
- **Pressure pads** 20g-2kg range (arriving today)

## Pin Mapping

See [src/config.h](src/config.h) for complete pin assignments.

### I2S Audio
- BCK (Bit Clock): GPIO 27 (shared)
- WS (Word Select): GPIO 26 (shared)
- Speaker Data Out: GPIO 25 (MAX98357A)
- Mic Data In: GPIO 32 (INMP441)

### I2C Bus (Shared)
- SDA: GPIO 21
- SCL: GPIO 22
- Devices: APDS9960 (0x39), MAX30102 (0x57), MPU6050 (0x68)

### Analog Inputs (ADC1)
- GPIO 36: Flex sensor - Index finger
- GPIO 39: Flex sensor - Middle/Ring finger
- GPIO 34: Flex sensor - Pinky finger
- GPIO 35: Pressure pad (gesture activation)
- GPIO 33: Pressure pad 2 (optional)

### Digital GPIO
- GPIO 23: Vibration motor (PWM)

## Getting Started

### Prerequisites
- [PlatformIO](https://platformio.org/) (handles ESP-IDF toolchain automatically)
- CLion (optional, for better IDE experience)
- USB cable for ESP32 programming

### Building the Project
```bash
# Build
pio run

# Upload to device
pio run -t upload

# Open serial monitor
pio run -t monitor

# Clean build
pio run -t clean
```

### Testing Individual Components

The `test/` directory contains standalone test programs for each hardware component. To run a test:

1. Edit `src/CMakeLists.txt` - comment out `main.cpp`
2. Copy desired test file to `src/main.cpp` temporarily, OR:
3. Use PlatformIO test environments (see below)

#### Available Tests
- **i2s_speaker_test.cpp** - Play 440Hz tone through speaker
- **i2s_mic_test.cpp** - Show microphone audio levels (VU meter)
- **i2c_scanner_test.cpp** - Detect I2C devices on bus
- **bt_a2dp_sink_test.cpp** - Receive and play Bluetooth audio from phone

### Project Structure
```
Smart-Glove/
├── platformio.ini          # PlatformIO configuration
├── CMakeLists.txt          # CLion compatibility
├── src/
│   ├── main.cpp            # Main application entry point
│   ├── config.h            # Pin definitions & constants
│   ├── CMakeLists.txt      # ESP-IDF component config
│   ├── bluetooth/          # BT Classic A2DP/HFP handlers
│   ├── audio/              # I2S audio streaming
│   ├── sensors/            # Flex, gyro, APDS9960, MAX30102
│   ├── gestures/           # Pattern recognition
│   ├── ui/                 # LVGL display screens
│   └── power/              # Power management
└── test/                   # Component test programs
```

## Development Notes

### Bluetooth Audio
- Uses **ESP-IDF framework** for full Bluetooth Classic stack access
- A2DP Sink: Receive audio from phone
- HFP Client: Hands-free calling with microphone
- Requires original ESP32 (not S3/C3/C6 which only have BLE)

### Gesture Recognition
- **"Call Me" gesture**: Index/Middle/Ring curled, Pinky + Thumb extended
- Thumb swipes (APDS9960): Menu navigation
- Pressure pad: Activates gesture sensor (power saving)

### Power Management
- Gesture sensor: Only active when pressure pad squeezed
- Speaker/amp: Only during calls
- Display: Sleep after 10s idle
- Target: Full day battery life

### Physical Design (Prototype)
- Base: Spandex cosplay glove
- Mounting: Leather/foam backing, fabric pockets, Velcro, elastic straps
- No 3D printing for prototype (timeline constraint)
- IdeaSpark on back of hand (has bulky soldered headers)

## Week 2 Priorities
1. ✅ Project structure setup with ESP-IDF
2. Test existing breadboard components:
   - [ ] I2S speaker playback (use i2s_speaker_test.cpp)
   - [ ] I2S microphone recording (use i2s_mic_test.cpp)
   - [ ] I2C device detection (use i2c_scanner_test.cpp)
   - [ ] Bluetooth A2DP audio reception (use bt_a2dp_sink_test.cpp)
3. Order APDS9960 from Amazon (fast shipping)
4. Design basic UI screens with LVGL Pro
5. Begin Bluetooth calling integration

## Future Iterations
- Custom PCB with low-profile components
- Dual-material flexible housing
- Surface mount components
- Modular design (XIAO boards when LE Audio matures)

## Context
Built for someone with Periodic Paralysis - episodes cause inability to move arms/legs but finger movement remains. Device must be comfortable for all-day wear and usable during weak episodes.

PP often involves potassium imbalances affecting heart rhythm, making heart monitoring especially important.

## License
[Add your license here]

## Author
Jacob (jek) - Class project prototype
