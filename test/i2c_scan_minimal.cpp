#include <Arduino.h>
#include <Wire.h>

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== I2C Scanner ===");
    Wire.begin(21, 22);  // SDA=21, SCL=22
}

void loop() {
    int found = 0;
    Serial.println("\nScanning I2C bus (0x03-0x77)");
    Serial.println("     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F");
    Serial.print("00:         ");
    
    for(uint8_t addr = 3; addr < 0x78; addr++) {
        if(addr % 16 == 0) {
            Serial.printf("\n%02X: ", addr);
        }
        
        Wire.beginTransmission(addr);
        if(Wire.endTransmission() == 0) {
            Serial.printf("%02X ", addr);
            found++;
        } else {
            Serial.print("-- ");
        }
        delay(5);
    }
    
    Serial.printf("\n\nFound %d device(s)\n", found);
    if(found > 0) {
        Serial.println("\nKnown addresses:");
        Wire.beginTransmission(0x57);
        if(Wire.endTransmission() == 0) Serial.println("  0x57: MAX30102 (Heart Rate/O2)");
        Wire.beginTransmission(0x68);
        if(Wire.endTransmission() == 0) Serial.println("  0x68: MPU6050 (Gyro/Accel)");
        Wire.beginTransmission(0x39);
        if(Wire.endTransmission() == 0) Serial.println("  0x39: APDS9960 (Gesture)");
    } else {
        Serial.println("\n⚠ NO DEVICES - Check wiring & power!");
    }
    
    Serial.println("\nNext scan in 5 seconds...");
    delay(5000);
}
