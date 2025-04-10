// Include necessary libraries
#include <Wire.h>               // For I2C communication
#include <Arduino.h>           // Core Arduino functionalities
#include "stdio.h"             // Standard I/O operations
#include <stdlib.h>            // Standard utilities
#include "hardware/pio.h"      // Access to PIO hardware on RP2040
#include "hardware/gpio.h"     // GPIO control
#include "hardware/pwm.h"      // PWM control
#include "hardware/dma.h"      // DMA access
#include "hardware/clocks.h"   // Clock control
#include "ws2812.pio.h"        // Custom PIO program for NeoPixel (WS2812)

//------------------IR Remote Key Definitions--------------------
#define number_1 69
#define number_2 70
#define number_3 71
#define number_4 68
#define number_5 64
#define number_6 67
#define number_7 7
#define number_8 21
#define number_9 9
#define number_0 25
#define button_up 24
#define button_down 82
#define button_right 90
#define button_left 8
#define button_ok 28

/******************************
 SSD1306 OLED DISPLAY CLASS
*******************************/
class SSD1306 {
public:
    SSD1306(uint8_t address, uint8_t width, uint8_t height); // Constructor
    void init();                                              // Initialize display
    void clear();                                             // Clear the screen
    void setCursor(uint8_t x, uint8_t y);                     // Set text cursor position
    void print(const char* text);                             // Print text on the screen
    void show();                                              // Send buffer to display
    void drawBitmap(int16_t x, int16_t y, const uint8_t *bitmap, uint8_t w, uint8_t h); // Draw image

private:
    uint8_t _address;                                         // I2C address of the display
    uint8_t _width, _height;                                  // Display dimensions
    uint8_t _cursorX, _cursorY;                               // Cursor position
    uint8_t _buffer[1024];                                    // Display memory buffer

    void sendCommand(uint8_t command);                        // Send command to OLED
    void sendData(uint8_t data);                              // Send data to OLED
    void drawChar(int16_t x, int16_t y, unsigned char c);     // Draw single character
    void drawPixel(uint8_t x, uint8_t y, bool color);         // Draw a pixel
};

/******************************
 Servo Motor Class (PWM Control)
*******************************/
class ServoSimple {
public:
    ServoSimple(uint gpio_pin);       // Constructor
    void begin();                     // Init PWM for servo
    void setAngle(int angle);         // Set servo angle (0-180)

private:
    uint pin;                         // GPIO pin number
    uint slice_num;                   // PWM slice number
    void restartPWM();               // Reinitialize PWM settings
};

/******************************
 SHTC3 Temperature & Humidity Sensor
*******************************/
class SHTC3 {
public:
    SHTC3(uint8_t address = 0x70);      // Constructor with default I2C address
    void begin();                       // Initialize sensor
    float readTemperature();           // Return temperature in °C
    float readHumidity();              // Return humidity percentage
    void wakeUp();                     // Wake the sensor
    void sleep();                      // Put the sensor to sleep

private:
    uint8_t _address;                  // I2C address
    void sendCommand(uint8_t high, uint8_t low); // Send a command
    void readData(uint8_t *data, uint8_t len);   // Read data from sensor
};

/******************************
 DHT11 Temperature & Humidity Sensor
*******************************/
class DHT11 {
public:
    DHT11(uint8_t pin);               // Constructor: GPIO pin connected to DHT11
    void begin(uint8_t usec = 55);    // Initialize the sensor
    float readTemperature();         // Get temperature in °C
    float readHumidity();            // Get humidity in %
    bool read();                     // Read sensor data into buffer

private:
    uint8_t data[5];                 // Raw data buffer
    uint8_t _pin;                    // GPIO pin number
    uint32_t _lastreadtime;         // Last time data was read
    uint32_t _maxcycles;            // Timeout cycles
    uint8_t pullTime;               // Pull-up delay time
    bool _lastresult;               // Store the last successful read result

    uint32_t expectPulse(bool level); // Wait for a signal pulse and measure its duration
};

/******************************
 NeoPixel (WS2812) LED Strip Control
*******************************/
class NeoPixel {
public:
    NeoPixel(byte pinNumber, uint16_t numberOfPixels);                         // Constructor using default PIO
    NeoPixel(byte pinNumber, uint16_t numberOfPixels, PIO pio, uint sm);      // Constructor using specific PIO and state machine
    virtual ~NeoPixel(){};                                                    // Destructor

    void Init(byte pinNumber, uint16_t numberOfPixels);                       // Initialize LED strip
    void setPixelColor(uint16_t pixel_number, uint8_t r=0, uint8_t g=0, uint8_t b=0); // Set specific LED color
    void Fill(uint8_t r=0, uint8_t g=0, uint8_t b=0);                          // Fill entire strip with a color
    void Show(void);                                                          // Push buffer to LEDs

private:
    PIO pixelPio;                            // PIO to use
    uint pixelOffset;                       // Offset for the PIO program
    uint pixelSm;                           // PIO state machine
    uint16_t actual_number_of_pixels;       // Number of LEDs
    uint8_t pixelBuffer[1024][3];           // RGB values for each LED

    uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b); // Convert RGB to 24-bit GRB format
    void putPixel(uint32_t pixel_grb);                 // Send GRB pixel to PIO
};

/******************************
 Motor Driver Class (I2C Controlled)
*******************************/
class motorDriver {
public:
    motorDriver();                                        // Constructor
    void dc(int dcNumber, int speed, int direction);      // Control DC motor (number, speed 0-255, direction 0/1)
    void servo(int servoNumber, int angle);               // Control servo motor (number 1-4, angle 0-180)
};
