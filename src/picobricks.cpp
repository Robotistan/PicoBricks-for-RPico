#include "picobricks.h"
#include "font.h"

/*****************************
SSD1306 LIBRARY
******************************/
// Constructor to initialize the SSD1306 display
SSD1306::SSD1306(uint8_t address, uint8_t width, uint8_t height)
    : _address(address), _width(width), _height(height), _cursorX(0), _cursorY(0) {
    memset(_buffer, 0, sizeof(_buffer));  // Initialize the buffer to store the pixel data
}

// Function to initialize the SSD1306 display with necessary commands
void SSD1306::init() {
    sendCommand(0xAE); // Display OFF
    sendCommand(0xD5); // Set Display Clock
    sendCommand(0x80); // Suggested value
    sendCommand(0xA8); // Set Multiplex Ratio
    sendCommand(_height - 1); // Set height
    sendCommand(0xD3); // Set Display Offset
    sendCommand(0x00); // No offset
    sendCommand(0x40); // Set Display Start Line
    sendCommand(0x8D); // Charge Pump
    sendCommand(0x14); // Enable Charge Pump
    sendCommand(0x20); // Memory Addressing Mode
    sendCommand(0x00); // Horizontal Addressing
    sendCommand(0xA1); // Set Segment Re-map
    sendCommand(0xC8); // Set COM Output Scan Direction
    sendCommand(0xDA); // Set COM Pins Hardware
    sendCommand(0x12); // Select hardware configuration
    sendCommand(0x81); // Set Contrast
    sendCommand(0xCF); // Max contrast
    sendCommand(0xD9); // Set Pre-charge Period
    sendCommand(0xF1); // Pre-charge value
    sendCommand(0xDB); // Set VCOMH Deselect Level
    sendCommand(0x40); // Set VCOMH level
    sendCommand(0xA4); // Entire Display ON
    sendCommand(0xA6); // Set Normal/Inverse Display
    sendCommand(0xAF); // Display ON
}

// Clear the display and reset the cursor position
void SSD1306::clear() {
    memset(_buffer, 0, sizeof(_buffer)); // Reset the buffer
    _cursorX = 0; // Reset the cursor to the starting position
    _cursorY = 0; // Reset the cursor to the top
}

// Set the cursor position for drawing text or characters
void SSD1306::setCursor(uint8_t x, uint8_t y) {
    _cursorX = x; // Set the X position
    _cursorY = y; // Set the Y position
}

// Function to print a string on the display starting from the cursor position
void SSD1306::print(const char* text) {
    while (*text) {
        drawChar(_cursorX, _cursorY, *text++); // Draw each character
        _cursorX += 6; // Move the cursor to the right (space for the next character)
        if (_cursorX + 6 >= _width) { // If the cursor reaches the end of the screen
            _cursorX = 0; // Reset X to 0 (new line)
            _cursorY += 8;  // Move the cursor down (new line)
        }
    }
}

// Display the buffer content on the screen
void SSD1306::show() {
    for (uint8_t i = 0; i < 8; i++) {
        sendCommand(0xB0 + i); // Set the page start address
        sendCommand(0x00); // Low column start address
        sendCommand(0x10); // High column start address
        for (uint8_t j = 0; j < 128; j++) {
            sendData(_buffer[i * 128 + j]);
        }
    }
}

// Send a command to the OLED
void SSD1306::sendCommand(uint8_t command) {
    Wire.beginTransmission(_address); // Start communication with the display
    Wire.write(0x00); // Command mode
    Wire.write(command); // Send the command
    Wire.endTransmission(); // End communication
}

// Send data to the OLED (for pixels)
void SSD1306::sendData(uint8_t data) {
    Wire.beginTransmission(_address); // Start communication with the display
    Wire.write(0x40); // Data mode
    Wire.write(data); // Send the data
    Wire.endTransmission(); // End communication
}

// Draw a character at the given position (x, y)
void SSD1306::drawChar(int16_t x, int16_t y, unsigned char c) {
    if (c >= '0' && c <= '9') { // If the character is a number
        c = c - '0'; 
    } 
    else if (c >= 'A' && c <= 'Z') {  // If the character is an uppercase letter
        c = c - 'A' + 10; 
    } 
    else if (c >= 'a' && c <= 'z') {  // If the character is a lowercase letter
        c = c - 'a' + 36; 
    }
    else if (c >= '!' && c <= '/') {  // Special characters
        c = c - '!' + 62; 
    }
    else if (c >= ':' && c <= '@') {  // Special characters 2
        c = c - ':' + 77; 
    }
    else {
        return; // If the character is not supported, do nothing
    }

    for (int i = 0; i < 5; i++) {
        uint8_t line = font[c][i];  // Get the pixel data for the character
        for (int j = 0; j < 8; j++) {
            if (line & (1 << j)) { // If the pixel is set (bit is 1)
                drawPixel(x + i, y + j, true);  // Draw a filled pixel
            } else {
                drawPixel(x + i, y + j, false);  // Draw an empty pixel
            }
        }
    }
}

// Draw a pixel at the given (x, y) position with the specified color (true for ON, false for OFF)
void SSD1306::drawPixel(uint8_t x, uint8_t y, bool color) {
    if (x >= _width || y >= _height) return; // If the pixel is out of bounds, do nothing
    if (color) {
        _buffer[x + (y / 8) * _width] |= (1 << (y % 8)); // Set the bit for ON
    } else {
        _buffer[x + (y / 8) * _width] &= ~(1 << (y % 8)); // Clear the bit for OFF
    }
}

void SSD1306::drawBitmap(int16_t x, int16_t y, const uint8_t *bitmap, uint8_t w, uint8_t h) {
  uint16_t byteWidth = (w + 7) / 8;
  for (uint8_t j = 0; j < h; j++) {
    for (uint8_t i = 0; i < w; i++) {
      uint8_t byte = bitmap[j * byteWidth + i / 8];
      if (byte & (128 >> (i & 7))) {
        drawPixel(x + i, y + j, true);
      }
    }
  }
}

/*****************************
ServoSimple LIBRARY
******************************/
ServoSimple::ServoSimple(uint gpio_pin)
    : pin(gpio_pin), slice_num(pwm_gpio_to_slice_num(gpio_pin)) {
    // Save the GPIO and corresponding PWM slice number
}

void ServoSimple::begin() {
    gpio_set_function(pin, GPIO_FUNC_PWM);  // Set GPIO to PWM mode
    restartPWM();                           // Init PWM timing
}

void ServoSimple::restartPWM() {
    pwm_set_enabled(slice_num, false);           // Disable first (clean reset)
    pwm_set_wrap(slice_num, 20000);              // Set period (20ms → 50Hz)
    pwm_set_clkdiv(slice_num, 125.0f);           // Set clock divider for 1us resolution
    pwm_set_enabled(slice_num, true);            // Re-enable
}

void ServoSimple::setAngle(int angle) {
    angle = constrain(angle, 0, 180);                            // Clamp angle
    gpio_set_function(pin, GPIO_FUNC_PWM);                      // Force GPIO back to PWM mode
    restartPWM();                                               // Restart PWM settings
    uint16_t pulse_width_us = map(angle, 0, 180, 500, 2500);     // Convert angle to pulse width
    pwm_set_gpio_level(pin, pulse_width_us);                     // Write pulse width to GPIO
}

/*****************************
SHTC3 LIBRARY
******************************/
SHTC3::SHTC3(uint8_t address) {
    _address = address;
}

/**
 * Initializes the SHTC3 sensor by starting the I2C communication
 * and waking up the sensor to start measurements.
 */
void SHTC3::begin() {
    Wire.begin(); // Start the I2C bus
    wakeUp();     // Wake up the sensor to begin measuring
}

/**
 * Sends a command to the SHTC3 sensor.
 * @param high The high byte of the command.
 * @param low The low byte of the command.
 */
void SHTC3::sendCommand(uint8_t high, uint8_t low) {
    Wire.beginTransmission(_address); // Begin communication with the sensor
    Wire.write(high); // Send the high byte of the command
    Wire.write(low); // Send the low byte of the command
    Wire.endTransmission(); // End the communication
}

/**
 * Reads data from the SHTC3 sensor.
 * @param data Pointer to the buffer where the data will be stored.
 * @param len The number of bytes to read.
 */
void SHTC3::readData(uint8_t *data, uint8_t len) {
    Wire.requestFrom(_address, len); // Request 'len' bytes of data from the sensor
    uint8_t index = 0;
    while (Wire.available() && index < len) {
        data[index++] = Wire.read(); // Store the data in the buffer
    }
}

/**
 * Wakes up the SHTC3 sensor to begin measurement.
 * Sends the appropriate wake-up command and waits for the sensor to be ready.
 */
void SHTC3::wakeUp() {
    sendCommand(0x35, 0x17); // Wake up command (start measurement)
    delay(500);               // Delay to allow sensor to wake up
}

/**
 * Puts the SHTC3 sensor to sleep to conserve power.
 * Sends the sleep command to turn off the sensor's measurement functions.
 */
void SHTC3::sleep() {
    sendCommand(0xB0, 0x64); // Sleep command
}

/**
 * Reads the temperature value from the SHTC3 sensor.
 * The temperature is in Celsius and is calculated based on the sensor's raw data.
 * @return The temperature in Celsius.
 */
float SHTC3::readTemperature() {
    uint8_t data[6];
    sendCommand(0x78, 0x66); // Start measurement for temperature
    delay(100);               // Wait for the measurement
    readData(data, 6);        // Read the 6 bytes of data (2 for temperature, 2 for humidity)
    
    uint16_t rawTemp = (data[0] << 8) | data[1]; // Combine the 2 bytes
    rawTemp = rawTemp & 0xFFFC;  // Mask the last two bits (for the CRC check)
    return (((4375 * rawTemp) >> 14) - 4500) / 100.0;  // Convert raw data to temperature in Celsius based on sensor's formula
}

/**
 * Reads the humidity value from the SHTC3 sensor.
 * The humidity is in percentage and is calculated based on the sensor's raw data.
 * @return The humidity in percentage.
 */
float SHTC3::readHumidity() {
    uint8_t data[6];
    sendCommand(0x78, 0x66); // Start measurement for humidity
    delay(100);               // Wait for the measurement
    readData(data, 6);        // Read the 6 bytes of data (2 for temperature, 2 for humidity)

    uint16_t rawHumidity = (data[3] << 8) | data[4]; // Combine the 2 bytes for humidity
    rawHumidity = rawHumidity & 0xFFFC; // Mask the last two bits (for the CRC check)
    return ((625 * rawHumidity) >> 12) / 100.0; // Convert raw humidity data to percentage based on sensor's formula
}

/*****************************
DHT11 LIBRARY
******************************/
#define MIN_INTERVAL 4000                 // Minimum interval between reads (in milliseconds)
#define TIMEOUT UINT32_MAX                // Timeout value for signal pulses
float last_temp;                          // Store the last valid temperature reading
float last_hum;                           // Store the last valid humidity reading

// Constructor: initializes pin and timing cycles
DHT11::DHT11(uint8_t pin) {
  _pin = pin;                             // Set the GPIO pin for DHT11 data line
  _maxcycles = microsecondsToClockCycles(1000);  // Calculate the maximum cycles for 1ms
}

// Initializes the sensor
void DHT11::begin(uint8_t usec) {
  pinMode(_pin, INPUT_PULLUP);            // Use pull-up on input pin
  _lastreadtime = millis() - MIN_INTERVAL; // Allow immediate first read
  pullTime = usec;                        // Set pulling time (in microseconds)
}

// Read temperature from the sensor
float DHT11::readTemperature() {
  if (!read() && last_temp == 0) return 0;  // Return 0 if reading fails and no last value

  float t = data[2];                      // Integer part of temperature
  if (data[3] & 0x80) t = -1 - t;         // Check if temperature is negative
  t += (data[3] & 0x0f) * 0.1;            // Add decimal part

  if (t == 0)
    return last_temp;                    // Return previous value if new read is invalid
  else {
    last_temp = t;
    return t;
  }
}

// Read humidity from the sensor
float DHT11::readHumidity() {
  if (!read() && last_temp == 0) return 0;  // Return 0 if reading fails and no last value

  float h = data[0] + data[1] * 0.1;        // Combine integer and decimal parts

  if (h == 0)
    return last_hum;                       // Return previous value if new read is invalid
  else {
    last_hum = h;
    return h;
  }
}

// Reads raw data from DHT11 and validates checksum
bool DHT11::read() {
  uint32_t currenttime = millis();
  if ((currenttime - _lastreadtime) < MIN_INTERVAL) return _lastresult;  // Enforce read interval
  _lastreadtime = currenttime;

  // Reset data buffer
  data[0] = data[1] = data[2] = data[3] = data[4] = 0;

  // Send start signal
  pinMode(_pin, INPUT_PULLUP); delay(1);   // Prepare line
  pinMode(_pin, OUTPUT); digitalWrite(_pin, LOW); delay(20); // Hold low for 20ms
  pinMode(_pin, INPUT_PULLUP); delayMicroseconds(pullTime);  // Pull high before read

  // Wait for sensor response
  if (expectPulse(LOW) == TIMEOUT) return false;  // Wait for LOW response
  if (expectPulse(HIGH) == TIMEOUT) return false; // Wait for HIGH response

  // Read 40 bits (80 pulse lengths)
  uint32_t cycles[80];
  for (int i = 0; i < 80; i += 2) {
    cycles[i] = expectPulse(LOW);   // LOW duration
    cycles[i + 1] = expectPulse(HIGH); // HIGH duration
  }

  // Parse 40 bits into 5 bytes
  for (int i = 0; i < 40; ++i) {
    uint32_t lowCycles = cycles[2 * i];
    uint32_t highCycles = cycles[2 * i + 1];
    if ((lowCycles == TIMEOUT) || (highCycles == TIMEOUT)) return false;

    data[i / 8] <<= 1;                     // Shift to the left
    if (highCycles > lowCycles)           // If HIGH duration is longer, bit is 1
      data[i / 8] |= 1;
  }

  // Verify checksum
  _lastresult = (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF));
  return _lastresult;
}

// Waits for a pin to go to a specific logic level and returns the cycle count
uint32_t DHT11::expectPulse(bool level) {
  uint32_t count = 0;
  while (digitalRead(_pin) == level) {   // Wait while the pin stays at the target level
    if (count++ >= _maxcycles) return TIMEOUT; // If too long, return timeout
  }
  return count;
}

/*****************************
NEOPIXEL LIBRARY
******************************/
// Constructor: Initializes NeoPixel with given pin number and number of LEDs
NeoPixel::NeoPixel(byte pinNumber, uint16_t numberOfPixels) {
    this->pixelSm = 0;                         // Use state machine 0 of the PIO
    this->pixelPio = pio0;                     // Use PIO0 hardware
    this->Init(pinNumber, numberOfPixels);     // Call initialization function
}

// Initializes the PIO program for controlling WS2812 LEDs (NeoPixel)
void NeoPixel::Init(byte pinNumber, uint16_t numberOfPixels) {
    uint offset = pio_add_program(this->pixelPio, &ws2812_program); // Load WS2812 PIO program
    ws2812_program_init(this->pixelPio, this->pixelSm, offset, pinNumber, 800000, false); // Initialize with 800kHz

    this->actual_number_of_pixels = numberOfPixels; // Store number of LEDs

    // Initialize all pixel colors to off (0,0,0)
    for (uint16_t i = 0; i < this->actual_number_of_pixels; i++) {
        this->pixelBuffer[i][0] = 0; // Red
        this->pixelBuffer[i][1] = 0; // Green
        this->pixelBuffer[i][2] = 0; // Blue
    }

    this->Show();  // Update LEDs to reflect buffer
    delay(1);      // Short delay to stabilize
}

// Sets the RGB color of a specific LED and updates the strip
void NeoPixel::setPixelColor(uint16_t pixelNumber, uint8_t r, uint8_t g, uint8_t b) {
    this->pixelBuffer[pixelNumber][0] = r;  // Set red value
    this->pixelBuffer[pixelNumber][1] = g;  // Set green value
    this->pixelBuffer[pixelNumber][2] = b;  // Set blue value
    this->Show();                           // Send updated colors to LED strip
}

// Fills all LEDs with the same RGB color and updates the strip
void NeoPixel::Fill(uint8_t r, uint8_t g, uint8_t b) {
    for (uint16_t i = 0; i < this->actual_number_of_pixels; i++) {
        this->pixelBuffer[i][0] = r;  // Set red
        this->pixelBuffer[i][1] = g;  // Set green
        this->pixelBuffer[i][2] = b;  // Set blue
    }
    this->Show(); // Update all pixels at once
}

// Sends the RGB data of each pixel to the NeoPixel strip
void NeoPixel::Show(void) {
    for (uint16_t i = 0; i < this->actual_number_of_pixels; i++) {
        this->putPixel(                  // Send encoded pixel color
            urgb_u32(                    // Convert RGB values to 24-bit GRB format
                pixelBuffer[i][0],       // Red
                pixelBuffer[i][1],       // Green
                pixelBuffer[i][2]        // Blue
            )
        );
    }
}

// Converts individual RGB values into a single 24-bit GRB value
uint32_t NeoPixel::urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return
        ((uint32_t)(r) << 8) |          // Red in bits 15-8
        ((uint32_t)(g) << 16) |         // Green in bits 23-16
        (uint32_t)(b);                  // Blue in bits 7-0
}

// Sends a single pixel's GRB data to the PIO state machine
void NeoPixel::putPixel(uint32_t pixel_grb) {
    pio_sm_put_blocking(this->pixelPio, this->pixelSm,
                        pixel_grb << 8u); // Shift data to match 24-bit WS2812 format
}

/*****************************
MOTOR DRIVER LIBRARY
******************************/
motorDriver::motorDriver(){}

// Function to control a DC motor (1 or 2) with a given speed (0-255) and direction (0 or 1)
// dcNumber: 1 or 2 (selects motor)
// speed: 0 to 255 (controls the motor speed)
// direction: 0 for one direction, 1 for the opposite direction
void motorDriver::dc(int dcNumber, int speed, int direction) {  
  Wire.begin(); // Start the I2C communication
  Wire.beginTransmission(0x22); 
  Wire.write(0x26);      
  Wire.write(dcNumber);   
  Wire.write(speed);  
  Wire.write(direction);
  int cs = dcNumber ^ speed ^ direction;
  Wire.write(cs);
  Wire.endTransmission(); 
}

// Function to control a servo motor (1 to 4) with a given angle (0-180)
// servoNumber: 1, 2, 3, or 4 (selects servo motor)
// angle: 0 to 180 (controls the servo angle)
void motorDriver::servo(int servoNumber, int angle) { 
  Wire.begin(); // Start the I2C communication
  Wire.beginTransmission(0x22); 
  Wire.write(0x26);        
  Wire.write(servoNumber + 2);
  Wire.write(0x00);
  Wire.write(angle);
  int cs = ((servoNumber + 2) ^ angle);
  Wire.write(cs);
  Wire.endTransmission(); 
}
