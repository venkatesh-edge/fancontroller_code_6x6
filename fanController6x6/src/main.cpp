#include <Arduino.h>
#include <Wire.h>

// Define the I2C addresses of the two sensors
#define SENSOR1_ADDR 0x18
#define SENSOR2_ADDR 0x18 // Same address for both sensors, but on different buses

const int fanA1 = 1;
const int tachoA1 = 2;

const int fanA2 = 42;
const int tachoA2 = 41;

const int fanA3 = 40;
const int tachoA3 = 49;

const int fanA4 = 38;
const int tachoA4 = 37;

const int fanA5 = 36;
const int tachoA5 = 35;

const int fanA6 = 45;
const int tachoA6 = 48;

const int fanB1 = 47;
const int tachoB1 = 21;

const int fanB2 = 14;
const int tachoB2 = 13;

const int fanB3 = 12;
const int tachoB3 = 11;

const int fanB4 = 10;
const int tachoB4 = 9;

const int fanB5 = 46;
const int tachoB5 = 3;

const int fanB6 = 8;
const int tachoB6 = 16;



const int pwmChannel0 = 0;      // Define a PWM channel (0-15 available on ESP32-S3)
const int pwmChannel1 = 1;      // Define a PWM channel (0-15 available on ESP32-S3)
const int pwmFrequency = 25000; // 25kHz for fan PWM control
const  int pwmResolution = 8;    // 8-bit resolution (0-255)
int fanSpeedA = 128;            // Start with 50% duty cycle
int fanSpeedB = 128;
const int motherboardPWMpin = 15;

// Motherboard PWM signal threshold (microseconds) to detect active signal
unsigned long motherboardPwmThreshold = 70; // Adjust as needed based on motherboard PWM signal
unsigned long Mpwm; 

// Timing variables
unsigned long previousMillis = 0; // Stores the last time group switched
unsigned long interval;           // Interval in milliseconds
int hours, minutes, seconds;

// Variables to store RPM
volatile int rpmA1 = 0;
volatile int rpmA2 = 0;
volatile int rpmA3 = 0;
volatile int rpmA4 = 0;
volatile int rpmA5 = 0;
volatile int rpmA6 = 0;
volatile int rpmB1 = 0;
volatile int rpmB2 = 0;
volatile int rpmB3 = 0;
volatile int rpmB4 = 0;
volatile int rpmB5 = 0;
volatile int rpmB6 = 0;

bool groupAActive = true;      // Flag to determine which group is active
unsigned long lastRpmTime = 0; // Stores last time RPM was calculated

unsigned long calculatedRPM1 = 0;
unsigned long calculatedRPM2 = 0;

String mode = "switch";

// Create I2C buses for the two sensors
TwoWire I2C_1 = TwoWire(0); // First I2C bus (pins 4, 5)
TwoWire I2C_2 = TwoWire(1); // Second I2C bus (pins 6, 7)

// Function to convert milliseconds to hours, minutes, and seconds
void millisToHMS(unsigned long millis, int &hours, int &minutes, int &seconds)
{
    seconds = millis / 1000; // Convert milliseconds to seconds
    minutes = seconds / 60;  // Convert seconds to minutes
    hours = minutes / 60;    // Convert minutes to hours
    seconds %= 60;           // Get remaining seconds
    minutes %= 60;           // Get remaining minutes
}

// Function to set the interval and save it to EEPROM
void setInterval(unsigned long newInterval)
{
    interval = newInterval;
    // EEPROM.put(0, interval);  // Store the new interval in EEPROM
}

// Function to set fan speeds based on temperature
void switchMode(float tempA, float tempB)
{
    int fanSpeedA = 0;
    int fanSpeedB = 0;
    if (groupAActive)
    {
        if (Mpwm > tempA)
        {
            fanSpeedA = map(Mpwm, 0, 100, 0, 255);
        }
        else
        {
            fanSpeedA = map(tempA, 20, 40, 100, 255);
            fanSpeedA = constrain(fanSpeedA, 100, 255);
        }
        calculatedRPM1 = map(fanSpeedA, 0, 255, 0, 3000);
        calculatedRPM2 = 0;
        // Control fans for group A
        ledcWrite(pwmChannel0, fanSpeedA);
        ledcWrite(pwmChannel0, fanSpeedA);

        // Turn off fans for group B
        ledcWrite(pwmChannel1, 0);
        ledcWrite(pwmChannel1, 0);
    }
    else
    {
        if (Mpwm > tempB)
        {
            fanSpeedB = map(Mpwm, 0, 100, 0, 255);
        }
        else
        {
            fanSpeedB = map(tempB, 20, 40, 100, 255);
            fanSpeedB = constrain(fanSpeedB, 100, 255);
        }
        calculatedRPM2 = map(fanSpeedB, 0, 255, 0, 3000);
        calculatedRPM1 = 0;

        // Control fans for group B
        ledcWrite(pwmChannel1, fanSpeedB);
        ledcWrite(pwmChannel1, fanSpeedB);

        // Turn off fans for group A
        ledcWrite(pwmChannel0, 0);
        ledcWrite(pwmChannel0, 0);
    }
}

void continuousMode(float tempA, float tempB)
{
    int fanSpeedA;
    int fanSpeedB;
    if (Mpwm > tempA && Mpwm > tempB)
    {
        fanSpeedA = fanSpeedB = map(Mpwm, 0, 100, 0, 255);
    }
    else
    {
        fanSpeedA = map(tempA, 20, 40, 100, 255);
        fanSpeedA = constrain(fanSpeedA, 100, 255);

        fanSpeedB = map(tempB, 20, 40, 100, 255);
        fanSpeedB = constrain(fanSpeedB, 100, 255);
    }
    calculatedRPM1 = map(fanSpeedA, 0, 255, 0, 3000);
    calculatedRPM2 = map(fanSpeedB, 0, 255, 0, 3000);

    // Control fans for group A
    ledcWrite(pwmChannel0, fanSpeedA);
    ledcWrite(pwmChannel0, fanSpeedA);

    // Control fans for group B
    ledcWrite(pwmChannel1, fanSpeedB);
    ledcWrite(pwmChannel1, fanSpeedB);
}

// Interrupt service routines for tachometers
void rpmA1ISR()
{
    rpmA1 = rpmA1 + 1;
}
void rpmA2ISR()
{
    rpmA2 = rpmA2 + 1;
}
void rpmA3ISR()
{
    rpmA3 = rpmA3 + 1;
}
void rpmA4ISR()
{
    rpmA4 = rpmA4 + 1;
}
void rpmA5ISR()
{
    rpmA5 = rpmA5 + 1;
}
void rpmA6ISR()
{
    rpmA6 = rpmA6 + 1;
}


void rpmB1ISR()
{
    rpmB1 = rpmB1 + 1;
}
void rpmB2ISR()
{
    rpmB2 = rpmB2 + 1;
}
void rpmB3ISR()
{
    rpmB3 = rpmB3 + 1;
}
void rpmB4ISR()
{
    rpmB4 = rpmB4 + 1;
}
void rpmB5ISR()
{
    rpmB5 = rpmB5 + 1;
}
void rpmB6ISR()
{
    rpmB6 = rpmB6 + 1;
}

float readTemperature(TwoWire &i2cBus, uint8_t address)
{
    i2cBus.beginTransmission(address);
    i2cBus.write(0x05); // Point to the temperature register
    i2cBus.endTransmission();

    i2cBus.requestFrom(static_cast<uint8_t>(address), static_cast<uint8_t>(2));  // Read 2 bytes (MSB and LSB)


    if (i2cBus.available() == 2)
    {
        byte msb = i2cBus.read(); // Read most significant byte
        byte lsb = i2cBus.read(); // Read least significant byte

        int temp = ((msb & 0x1F) << 8) | lsb;
        return temp * 0.0625; // Convert to Celsius
    }
    return -1000; // Error value if not available
}

bool initializeSensor(TwoWire &i2cBus, uint8_t address)
{
    i2cBus.beginTransmission(address);
    return i2cBus.endTransmission() == 0; // Return true if no error
}

void setup()
{
    Serial.begin(115200);
    Serial1.begin(115200, SERIAL_8N1, 18, 17);
    I2C_1.begin(4, 5); // First I2C bus (pins 4, 5)
    I2C_2.begin(6, 7); // Second I2C bus (pins 6, 7)

    delay(1000); // Wait for serial monitor to start
    Serial.println("MCP9808 Temperature Sensors");

    // Initialize both sensors
    if (!initializeSensor(I2C_1, SENSOR1_ADDR))
    {
        Serial.println("Couldn't connect to Sensor 1");
        while (1)
            ;
    }

    if (!initializeSensor(I2C_2, SENSOR2_ADDR))
    {
        Serial.println("Couldn't connect to Sensor 2");
        while (1)
            ;
    }

    // Initialize fan pins as outputs
    pinMode(fanA1, OUTPUT);
    pinMode(fanA2, OUTPUT);
    pinMode(fanA3, OUTPUT);
    pinMode(fanA4, OUTPUT);
    pinMode(fanA5, OUTPUT);
    pinMode(fanA6, OUTPUT);

    pinMode(fanB1, OUTPUT);
    pinMode(fanB2, OUTPUT);
    pinMode(fanB3, OUTPUT);
    pinMode(fanB4, OUTPUT);
    pinMode(fanB5, OUTPUT);
    pinMode(fanB6, OUTPUT);

    // Initialize tachometer pins as inputs
    pinMode(tachoA1, INPUT);
    pinMode(tachoA2, INPUT);
    pinMode(tachoA3, INPUT);
    pinMode(tachoA4, INPUT);
    pinMode(tachoA5, INPUT);
    pinMode(tachoA6, INPUT);

    pinMode(tachoB1, INPUT);
    pinMode(tachoB2, INPUT);
    pinMode(tachoB3, INPUT);
    pinMode(tachoB4, INPUT);
    pinMode(tachoB5, INPUT);
    pinMode(tachoB6, INPUT);

    Serial.println("Sensors Initialized!");

    pinMode(motherboardPWMpin, INPUT);

    // PWM Setup with ESP32-S3 3.x API
    ledcSetup(pwmChannel0, pwmFrequency, pwmResolution); // Configure PWM channel
    ledcAttachPin(fanA1, pwmChannel0);                   // Attach PWM channel to GPIO pin
    ledcAttachPin(fanA2, pwmChannel0);                   // Attach PWM channel to GPIO pin
    ledcAttachPin(fanA3, pwmChannel0);                   
    ledcAttachPin(fanA4, pwmChannel0);                   
    ledcAttachPin(fanA5, pwmChannel0);                   
    ledcAttachPin(fanA6, pwmChannel0);                   

    ledcWrite(pwmChannel0, fanSpeedA);                   // Set initial duty cycle to 50%

    ledcSetup(pwmChannel1, pwmFrequency, pwmResolution); // Configure PWM channel
    ledcAttachPin(fanB1, pwmChannel1);                   // Attach PWM channel to GPIO pin
    ledcAttachPin(fanB2, pwmChannel1);                   // Attach PWM channel to GPIO pin
    ledcAttachPin(fanB3, pwmChannel1);
    ledcAttachPin(fanB4, pwmChannel1);
    ledcAttachPin(fanB5, pwmChannel1);
    ledcAttachPin(fanB6, pwmChannel1);
    ledcWrite(pwmChannel1, fanSpeedB);                   // Set initial duty cycle to 50%

    // Attach interrupts for tachometer readings
    attachInterrupt(digitalPinToInterrupt(tachoA1), rpmA1ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(tachoA2), rpmA2ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(tachoA3), rpmA3ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(tachoA4), rpmA4ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(tachoA5), rpmA5ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(tachoA6), rpmA6ISR, RISING);

    attachInterrupt(digitalPinToInterrupt(tachoB1), rpmB1ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(tachoB2), rpmB2ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(tachoB3), rpmB3ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(tachoB4), rpmB4ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(tachoB5), rpmB5ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(tachoB6), rpmB6ISR, RISING);
    // Read the interval from EEPROM on startup
    interval = 20000;
    // EEPROM.get(0, interval);  // Default to 50 seconds if invalid
    setInterval(interval); // Save the default interval to EEPROM
}

void loop()
{
    // Read and print temperature for both sensors

    delay(1000); // Wait before reading again

    // Read motherboard PWM signal
    int highTime = pulseIn(motherboardPWMpin, HIGH);
    int lowTime = pulseIn(motherboardPWMpin, LOW);
    int cycleTime = highTime + lowTime;

    if (cycleTime > 0)
    {
        Mpwm = (100 * highTime) / cycleTime; // Calculate the PWM percentage
    }
    else
    {
        Mpwm = 0; // Set to 0 if there's no valid pulse
    }

    // Check if motherboard PWM signal is active

    unsigned long currentMillis = millis();

    // Read from serial to set new interval

    if (Serial.available())
    {
        String input = Serial.readStringUntil('\n'); // Read input until a new line
        Serial.println(input);

        // Attempt to convert the input to an integer
        bool isNumber = true;
        for (unsigned int i = 0; i < input.length(); i++)
        {
            if (!isDigit(input[i]))
            {
                isNumber = false;
                break;
            }
        }

        if (isNumber)
        {
            // If the input is a number, convert it and process it as an interval
            unsigned long newInterval = input.toInt(); // Convert input to int
            if (newInterval >= 10000)
            {                             // Validate the new interval (1 sec - 60 sec)
                setInterval(newInterval); // Update and save the new interval
                Serial.print("Interval updated to: ");
                Serial.println(newInterval); // Print the updated interval
            }
            else
            {
                Serial.println("Invalid interval. Please enter a value above 10sec");
                Serial.println(newInterval);
            }
        }
        else
        {
            // If the input is not a number, check for specific commands like "switch" or "continuous"
            input.trim(); // Remove any trailing whitespace/newlines
            if (input == "switch")
            {
                Serial.println("Switch mode selected");
                mode = "switch";
                // Add logic to handle "switch" command
            }
            else if (input == "continuous")
            {
                Serial.println("Continuous mode selected");
                mode = "continuous";
                // Add logic to handle "continuous" command
            }
            else
            {
                Serial.print("Unknown command: ");
            }
        }
    }

    // Get temperatures
    float tempA = readTemperature(I2C_1, SENSOR1_ADDR);
    float tempB = readTemperature(I2C_2, SENSOR2_ADDR);

    if (mode == "switch")
    {
        // Check if it's time to switch groups
        if (currentMillis - previousMillis >= interval)
        {
            previousMillis = currentMillis; // Save the last switch time
            groupAActive = !groupAActive;   // Toggle active group
            Serial.print("Switched to group: ");
            Serial.println(groupAActive ? "A" : "B"); // Debug output
            Serial1.print("Switched to group: ");
            Serial1.println(groupAActive ? "A" : "B");
        }
        switchMode(tempA, tempB);
    }
    else if (mode == "continuous")
    {
        continuousMode(tempA, tempB);
    }

    // RPM outputs
    if (currentMillis - lastRpmTime >= 5000)
    { // Update every 5 seconds
        lastRpmTime = currentMillis;
        Serial.println("===========================");
        Serial.print("Temperature A: ");
        Serial.print(tempA);
        Serial.print(" °C, Temperature B: ");
        Serial.print(tempB);
        Serial.println(" °C");
        Serial.print("RPM A1: ");
        Serial.println(calculatedRPM1);
        Serial.print("RPM A2: ");
        Serial.println(calculatedRPM1);
        Serial.print("RPM B1: ");
        Serial.println(calculatedRPM2);
        Serial.print("RPM B2: ");
        Serial.println(calculatedRPM2);
        Serial.print("Seconds: ");
        Serial.println(interval);
        Serial.println(mode);
        Serial.println(Mpwm);

        Serial.println("===========================");

        Serial1.println("===========================");
        Serial1.println("this is serial 1");
        Serial1.print("Temperature A: ");
        Serial1.print(tempA);
        Serial1.print(" °C, Temperature B: ");
        Serial1.print(tempB);
        Serial1.println(" °C");
        Serial1.print("RPM A1: ");
        Serial1.println(calculatedRPM1);
        Serial1.print("RPM A2: ");
        Serial1.println(calculatedRPM1);
        Serial1.print("RPM B1: ");
        Serial1.println(calculatedRPM2);
        Serial1.print("RPM B2: ");
        Serial1.println(calculatedRPM2);
        Serial1.print("Seconds: ");
        Serial1.println(interval);
        Serial1.println(mode);
        Serial1.println("===========================");
    }
}
