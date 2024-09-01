#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Bounce2.h>
#include <Stepper.h>
#include "esp_wifi.h"

// Function declarations
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int len);
void runMotorSequence();
void moveMotorForward();
void moveMotorBackward();

// Motor settings
const int stepsPerRevolution = 200;  // Steps per revolution for your motor
const int desiredSteps = 24000;      // Desired number of steps to move
const int stepChunkSize = 100;       // Number of steps per chunk to avoid WDT triggering

// Pin assignments
const int dirPin = 32;
const int stepPin = 25;
const int enPin = 27;
const int toggleSwitch = 26;  // Control button (manual)
const int simulatedTogglePin = 33;  // New pin to simulate button press

// Light sensor pins
const int lightSensorDigitalPin = 35; // DO pin of the light sensor module

// LED pin
#define ONBOARD_LED_PIN 2  // Onboard LED on the ESP32

// Debounce settings
const unsigned long debounceInterval = 50;  // Debounce time for the button

// Light and darkness detection settings
int lightRequiredTime = 10000;  // Light must persist for 10 seconds (10000 milliseconds) before starting darkness detection
int darknessThreshold = 60000;  // Darkness must persist for 10 seconds (one minute) to trigger motor sequence
unsigned long lightStartTime = 0;
unsigned long darknessStartTime = 0;
bool isDark = false;
bool isLightConfirmed = false;

Bounce debouncer = Bounce();
Stepper stepper(stepsPerRevolution, stepPin, dirPin);  // Initialize stepper motor

bool motorSequenceTriggered = false;  // Flag to trigger the motor sequence

void setup() {
    Serial.begin(115200);
    Serial.println("Receiver Setup Started");

    // Initialize LEDs and motor pins
    pinMode(ONBOARD_LED_PIN, OUTPUT);
    pinMode(enPin, OUTPUT);
    pinMode(toggleSwitch, INPUT_PULLUP);  // Keep this as INPUT_PULLUP for manual button
    pinMode(simulatedTogglePin, OUTPUT);  // Set the new pin as OUTPUT
    digitalWrite(simulatedTogglePin, LOW); // Ensure the simulated pin is LOW initially

    debouncer.attach(toggleSwitch);
    debouncer.interval(debounceInterval);  // Debounce time for the button

    pinMode(lightSensorDigitalPin, INPUT);  // Set the digital pin of the light sensor as input

    digitalWrite(enPin, HIGH);  // Disable the stepper driver initially
    digitalWrite(ONBOARD_LED_PIN, LOW);  // Turn off onboard LED initially

    stepper.setSpeed(200);  // Increase speed for testing (steps per second)

    // Initialize ESP-NOW
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != 0) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Register the receive callback
    esp_now_register_recv_cb(OnDataRecv);

    Serial.println("Receiver Setup Completed, Waiting for light to start monitoring darkness...");
}

void loop() {
    debouncer.update();

    int lightLevelDigital = digitalRead(lightSensorDigitalPin);
    unsigned long currentMillis = millis();

    // Check for required light period at startup
    if (!isLightConfirmed) {
        if (lightLevelDigital == LOW) {  // LOW means bright based on the previous test
            if (lightStartTime == 0) {
                lightStartTime = currentMillis;
                Serial.println("Brightness detected. Starting light confirmation timer...");
            } else if (currentMillis - lightStartTime >= lightRequiredTime) {
                isLightConfirmed = true;
                Serial.println("Light confirmed. Beginning darkness monitoring.");
            } else {
                Serial.print("Brightness ongoing for ");
                Serial.print((currentMillis - lightStartTime) / 1000);
                Serial.println(" seconds.");
            }
        } else {
            // Reset the timer if it gets dark again
            if (lightStartTime != 0) {
                Serial.println("Darkness detected during light confirmation. Resetting timer.");
                lightStartTime = 0;
            } else {
                Serial.println("It's dark. Waiting for light to begin monitoring.");
            }
        }
    } else {
        // Once light is confirmed, start darkness detection
        if (lightLevelDigital == HIGH) {  // HIGH means dark based on the previous test
            if (!isDark) {
                // If it just became dark, start the timer
                darknessStartTime = currentMillis;
                isDark = true;
                Serial.println("Darkness detected. Starting timer...");
            } else {
                // If it has been dark for long enough, trigger the motor sequence
                if (currentMillis - darknessStartTime >= darknessThreshold) {
                    Serial.println("It's been dark for 10 seconds. Waiting for 30 minutes before triggering motor sequence.");

                    // Delay for 30 minutes (1800000 milliseconds)
                    delay(1800000);

                    Serial.println("30 minutes delay completed. Triggering motor sequence.");
                    motorSequenceTriggered = true;
                } else {
                    Serial.print("Darkness ongoing for ");
                    Serial.print((currentMillis - darknessStartTime) / 1000);
                    Serial.println(" seconds.");
                }
            }
        } else {
            // If it gets bright again, reset the darkness check
            if (isDark) {
                Serial.println("Light detected. Resetting darkness timer.");
                isDark = false;
            } else {
                Serial.println("It's bright. No action required.");
            }
        }
    }

    // Handle button press manually for testing or manual override
    if (debouncer.fell()) {
        Serial.println("Manual button press detected");
        motorSequenceTriggered = true;  // Trigger motor sequence when the button is pressed
    }

    // Check if the motor sequence was triggered
    if (motorSequenceTriggered) {
        motorSequenceTriggered = false;  // Reset the flag

        // Stop and deinitialize Wi-Fi
        Serial.println("Stopping Wi-Fi...");
        esp_wifi_stop();
        delay(500);  // Added delay to ensure Wi-Fi has fully stopped
        esp_wifi_deinit();
        delay(100);  // Short delay to ensure Wi-Fi is fully stopped

        // Run the motor sequence
        runMotorSequence();

        // Reboot the board
        Serial.println("Rebooting the board...");
        ESP.restart();
    }

    delay(1000); // Update every second
}

// Function to run the motor forward and then backward
void runMotorSequence() {
    Serial.println("Running motor sequence...");

    // Move the motor forward
    moveMotorForward();

    // Wait for 1 second before moving backward
    delay(1000);

    // Move the motor backward
    moveMotorBackward();

    // Disable the motor driver
    digitalWrite(enPin, HIGH);
    Serial.println("Motor movement complete. Motor disabled.");
}

// Function to move the motor forward
void moveMotorForward() {
    Serial.println("Moving forward");
    digitalWrite(enPin, LOW);  // Enable the motor driver
    delay(50);  // Short delay to ensure the driver is enabled

    // Move the motor in smaller chunks to prevent WDT triggering
    int stepsRemaining = desiredSteps;
    while (stepsRemaining > 0) {
        int stepsToMove = min(stepsRemaining, stepChunkSize);  // Step in chunks
        stepper.step(stepsToMove);
        stepsRemaining -= stepsToMove;
        yield();  // Reset the watchdog timer and allow background tasks
    }

    Serial.print("Expected steps forward: ");
    Serial.println(desiredSteps);
}

// Function to move the motor backward
void moveMotorBackward() {
    Serial.println("Moving backward");

    // Move the motor in smaller chunks to prevent WDT triggering
    int stepsRemaining = desiredSteps;
    while (stepsRemaining > 0) {
        int stepsToMove = min(stepsRemaining, stepChunkSize);  // Step in chunks
        stepper.step(-stepsToMove);
        stepsRemaining -= stepsToMove;
        yield();  // Reset the watchdog timer and allow background tasks
    }

    Serial.print("Expected steps backward: ");
    Serial.println(desiredSteps);
}

// ESP-NOW receive callback function for ESP32
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int len) {
    Serial.println("Data received via ESP-NOW");

    // Print the MAC address of the sender
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
            mac_addr[0], mac_addr[1], mac_addr[2],
            mac_addr[3], mac_addr[4], mac_addr[5]);
    Serial.print("Received from MAC: ");
    Serial.println(macStr);

    // Turn on the onboard LED to indicate a command was received
    digitalWrite(ONBOARD_LED_PIN, HIGH);

    // Set the flag to trigger the motor sequence after Wi-Fi is disabled
    if (len == 1 && *data == 1) {  // Assuming '1' is the command to simulate a button press
        Serial.println("Received command to simulate button press.");
        motorSequenceTriggered = true;
    } else {
        Serial.print("Unexpected data received: ");
        Serial.println(*data);
    }

    // Turn off the onboard LED after a short delay
    delay(1000);
    digitalWrite(ONBOARD_LED_PIN, LOW);
}
