#include <ESP8266WiFi.h>
#include <espnow.h>

// Pin assignments
const int buttonPin = D3;        // Signal button
const int greenLEDPin = D1;      // Green LED (active-high)
const int onboardLEDPin = D4;    // Onboard LED (active-low)

// MAC address of the receiver ESP32
uint8_t broadcastAddress[] = {0x80, 0x64, 0x6F, 0xC5, 0x3B, 0x6C};

// LED states
const int greenLEDOn = HIGH;   // Active-high: HIGH turns the LED on
const int greenLEDOff = LOW;   // Active-high: LOW turns the LED off
const int onboardLEDOn = LOW;   // Active-low: LOW turns the LED on
const int onboardLEDOff = HIGH; // Active-low: HIGH turns the LED off

// Timing settings
const unsigned long debounceDelay = 50;  // Debounce delay in milliseconds
const unsigned long ledBlinkDuration = 1500; // Duration to blink the LED on success in milliseconds
const unsigned long sleepAfterStart = 10000;  // 10 seconds before sleep after start
const unsigned long sleepAfterNoAction = 15000; // 15 seconds of inactivity before sleep
const unsigned long sleepAfterButtonPress = 10000; // 10 seconds after button press before sleep

// Variables to track time
unsigned long startTime;
unsigned long lastActionTime;
bool buttonPressed = false;

void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);
  Serial.println("Setup started...");

  // Initialize LED and button pins
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(greenLEDPin, OUTPUT);
  pinMode(onboardLEDPin, OUTPUT);
  
  // Ensure LEDs are on/off initially
  digitalWrite(greenLEDPin, greenLEDOff);  // Ensure the green LED is off at startup
  digitalWrite(onboardLEDPin, onboardLEDOn); // Turn on the onboard LED (blue LED is ON while awake)
  Serial.println("LEDs initialized. Blue LED is ON.");

  // Initialize ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  Serial.println("ESP-NOW initialized successfully.");

  // Register the peer
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
  esp_now_register_send_cb(OnDataSent);
  Serial.println("ESP-NOW peer registered.");

  // Initialize timers
  startTime = millis();  // Track when the board starts
  lastActionTime = startTime;  // Track the last action time

  Serial.println("Setup completed. Waiting for actions...");
}

void loop() {
  unsigned long currentTime = millis();
  
  // Read the button state
  int reading = digitalRead(buttonPin);
  
  // Debounce the button
  if (reading == LOW && !buttonPressed) {
    buttonPressed = true;
    onButtonPress();
    lastActionTime = millis();  // Reset the last action time
    Serial.println("Button press detected.");
  } else if (reading == HIGH && buttonPressed) {
    buttonPressed = false;
  }

  // Sleep after 10 seconds from startup if no action is done
  if (currentTime - startTime >= sleepAfterStart && lastActionTime == startTime) {
    Serial.println("No action after startup, going to sleep...");
    goToSleep();
  }

  // Sleep after 15 seconds of inactivity
  if (currentTime - lastActionTime >= sleepAfterNoAction) {
    Serial.println("No action for 15 seconds, going to sleep...");
    goToSleep();
  }
}

void onButtonPress() {
  Serial.println("Button pressed, sending data...");

  // Turn on the onboard LED to indicate transmission
  digitalWrite(onboardLEDPin, onboardLEDOn);
  Serial.println("Blue LED ON: Data transmission started.");
  
  // Send data to the receiver
  uint8_t data = 1;  // Example data
  esp_now_send(broadcastAddress, &data, sizeof(data));
  
  // Turn off the onboard LED after sending
  digitalWrite(onboardLEDPin, onboardLEDOff);
  Serial.println("Blue LED OFF: Data transmission completed.");
}

void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  if (sendStatus == 0) {
    Serial.println("Data sent successfully. Blinking green LED.");
    // Blink the green LED for a specified duration
    digitalWrite(greenLEDPin, greenLEDOn);   // Turn on the LED
    delay(ledBlinkDuration);                 // Wait for the specified duration
    digitalWrite(greenLEDPin, greenLEDOff);  // Turn off the LED
    Serial.println("Green LED OFF: Blinking completed.");

    // Sleep 10 seconds after button press
    Serial.println("Going to sleep 10 seconds after button press...");
    delay(sleepAfterButtonPress);
    goToSleep();
  } else {
    Serial.println("Data send failed.");
    // You can add an error handling routine here if needed
  }
}

void goToSleep() {
  Serial.println("Entering deep sleep...");
  digitalWrite(onboardLEDPin, onboardLEDOff); // Turn off the onboard LED before sleeping
  ESP.deepSleep(0);  // Enter deep sleep indefinitely until reset
}
