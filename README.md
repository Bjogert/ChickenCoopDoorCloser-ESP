# CoopDoorCloser-ESP

This project automates the closing of a chicken coop door using ESP32 and ESP8266 microcontrollers, with communication handled via the ESP-NOW protocol.

## Overview

The motorized system pulls a string attached to a weight on the door, effectively closing it. The door remains closed using a magnet. After closing, the motor unwinds the string, preventing it from breaking the door if the motor inadvertently winds up again. This setup ensures the door stays closed at night while avoiding potential damage if the door isn't manually opened the following day.

## Features

- **Remote Control**: The door can be closed remotely using an ESP8266 as a transmitter.
- **Automatic Motor Control**: The ESP32 receiver controls the motor to pull and unwind the string, securely closing the door and then releasing tension.
- **Safe Operation**: The system is designed to prevent damage to the door by unwinding the string after closing.

## Hardware Used

- **ESP32**: Receives the command and controls the motor.
- **ESP8266**: Sends the command to the ESP32 remotely.
- **Stepper Motor**: Pulls the string to close the door.
- **Magnet**: Holds the door closed once it's shut.
- **Miscellaneous**: Wires, string, and weight.

## How It Works

1. The system receives a signal via ESP-NOW to close the door.
2. The motor pulls the string, closing the door securely.
3. After the door is closed, the motor unwinds the string to avoid exerting continuous force on the door.

## Getting Started

Clone the repository and follow the instructions in the code to set up the hardware and software. Ensure you have the necessary components and that they are connected as described in the overview.
