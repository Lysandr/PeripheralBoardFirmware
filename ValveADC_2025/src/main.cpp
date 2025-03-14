// Teensy 4.0 UART Control Code
// Written by AI using OpenAI's GPT model
// This code manages valve control and analog multiplexer readings with CRC checking

#include <Arduino.h>
#include <CRC.h> // Replaces #include <CRC8.h>

#define NUM_VALVES 10
#define NUM_ANALOG_CHANNELS 10  // Analog MUX channels
#define COMMAND_LENGTH 11      // 10 valve states + 1 CRC byte
#define TIMEOUT_MS 30000       // 30 seconds timeout

// Pin Definitions
const int valvePins[NUM_VALVES] = {7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
const int muxSelectPins[4] = {3, 4, 5, 6};  // MUX selector pins
#define MUX_COMMON_PIN A8  // Common analog read pin for the multiplexer

// Valve State Array
bool valveStates[NUM_VALVES] = {false};

// Serial Communication
#define SERIAL_BAUD 115200
char commandBuffer[COMMAND_LENGTH];
CRC8 crc;

// Timer Interval for 100Hz
IntervalTimer statusTimer;
unsigned long lastCommandTime = 0;  // Track the last command received time

// Function Prototypes
void sendStatus();
bool validateCommand(const char* command);
void updateValves(const char* command);
void shutOffValves();
int readMuxChannel(int channel);

void setup() {
	// Start serial with PI, drive all valves low.
    Serial1.begin(SERIAL_BAUD);
    for (int i = 0; i < NUM_VALVES; i++) {
        pinMode(valvePins[i], OUTPUT);
        digitalWrite(valvePins[i], LOW);  // Initialize all valves as OFF
    }
    // Configure all mux pins as outputs, and drive them low for now.
    for (int i = 0; i < 4; i++) {
        pinMode(muxSelectPins[i], OUTPUT);
        digitalWrite(muxSelectPins[i], LOW);  // Initialize MUX select pins as LOW
    }

    // Configure an interrupt timer to 100Hz telemetry
    statusTimer.begin(sendStatus, 10000); // 100Hz (10ms interval)
    lastCommandTime = millis();  // Start the command timeout tracker
}

void loop() {
    // If we have a packet from RPi that makes sense, read it.
    if (Serial1.available() >= COMMAND_LENGTH) {
        Serial1.readBytes(commandBuffer, COMMAND_LENGTH);
        // If it's good, then update the valve command states.
        if (validateCommand(commandBuffer)) {
            updateValves(commandBuffer);
            lastCommandTime = millis();
        }
    }

    // If we haven't gotten a command in a bit, shut them all off.
    // TODO: future maybe dont shut off vent valves for lox tank.
    if (millis() - lastCommandTime >= TIMEOUT_MS) {
        shutOffValves();
    }
}

// 
bool validateCommand(const char* command) {
    crc.restart();
    for (int i = 0; i < COMMAND_LENGTH - 1; i++) {
        crc.add(command[i]);
    }
    return crc.calc() == command[COMMAND_LENGTH - 1];
}

void updateValves(const char* command) {
    for (int i = 0; i < NUM_VALVES; i++) {
        valveStates[i] = command[i] == '1';
        digitalWrite(valvePins[i], valveStates[i] ? HIGH : LOW);
    }
}

void shutOffValves() {
    for (int i = 0; i < NUM_VALVES; i++) {
        valveStates[i] = false;
        digitalWrite(valvePins[i], LOW);
    }
}

int readMuxChannel(int channel) {
    for (int i = 0; i < 4; i++) {
        digitalWrite(muxSelectPins[i], (channel >> i) & 1);
    }
    delayMicroseconds(5);  // Short delay for MUX to stabilize
    return analogRead(MUX_COMMON_PIN);
}

void sendStatus() {
    String status = "VALVES:";
    for (int i = 0; i < NUM_VALVES; i++) {
        status += valveStates[i] ? '1' : '0';
    }
    status += ",ANALOGS:";
    for (int i = 0; i < NUM_ANALOG_CHANNELS; i++) {
        status += String(readMuxChannel(i));
        if (i < NUM_ANALOG_CHANNELS - 1) {
            status += ",";
        }
    }

    crc.restart();
    for (unsigned int i = 0; i < status.length(); i++) {
        crc.add(status[i]);
    }
    status += ",CRC:" + String(crc.calc()) + "\n";

    Serial1.print(status);
}
