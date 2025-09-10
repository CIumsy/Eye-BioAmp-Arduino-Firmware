// This code is designed to work specifically on:
// 1. Arduino UNO R4 Minima
// 2. Arduino UNO R4 WiFi

// EOG Dino Game Controller using Single Blink Detection
// Single Blink = Space Bar (Jump/Action)
// Perfect for Chrome Dino Game, Flappy Bird, or any space-triggered games
// Adapted from Slide-Control.ino with simplified single blink detection

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.

// Copyright (c) 2025 Krishnanshu Mittal - krishnanshu@upsidedownlabs.tech
// Copyright (c) 2025 Aman Maheshwari - aman@upsidedownlabs.tech 
// Copyright (c) 2025 Deepak Khatri - deepak@upsidedownlabs.tech
// Copyright (c) 2025 Upside Down Labs - contact@upsidedownlabs.tech

// At Upside Down Labs, we create open-source DIY neuroscience hardware and software.
// Our mission is to make neuroscience affordable and accessible for everyone.
// By supporting us with your purchase, you help spread innovation and open science.
// Thank you for being part of this journey with us!

#include <Arduino.h>
#include <Keyboard.h>  // HID keyboard library for Arduino R4
#include <math.h>

#define DEBUG  // Uncomment this line to enable debugging

// ----------------- USER CONFIGURATION -----------------
#define SAMPLE_RATE       512          // samples per second
#define BAUD_RATE         115200
#define INPUT_PIN         A0
#define LED_PIN           LED_BUILTIN

// Envelope Configuration for EOG detection
#define ENVELOPE_WINDOW_MS 100  // Smoothing window in milliseconds
#define ENVELOPE_WINDOW_SIZE ((ENVELOPE_WINDOW_MS * SAMPLE_RATE) / 1000)

// Blink Detection Thresholds - adjust these based on your setup
const float BlinkLowerThreshold = 30.0;
const float BlinkUpperThreshold = 50.0;

// Circular buffer for timing-based sampling
#define BUFFER_SIZE 64
float eogCircBuffer[BUFFER_SIZE];
int writeIndex = 0;
int readIndex = 0;
int samplesAvailable = 0;

// Single Blink Detection Configuration
const unsigned long BLINK_DEBOUNCE_MS = 300;   // minimum time between blinks to prevent double-triggering
unsigned long lastBlinkTime = 0;               // time of most recent blink
float currentEOGEnvelope = 0;

// HID Command Cooldown to prevent rapid-fire commands
const unsigned long HID_COOLDOWN_MS = 250;  // 250ms between space commands (allows ~4 jumps per second)
unsigned long lastHIDCommandTime = 0;

// EOG Envelope Processing Variables
float eogEnvelopeBuffer[ENVELOPE_WINDOW_SIZE] = {0};
int eogEnvelopeIndex = 0;
float eogEnvelopeSum = 0;

// Game Statistics
unsigned long totalBlinks = 0;
unsigned long gameStartTime = 0;

// --- Filter Functions ---

// Band-Stop Butterworth IIR digital filter, generated using filter_gen.py.
// Sampling rate: 512.0 Hz, frequency: [48.0, 52.0] Hz.
// Filter is order 2, implemented as second-order sections (biquads).
// Reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
float Notch(float input)
{
  float output = input;
  {
    static float z1, z2; // filter section state
    float x = output - -1.58696045*z1 - 0.96505858*z2;
    output = 0.96588529*x + -1.57986211*z1 + 0.96588529*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - -1.62761184*z1 - 0.96671306*z2;
    output = 1.00000000*x + -1.63566226*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  return output;
}

// High-Pass Butterworth IIR digital filter, generated using filter_gen.py.
// Sampling rate: 512.0 Hz, frequency: 5.0 Hz.
// Filter is order 2, implemented as second-order sections (biquads).
// Reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
float EOGFilter(float input)
{
  float output = input;
  {
    static float z1, z2; // filter section state
    float x = output - -1.91327599*z1 - 0.91688335*z2;
    output = 0.95753983*x + -1.91507967*z1 + 0.95753983*z2;
    z2 = z1;
    z1 = x;
  }
  return output;
}

float updateEOGEnvelope(float sample) 
{
  float absSample = fabs(sample); 

  // Update circular buffer and running sum
  eogEnvelopeSum -= eogEnvelopeBuffer[eogEnvelopeIndex];
  eogEnvelopeSum += absSample;
  eogEnvelopeBuffer[eogEnvelopeIndex] = absSample;
  eogEnvelopeIndex = (eogEnvelopeIndex + 1) % ENVELOPE_WINDOW_SIZE;

  return eogEnvelopeSum / ENVELOPE_WINDOW_SIZE;  // Return moving average
}

// HID Keyboard Functions
void sendSpaceBar() {
  unsigned long nowMs = millis();
  if ((nowMs - lastHIDCommandTime) >= HID_COOLDOWN_MS) {
    Keyboard.press(' ');  // Press space bar
    delay(30);            // Hold key for 30ms (shorter for gaming responsiveness)
    Keyboard.release(' ');
    lastHIDCommandTime = nowMs;
    totalBlinks++;
    
    Serial.print("JUMP! Blink #");
    Serial.print(totalBlinks);
    Serial.print(" | EOG: ");
    Serial.println(currentEOGEnvelope);
    
    // LED feedback - quick single flash
    digitalWrite(LED_PIN, HIGH);
    delay(50);
    digitalWrite(LED_PIN, LOW);
  }
}

void setup() {
  Serial.begin(BAUD_RATE);
  while (!Serial);
  
  pinMode(INPUT_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  
  // Initialize HID Keyboard
  Keyboard.begin();
  
  // LED startup sequence - game ready indicator
  for(int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
  
  gameStartTime = millis();
  
  Serial.println("=================================");
  Serial.println("Arduino R4 EOG Dino Game Controller");
  Serial.println("=================================");
  Serial.println("Single Blink = Space Bar (Jump)");
  Serial.println("Perfect for Chrome Dino Game!");
  Serial.println("");
  Serial.println("Instructions:");
  Serial.println("1. Open Chrome and go to chrome://dino/");
  Serial.println("2. Or disconnect internet and try to browse");
  Serial.println("3. Press spacebar once to start game");
  Serial.println("4. Then use blinks to jump!");
  Serial.println("");
  Serial.println("Starting EOG monitoring at 512 Hz...");
  Serial.println("Ready to play! 🦕");
}

void loop() {
    static unsigned long lastMicros = 0;
    static long timer = 0;
    
    digitalWrite(LED_PIN, LOW);  // Default LED state
    
    // Timing-based sampling for 512 Hz
    unsigned long currentMicros = micros();
    unsigned long interval = currentMicros - lastMicros;
    lastMicros = currentMicros;
    
    timer -= interval;
    if (timer < 0) {
        timer += 1000000 / SAMPLE_RATE; // 1,000,000 / 512 ≈ 1953 microseconds
        
        // Sample and filter for EOG
        int raw = analogRead(INPUT_PIN);
        float filtered = Notch(raw);
        float eog = EOGFilter(filtered);
        
        // Store in circular buffer
        eogCircBuffer[writeIndex] = eog;
        writeIndex = (writeIndex + 1) % BUFFER_SIZE;
        samplesAvailable++;
        if (samplesAvailable > BUFFER_SIZE) {
            samplesAvailable = BUFFER_SIZE; // Prevent overflow
        }
    }
    
    // Process all available samples from circular buffer
    while (samplesAvailable > 0) {
        float eog = eogCircBuffer[readIndex];
        readIndex = (readIndex + 1) % BUFFER_SIZE;
        samplesAvailable--;
        
        // Process the sample (envelope calculation)
        currentEOGEnvelope = updateEOGEnvelope(eog);
    }
    
    // Get current time for blink detection logic
    unsigned long nowMs = millis();
    
    // ===== SINGLE BLINK DETECTION AND SPACE BAR CONTROL =====
    if (currentEOGEnvelope > BlinkLowerThreshold && 
        currentEOGEnvelope < BlinkUpperThreshold && 
        (nowMs - lastBlinkTime) >= BLINK_DEBOUNCE_MS) {
        
        lastBlinkTime = nowMs;
        
        #ifdef DEBUG
        Serial.print("Blink detected! EOG Envelope: ");
        Serial.print(currentEOGEnvelope);
        Serial.print(" | Time since last: ");
        Serial.print(nowMs - lastBlinkTime + BLINK_DEBOUNCE_MS);
        Serial.println(" ms");
        #endif
        
        // Send space bar immediately for single blink
        sendSpaceBar();
    }
    
    // ===== PERIODIC STATUS UPDATES =====
    static unsigned long lastStatusUpdate = 0;
    if ((nowMs - lastStatusUpdate) >= 30000) {  // Every 30 seconds
        unsigned long gameTimeSeconds = (nowMs - gameStartTime) / 1000;
        float blinksPerMinute = (totalBlinks * 60.0) / (gameTimeSeconds + 1);
        
        Serial.println("");
        Serial.println("=== Game Stats ===");
        Serial.print("Game Time: ");
        Serial.print(gameTimeSeconds);
        Serial.println(" seconds");
        Serial.print("Total Jumps: ");
        Serial.println(totalBlinks);
        Serial.print("Jump Rate: ");
        Serial.print(blinksPerMinute, 1);
        Serial.println(" per minute");
        Serial.print("Current EOG Level: ");
        Serial.println(currentEOGEnvelope);
        Serial.println("==================");
        Serial.println("");
        
        lastStatusUpdate = nowMs;
    }
    
    // ===== REAL-TIME EOG MONITORING (DEBUG) =====
    #ifdef DEBUG
    static unsigned long lastDebugPrint = 0;
    if ((nowMs - lastDebugPrint) >= 2000) {  // Every 2 seconds
        Serial.print("EOG Monitor: ");
        Serial.print(currentEOGEnvelope);
        Serial.print(" | Threshold: ");
        Serial.print(BlinkLowerThreshold);
        Serial.print("-");
        Serial.print(BlinkUpperThreshold);
        Serial.print(" | Jumps: ");
        Serial.println(totalBlinks);
        lastDebugPrint = nowMs;
    }
    #endif
}