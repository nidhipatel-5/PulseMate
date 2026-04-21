#include <Wire.h>
#include "MAX30105.h"
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

MAX30105 particleSensor;
Adafruit_SSD1306 display(64, 32, &Wire, -1);

#define PIN_VIBRATION 10
#define PIN_BUTTON 7
#define POTS_THRESHOLD 30
#define BASELINE_SAMPLES 60
#define EPISODE_DURATION_MS 30000
#define FALSE_POSITIVE_DISPLAY_MS 5000

// Signal processing
float dcFilter = 0;
float acSignal = 0;
float previousAcSignal = 0;
bool isPeaking = false;
long lastBeatTime = 0;
int currentBPM = 0;

// Averaging
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
int beatAvg = 0;

// Baseline and episode
float baselineSamples[BASELINE_SAMPLES];
int baselineCount = 0;
float baselineBpm = 0.0f;
bool baselineValid = false;
bool episodeActive = false;
uint32_t episodeStartMs = 0;

// False positive
bool falsePositiveActive = false;
uint32_t falsePositiveStartMs = 0;

// Button
bool lastButtonState = HIGH;

uint32_t lastDisplayMs = 0;

// Draw small heart at x, y
void drawHeart(int x, int y) {
  display.drawPixel(x+1, y,   SSD1306_WHITE);
  display.drawPixel(x+3, y,   SSD1306_WHITE);
  display.drawPixel(x,   y+1, SSD1306_WHITE);
  display.drawPixel(x+1, y+1, SSD1306_WHITE);
  display.drawPixel(x+2, y+1, SSD1306_WHITE);
  display.drawPixel(x+3, y+1, SSD1306_WHITE);
  display.drawPixel(x+4, y+1, SSD1306_WHITE);
  display.drawPixel(x,   y+2, SSD1306_WHITE);
  display.drawPixel(x+1, y+2, SSD1306_WHITE);
  display.drawPixel(x+2, y+2, SSD1306_WHITE);
  display.drawPixel(x+3, y+2, SSD1306_WHITE);
  display.drawPixel(x+4, y+2, SSD1306_WHITE);
  display.drawPixel(x+1, y+3, SSD1306_WHITE);
  display.drawPixel(x+2, y+3, SSD1306_WHITE);
  display.drawPixel(x+3, y+3, SSD1306_WHITE);
  display.drawPixel(x+2, y+4, SSD1306_WHITE);
}

void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);

  if (falsePositiveActive) {
    display.println("False positive");
    display.println("detected!");
    display.println("Recalibrating");
    drawHeart(54, 24);
  } else if (episodeActive) {
    display.print("BPM:"); display.println(beatAvg);
    display.println("EPISODE!");
    display.print("End:");
    display.print((EPISODE_DURATION_MS - (millis() - episodeStartMs)) / 1000);
    display.println("s");
    drawHeart(54, 24);
  } else if (!baselineValid) {
    display.println("Calibrating");
    display.print(baselineCount); display.print("/"); display.println(BASELINE_SAMPLES);
    if (beatAvg > 0) { display.print("BPM:"); display.println(beatAvg); }
    drawHeart(54, 24);
  } else {
    if (beatAvg > 0) { display.print("BPM:"); display.println(beatAvg); }
    else display.println("No finger");
    display.print("Base:"); display.println((int)baselineBpm);
    drawHeart(54, 24);
  }

  display.display();
}

void resetBaseline() {
  baselineCount = 0;
  baselineBpm = 0.0f;
  baselineValid = false;
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_VIBRATION, OUTPUT);
  digitalWrite(PIN_VIBRATION, LOW);
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  Wire.begin();

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Starting...");
  display.display();

  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println("MAX30102 missing.");
    while (1);
  }

  byte ledBrightness = 60;
  byte sampleAverage = 4;
  byte ledMode = 2;
  int sampleRate = 100;
  int pulseWidth = 411;
  int adcRange = 4096;
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);

  dcFilter = particleSensor.getIR();
  Serial.println("Ready!");
}

void loop() {
  uint32_t nowMs = millis();

  // Button check
  bool buttonState = digitalRead(PIN_BUTTON);
  if (buttonState == LOW && lastButtonState == HIGH) {
    // Button pressed — cancel episode, mark false positive
    if (episodeActive) {
      episodeActive = false;
      digitalWrite(PIN_VIBRATION, LOW);
      falsePositiveActive = true;
      falsePositiveStartMs = nowMs;
      resetBaseline();
      Serial.println("False positive marked");
    }
  }
  lastButtonState = buttonState;

  // False positive display timeout
  if (falsePositiveActive && (nowMs - falsePositiveStartMs >= FALSE_POSITIVE_DISPLAY_MS)) {
    falsePositiveActive = false;
  }

  long rawIR = particleSensor.getIR();

  if (rawIR < 50000) {
    Serial.println("0,0");
    currentBPM = 0;
    beatAvg = 0;
    delay(20);
    if ((nowMs - lastDisplayMs) >= 250) {
      lastDisplayMs = nowMs;
      updateDisplay();
    }
    return;
  }

  dcFilter = (0.95 * dcFilter) + (0.05 * rawIR);
  acSignal = rawIR - dcFilter;

  float noiseThreshold = 50.0;
  long currentTime = millis();

  if (acSignal > noiseThreshold && previousAcSignal <= noiseThreshold && !isPeaking) {
    isPeaking = true;
    long timeSinceLastBeat = currentTime - lastBeatTime;

    if (timeSinceLastBeat > 300) {
      currentBPM = 60000.0 / timeSinceLastBeat;

      if (currentBPM > 40 && currentBPM < 200) {
        rates[rateSpot++] = (byte)currentBPM;
        rateSpot %= RATE_SIZE;

        beatAvg = 0;
        for (byte x = 0; x < RATE_SIZE; x++) beatAvg += rates[x];
        beatAvg /= RATE_SIZE;

        Serial.print("PulseWave:");
        Serial.print(acSignal);
        Serial.print(",");
        Serial.print("SmoothedBPM:");
        Serial.println(beatAvg);

        // Baseline collection
        if (!baselineValid && !episodeActive && !falsePositiveActive) {
          if (baselineCount < BASELINE_SAMPLES) {
            baselineSamples[baselineCount++] = currentBPM;
          }
          if (baselineCount >= BASELINE_SAMPLES) {
            float sum = 0;
            for (int i = 0; i < BASELINE_SAMPLES; i++) sum += baselineSamples[i];
            baselineBpm = sum / BASELINE_SAMPLES;
            baselineValid = true;
            Serial.print("Baseline set: "); Serial.println(baselineBpm);
          }
        }

        // Episode detection
        if (baselineValid && !episodeActive && !falsePositiveActive) {
          if ((float)beatAvg - baselineBpm >= POTS_THRESHOLD) {
            episodeActive = true;
            episodeStartMs = nowMs;
            digitalWrite(PIN_VIBRATION, HIGH);
            Serial.println("EPISODE STARTED");
          }
        }

        // Episode timeout
        if (episodeActive && (nowMs - episodeStartMs >= EPISODE_DURATION_MS)) {
          episodeActive = false;
          digitalWrite(PIN_VIBRATION, LOW);
          resetBaseline();
          Serial.println("Episode ended, recalibrating...");
        }
      }
      lastBeatTime = currentTime;
    }
  }

  if (acSignal < 0) isPeaking = false;
  previousAcSignal = acSignal;

  if ((nowMs - lastDisplayMs) >= 250) {
    lastDisplayMs = nowMs;
    updateDisplay();
  }

  delay(10);
}