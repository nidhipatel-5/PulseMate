#include <Wire.h>
#include <SPI.h>
#include "Config.h"
#include "DataTypes.h"
#include "Utils.h"
#include "SensorManager.h"
#include "SignalProcessor.h"
#include "EpisodeDetector.h"
#include "AlertManager.h"
#include "DisplayManager.h"
#include "RTCManager.h"
#include "StorageManager.h"
#include "ExportManager.h"
#include "HealthMonitor.h"
#include "ButtonManager.h"

SensorManager sensorMgr;
SignalProcessor signalProc;
EpisodeDetector episodeDetector;
AlertManager alertMgr;
DisplayManager displayMgr;
RTCManager rtcMgr;
StorageManager storageMgr;
ExportManager exportMgr;
HealthMonitor healthMon;
ButtonManager buttonMgr;

uint32_t lastSampleMs = 0;
uint32_t lastLogMs = 0;
uint32_t lastDisplayMs = 0;
uint32_t lastSummaryMs = 0;

HeartRateState hrState = {};
SignalQuality signalQuality = {};
SensorSample sensorSample = {};

void initHeadersIfNeeded() {
  // Optional: create CSV headers once if files are missing.
  // Kept lightweight here; add SD.exists() checks if desired.
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial) {}

  pinMode(PIN_VIBRATION, OUTPUT);
  digitalWrite(PIN_VIBRATION, LOW);

  buttonMgr.begin(PIN_BUTTON);

  Wire.begin();
  SPI.begin();

  displayMgr.begin();
  displayMgr.showBoot();

  bool rtcOk = rtcMgr.begin();
  bool sensorOk = sensorMgr.begin();
  bool sdOk = storageMgr.begin(PIN_SD_CS);
  exportMgr.begin();

  signalProc.begin();
  episodeDetector.begin();
  alertMgr.begin(PIN_VIBRATION);
  healthMon.begin();

  initHeadersIfNeeded();

  if (!rtcOk || !sensorOk || !sdOk) {
    displayMgr.showError("Init failed", "Check wiring");
  }

  Serial.println("Pulse Mate started");
}

void loop() {
  const uint32_t nowMs = millis();

  buttonMgr.update(nowMs);

  // 100 Hz sample loop
  if ((nowMs - lastSampleMs) >= SENSOR_SAMPLE_PERIOD_MS) {
    lastSampleMs = nowMs;

    sensorMgr.readSample(sensorSample, signalQuality, nowMs);
    signalProc.update(sensorSample, signalQuality, hrState, nowMs);
  }

  DateTimeParts now = rtcMgr.now();

  // Episode logic
  EpisodeRecord episodeEvent = {};
  bool episodeStarted = false;
  bool episodeEnded = false;

  episodeDetector.update(hrState, now, episodeEvent, episodeStarted, episodeEnded);

  if (episodeStarted) {
    storageMgr.logEpisodeStart(episodeEvent);
    Serial.print("Episode started: ");
    Serial.println(episodeEvent.episodeId);
  }

  if (episodeEnded) {
    storageMgr.logEpisodeEnd(episodeEvent);
    Serial.print("Episode ended: ");
    Serial.println(episodeEvent.episodeId);
  }

  // False positive button
  if (buttonMgr.wasPressed()) {
    FalsePositiveRecord fp = {};
    if (episodeDetector.markFalsePositive(now, fp)) {
      storageMgr.logFalsePositive(fp);
      Serial.print("False positive marked for ");
      Serial.println(fp.episodeId);
    }
  }

  // Alerts
  alertMgr.update(episodeDetector.isEpisodeActive(), nowMs);

  // 1 Hz HR log
  if ((nowMs - lastLogMs) >= HR_LOG_PERIOD_MS) {
    lastLogMs = nowMs;

    HeartRateLog log = {};
    log.epoch = now.epoch;
    log.bpmInstant = hrState.instantValid ? hrState.instantBpm : 0.0f;
    log.bpmActive = hrState.activeValid ? hrState.activeBpm : 0.0f;
    log.bpmBaseline = hrState.baselineValid ? hrState.baselineBpm : 0.0f;
    log.signalValid = signalQuality.validSample;
    log.episodeActive = episodeDetector.isEpisodeActive();
    if (log.episodeActive) {
      Utils::safeStrCopy(log.episodeId, episodeDetector.currentEpisode().episodeId, sizeof(log.episodeId));
    } else {
      log.episodeId[0] = '\0';
    }

    if (now.valid) {
      storageMgr.logHeartRate(log);
      storageMgr.updateDailyStats(log.bpmActive, hrState.activeValid, log.episodeActive, now);
    }

    Serial.print("HR active: ");
    Serial.print(log.bpmActive);
    Serial.print(" baseline: ");
    Serial.print(log.bpmBaseline);
    Serial.print(" episode: ");
    Serial.println(log.episodeActive ? "1" : "0");
  }

  // Display refresh
  if ((nowMs - lastDisplayMs) >= DISPLAY_UPDATE_PERIOD_MS) {
    lastDisplayMs = nowMs;

    const char* epId = episodeDetector.isEpisodeActive()
      ? episodeDetector.currentEpisode().episodeId
      : "";

    displayMgr.update(hrState, episodeDetector.isEpisodeActive(), epId, healthMon.health());
  }

  // Summary/export maintenance
  if ((nowMs - lastSummaryMs) >= SUMMARY_UPDATE_PERIOD_MS) {
    lastSummaryMs = nowMs;

    if (now.valid) {
      storageMgr.finalizeSummaryIfDayChanged(now);
      storageMgr.cleanupOldFiles(now);
      exportMgr.exportCsvManifest(now);
      if (ENABLE_PDF_PLACEHOLDER_EXPORT) {
        exportMgr.exportPdfPlaceholder(now);
      }
    }
  }

  // Health monitoring and recovery
  healthMon.update(nowMs, sensorMgr, rtcMgr, displayMgr, storageMgr);

  if (healthMon.health().degradedMode) {
    displayMgr.showError("Subsystem err", "Degraded mode");
  }
}
