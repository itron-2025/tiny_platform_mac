// =============================================================================
//  tiny_probe  -  Throwaway board-identification sketch. NOT part of the robot.
//
//  Purpose: decide WROOM-32 vs WROVER without reading the tiny print on the
//  module's shield. The two share the same die (ESP32-D0WD-V3), so esptool
//  cannot tell them apart -- but WROVER wires GPIO16/17 to an on-module PSRAM
//  chip, which makes those two pins unusable. The pin map in docs/WIRING.md
//  assigns 16/17 to the FR motor, so this has to be settled before wiring.
//
//  MUST be compiled with PSRAM enabled, or psramFound() returns false on every
//  board and the test proves nothing:
//    arduino-cli compile --fqbn esp32:esp32:esp32:PSRAM=enabled firmware/tiny_probe
//
//  Safe to run with nothing wired: it drives no GPIO except the on-board LED.
// =============================================================================
#include <Arduino.h>

#define LED_PIN 2

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  delay(1500);  // let the USB-serial side settle before the first print
}

void loop() {
  bool psram = psramFound();

  Serial.println();
  Serial.println("===== tiny_platform_mac board probe =====");
  Serial.printf("chip model      : %s\n", ESP.getChipModel());
  Serial.printf("chip revision   : %d\n", ESP.getChipRevision());
  Serial.printf("cpu cores       : %d\n", ESP.getChipCores());
  Serial.printf("cpu freq        : %u MHz\n", ESP.getCpuFreqMHz());
  Serial.printf("flash size      : %u bytes (%u MB)\n",
                ESP.getFlashChipSize(), ESP.getFlashChipSize() / (1024 * 1024));
  Serial.printf("psram found     : %s\n", psram ? "YES" : "no");
  Serial.printf("psram size      : %u bytes\n", ESP.getPsramSize());
  Serial.printf("free heap       : %u bytes\n", ESP.getFreeHeap());
  Serial.printf("efuse MAC       : %012llX\n", ESP.getEfuseMac());
  Serial.println();
  // The whole point of the sketch:
  Serial.printf(">>> VERDICT: %s\n",
                psram ? "WROVER (PSRAM present) -- GPIO16/17 are NOT usable!"
                      : "WROOM-32 (no PSRAM)    -- GPIO16/17 are free to use.");
  Serial.println("=========================================");

  digitalWrite(LED_PIN, HIGH);
  delay(200);
  digitalWrite(LED_PIN, LOW);
  delay(1800);
}
