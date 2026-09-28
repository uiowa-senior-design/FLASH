#include "MB7389.h"
#include "SonarMesh.h"

#include <Arduino.h> // needed for PlatformIO
#include <Wire.h>
#include <target.h>

#ifndef LORA_CR
#define LORA_CR 5
#endif

#ifndef SONAR_SEND_INTERVAL_MS
#define SONAR_SEND_INTERVAL_MS 15000 // how often to broadcast the latest reading
#endif

#ifndef SONAR_TIMEOUT_MS
#define SONAR_TIMEOUT_MS 2000 // no reading for this long = no sensor connected
#endif

StdRNG fast_rng;
SimpleMeshTables tables;
SonarMesh the_mesh(radio_driver, fast_rng, rtc_clock, tables);
MB7389 sonar;

unsigned long next_send_at = 0;
bool was_connected = false;

// The sensor counts as connected if it has sent a reading recently
bool sensorConnected() {
  return sonar.getLastReadAt() > 0 && millis() - sonar.getLastReadAt() < SONAR_TIMEOUT_MS;
}

void halt() {
  while (1)
    ;
}

void setup() {
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE, HIGH);

  Serial.begin(115200);
  delay(1000);
  board.begin();
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, LOW);

  if (!radio_init()) {
    halt();
  }

  fast_rng.begin(radio_driver.getRngSeed());
  the_mesh.begin();

  radio_driver.setParams(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR);
  radio_driver.setTxPower(LORA_TX_POWER);

  // Serial1 uses D6 (TX) / D7 (RX), the same pins as I2C on this board,
  // so release I2C before starting the UART.
  Serial1.begin(9600);
  while (!Serial1)
    ;
  sonar.begin(Serial1);

  if(Serial){
    Serial.println("Sonar sensor started");
  }
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);
}

void loop() {
  sonar.poll();

  bool connected = sensorConnected();
  if (connected != was_connected) {
    was_connected = connected;
    Serial.println(connected ? "Sensor connected - sending readings" : "No sensor - listening for readings");
  }

  if (connected) {
    // Sender: broadcast the latest reading every SONAR_SEND_INTERVAL_MS
    if (millis() >= next_send_at) {
      next_send_at = millis() + SONAR_SEND_INTERVAL_MS;
      Serial.print("TX distance: ");
      Serial.print(sonar.getDistanceMM());
      Serial.println(" mm");
      the_mesh.sendDistance(sonar.getDistanceMM());
    }
  } else {
    // Receiver: print readings from other nodes
    uint16_t distance_mm;
    float snr;
    if (the_mesh.getReceivedDistance(distance_mm, snr)) {
      Serial.print("RX distance: ");
      Serial.print(distance_mm);
      Serial.print(" mm  (SNR ");
      Serial.print(snr);
      Serial.println(")");
    }
  }

  the_mesh.loop();
  rtc_clock.tick();
}
