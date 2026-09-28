#pragma once

#include <Mesh.h>
#include <helpers/ArduinoHelpers.h>
#include <helpers/StaticPoolPacketManager.h>
#include <helpers/SimpleMeshTables.h>

// First byte of every payload, so more message types can be added later
#define SONAR_MSG_DISTANCE   0x01

/*
 * Placeholder mesh: sends readings as unencrypted RAW_CUSTOM packets to
 * zero-hop neighbours only (MeshCore does not re-flood RAW_CUSTOM packets).
 * Payload for SONAR_MSG_DISTANCE: [type:1][distance_mm:2, little-endian]
 */
class SonarMesh : public mesh::Mesh {
  bool _has_received = false;
  uint16_t _received_mm = 0;
  float _received_snr = 0;

public:
  SonarMesh(mesh::Radio& radio, mesh::RNG& rng, mesh::RTCClock& rtc, SimpleMeshTables& tables)
    : mesh::Mesh(radio, *new ArduinoMillis(), rng, rtc, *new StaticPoolPacketManager(8), tables) { }

  void sendDistance(uint16_t distance_mm) {
    uint8_t data[3];
    data[0] = SONAR_MSG_DISTANCE;
    data[1] = distance_mm & 0xFF;
    data[2] = distance_mm >> 8;

    mesh::Packet* pkt = createRawData(data, sizeof(data));
    if (pkt) {
      sendZeroHop(pkt);
    }
  }

  // Returns true (once) when a new distance has arrived since the last call
  bool getReceivedDistance(uint16_t& distance_mm, float& snr) {
    if (!_has_received) return false;
    _has_received = false;
    distance_mm = _received_mm;
    snr = _received_snr;
    return true;
  }

protected:
  void onRawDataRecv(mesh::Packet* packet) override {
    if (packet->payload_len >= 3 && packet->payload[0] == SONAR_MSG_DISTANCE) {
      _received_mm = packet->payload[1] | (packet->payload[2] << 8);
      _received_snr = packet->getSNR();
      _has_received = true;
    }
  }
};
