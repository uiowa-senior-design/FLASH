#pragma once

#include <Mesh.h>
#include <helpers/ArduinoHelpers.h>
#include <helpers/StaticPoolPacketManager.h>
#include <helpers/SimpleMeshTables.h>

// Group datagram data type (FF00-FFFF is MeshCore's unregistered dev range, see docs/number_allocations.md)
#define SONAR_DATA_TYPE_DISTANCE   0xFF01

#ifndef SONAR_MAX_HOPS
#define SONAR_MAX_HOPS  4   // packets that have already been relayed this many times are not relayed again
#endif

/*
 * Multi-hop mesh: readings are flooded as encrypted GRP_DATA packets on a shared channel,
 * and every node re-broadcasts packets it hasn't seen before (up to SONAR_MAX_HOPS).
 * Decrypted payload: [data_type:2][data_len:1][distance_mm:2][seq:2], all little-endian.
 * seq makes each reading unique, so repeated identical distances aren't dropped as duplicates.
 */
class SonarMesh : public mesh::Mesh {
  mesh::GroupChannel _channel;
  uint16_t _seq = 0;

  bool _has_received = false;
  uint16_t _received_mm = 0;
  float _received_snr = 0;
  uint8_t _received_hops = 0;

public:
  SonarMesh(mesh::Radio& radio, mesh::RNG& rng, mesh::RTCClock& rtc, SimpleMeshTables& tables)
    : mesh::Mesh(radio, *new ArduinoMillis(), rng, rtc, *new StaticPoolPacketManager(8), tables) {
    // Shared 128-bit channel key, must be identical on every node. Change it for your own deployment.
    static const uint8_t key[CIPHER_KEY_SIZE] = {
      0x5f, 0x1a, 0xc3, 0x72, 0x9e, 0x04, 0xb8, 0x2d, 0x61, 0xf7, 0x3c, 0x90, 0x4e, 0xa5, 0x18, 0xdb
    };
    memset(_channel.secret, 0, sizeof(_channel.secret));
    memcpy(_channel.secret, key, sizeof(key));
    mesh::Utils::sha256(_channel.hash, sizeof(_channel.hash), key, sizeof(key));
  }

  void begin() {
    mesh::Mesh::begin();
    // Random start so readings sent after a reboot don't match ones neighbours still remember as seen
    _seq = getRNG()->nextInt(0, 0x10000);
  }

  void sendDistance(uint16_t distance_mm) {
    uint8_t data[7];
    data[0] = SONAR_DATA_TYPE_DISTANCE & 0xFF;
    data[1] = SONAR_DATA_TYPE_DISTANCE >> 8;
    data[2] = 4;   // data_len
    data[3] = distance_mm & 0xFF;
    data[4] = distance_mm >> 8;
    data[5] = _seq & 0xFF;
    data[6] = _seq >> 8;
    _seq++;

    mesh::Packet* pkt = createGroupDatagram(PAYLOAD_TYPE_GRP_DATA, _channel, data, sizeof(data));
    if (pkt) {
      sendFlood(pkt);
    }
  }

  // Returns true (once) when a new distance has arrived since the last call.
  // hops = number of nodes that relayed it (0 = heard directly from the sender)
  bool getReceivedDistance(uint16_t& distance_mm, float& snr, uint8_t& hops) {
    if (!_has_received) return false;
    _has_received = false;
    distance_mm = _received_mm;
    snr = _received_snr;
    hops = _received_hops;
    return true;
  }

protected:
  bool allowPacketForward(const mesh::Packet* packet) override {
    return packet->isRouteFlood() && packet->getPathHashCount() < SONAR_MAX_HOPS;
  }

  int searchChannelsByHash(const uint8_t* hash, mesh::GroupChannel channels[], int max_matches) override {
    if (max_matches > 0 && hash[0] == _channel.hash[0]) {
      channels[0] = _channel;
      return 1;
    }
    return 0;
  }

  void onGroupDataRecv(mesh::Packet* packet, uint8_t type, const mesh::GroupChannel& channel, uint8_t* data, size_t len) override {
    // len is padded up to the cipher block size, so trust data_len rather than len
    if (type != PAYLOAD_TYPE_GRP_DATA || len < 3) return;
    uint16_t data_type = data[0] | (data[1] << 8);
    uint8_t data_len = data[2];
    if (data_type != SONAR_DATA_TYPE_DISTANCE || data_len < 4 || 3 + data_len > len) return;

    _received_mm = data[3] | (data[4] << 8);
    _received_snr = packet->getSNR();
    _received_hops = packet->getPathHashCount();
    _has_received = true;
  }
};
