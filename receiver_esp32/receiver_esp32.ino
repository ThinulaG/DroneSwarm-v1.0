// Mocap drone-side ESP32-C3.
// ESP-NOW <-> CRSF bridge + CRSF telemetry relay back to the laptop.
//
// Inbound (sender -> here):  ControlPacket via ESP-NOW
//                            -> packed RC channels -> CRSF UART to Betaflight FC.
//                            500 ms failsafe forces safe sticks if comm dies.
//
// Outbound (here -> sender):  CRSF telemetry ATTITUDE frames (type 0x1E)
//                            received from FC TX line are decoded and
//                            forwarded as TelemetryPacket via ESP-NOW @ 50 Hz.
//                            The laptop ESP32 then prints "H<yaw>" to USB.
//
// The CRSF UART (RX pin 20 / TX pin 21) is already bidirectional in your
// wiring -- no new wires needed. Enable CRSF telemetry on this UART in
// Betaflight's Ports tab.

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// ================= USER SETTINGS =================
#define CRSF_RX_PIN 20
#define CRSF_TX_PIN 21
#define ESPNOW_CHANNEL 1
#define FAILSAFE_MS 500
#define TELEMETRY_PERIOD_MS 20   // 50 Hz drone -> laptop attitude updates
#define DEBUG_RX_PRINT 0
// =================================================

// Sender ESP32's STA MAC. Replace with the MAC printed at boot by
// sender_esp32.ino's setup() ("[sender] STA MAC: ...").
uint8_t senderAddress[] = { 0x70, 0x4B, 0xCA, 0x48, 0xC1, 0x24 };

HardwareSerial CRSFSerial(1);

typedef struct __attribute__((packed)) {
  uint16_t throttle_us;
  uint16_t roll_us;
  uint16_t pitch_us;
  uint16_t yaw_us;
  uint8_t  armed;
  uint32_t seq;
} ControlPacket;

// MUST match sender_esp32.ino exactly.
typedef struct __attribute__((packed)) {
  int16_t  pitch_centirad;
  int16_t  roll_centirad;
  int16_t  yaw_centirad;
  uint32_t seq;
} TelemetryPacket;

ControlPacket lastCmd;
TelemetryPacket telem = {0, 0, 0, 0};
uint32_t lastRecvTime = 0;
uint32_t lastTelemSend = 0;
uint16_t channels[16];

// ---------------- CRSF ----------------

uint8_t crc8(const uint8_t *ptr, uint8_t len) {
  uint8_t crc = 0;
  while (len--) {
    crc ^= *ptr++;
    for (uint8_t i = 0; i < 8; i++) {
      crc = (crc & 0x80) ? (crc << 1) ^ 0xD5 : (crc << 1);
    }
  }
  return crc;
}

uint16_t usToCRSF(uint16_t us) {
  us = constrain(us, 1000, 2000);
  return map(us, 1000, 2000, 172, 1811);
}

void setSafeChannels() {
  for (int i = 0; i < 16; i++) channels[i] = 992;
  channels[0] = usToCRSF(1500);
  channels[1] = usToCRSF(1500);
  channels[2] = usToCRSF(1000);
  channels[3] = usToCRSF(1500);
  channels[4] = usToCRSF(1000);
}

void applyCommandToChannels(const ControlPacket &cmd) {
  for (int i = 0; i < 16; i++) channels[i] = 992;
  channels[0] = usToCRSF(cmd.roll_us);
  channels[1] = usToCRSF(cmd.pitch_us);
  channels[2] = usToCRSF(cmd.throttle_us);
  channels[3] = usToCRSF(cmd.yaw_us);
  channels[4] = cmd.armed ? usToCRSF(2000) : usToCRSF(1000);
}

void sendCRSF() {
  uint8_t packet[26];
  packet[0] = 0xC8;
  packet[1] = 24;
  packet[2] = 0x16;

  uint32_t buffer = 0;
  uint8_t bits = 0;
  int idx = 3;
  for (int i = 0; i < 16; i++) {
    buffer |= ((uint32_t)(channels[i] & 0x07FF)) << bits;
    bits += 11;
    while (bits >= 8) {
      packet[idx++] = buffer & 0xFF;
      buffer >>= 8;
      bits -= 8;
    }
  }
  packet[25] = crc8(&packet[2], 23);
  CRSFSerial.write(packet, 26);
}

// ---------------- CRSF telemetry parser ----------------
// Frame: [addr][len][type][payload...][crc]
//   len = type + payload + crc count = payload_size + 2
//   crc8 (poly 0xD5) computed over type + payload (len - 1 bytes from buf[2]).
// Address may be 0xC8 (FC), 0xEA (handset), 0xC8 (broadcast) depending on
// origin -- accept any common value to stay forgiving.

#define CRSF_FRAMETYPE_ATTITUDE 0x1E

enum CrsfParseState { CRSF_WAIT_SYNC, CRSF_READ_LEN, CRSF_READ_DATA };
static CrsfParseState crsfState = CRSF_WAIT_SYNC;
static uint8_t crsfBuf[64];
static uint8_t crsfLen = 0;
static uint8_t crsfIdx = 0;

static bool isCrsfAddr(uint8_t b) {
  return (b == 0xC8 || b == 0xEA || b == 0xEC || b == 0xEE);
}

void parseCRSFByte(uint8_t b) {
  switch (crsfState) {
    case CRSF_WAIT_SYNC:
      if (isCrsfAddr(b)) {
        crsfBuf[0] = b;
        crsfState = CRSF_READ_LEN;
      }
      break;

    case CRSF_READ_LEN:
      if (b >= 2 && b <= 62) {
        crsfBuf[1] = b;
        crsfLen = b;
        crsfIdx = 0;
        crsfState = CRSF_READ_DATA;
      } else {
        crsfState = CRSF_WAIT_SYNC;
      }
      break;

    case CRSF_READ_DATA:
      crsfBuf[2 + crsfIdx++] = b;
      if (crsfIdx >= crsfLen) {
        uint8_t type    = crsfBuf[2];
        uint8_t recvCrc = crsfBuf[1 + crsfLen];
        uint8_t calcCrc = crc8(&crsfBuf[2], crsfLen - 1);
        if (recvCrc == calcCrc && type == CRSF_FRAMETYPE_ATTITUDE && crsfLen == 8) {
          // payload at crsfBuf[3..8]: pitch, roll, yaw  (each int16 BE, 1/10000 rad)
          int16_t pitch_cr = ((int16_t)crsfBuf[3] << 8) | crsfBuf[4];
          int16_t roll_cr  = ((int16_t)crsfBuf[5] << 8) | crsfBuf[6];
          int16_t yaw_cr   = ((int16_t)crsfBuf[7] << 8) | crsfBuf[8];
          telem.pitch_centirad = pitch_cr;
          telem.roll_centirad  = roll_cr;
          telem.yaw_centirad   = yaw_cr;
        }
        crsfState = CRSF_WAIT_SYNC;
      }
      break;
  }
}

// ---------------- ESP-NOW ----------------

void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  if (len != sizeof(ControlPacket)) return;
  memcpy(&lastCmd, incomingData, sizeof(lastCmd));
  lastRecvTime = millis();
  applyCommandToChannels(lastCmd);

}

// ---------------- setup / loop ----------------

void setup() {
  Serial.begin(115200);
  delay(500);

  setSafeChannels();
  CRSFSerial.begin(420000, SERIAL_8N1, CRSF_RX_PIN, CRSF_TX_PIN);

  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_CHANNEL);

  uint8_t newMAC[] = { 0x10, 0x00, 0x3B, 0xB1, 0x5B, 0x8C };
  esp_wifi_set_mac(WIFI_IF_STA, newMAC);

  uint8_t staMac[6];
  esp_wifi_get_mac(WIFI_IF_STA, staMac);
  Serial.printf("[receiver] STA MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                staMac[0], staMac[1], staMac[2],
                staMac[3], staMac[4], staMac[5]);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);

  // Add the sender ESP32 as an outbound peer so we can ESP-NOW telemetry to it.
  esp_now_peer_info_t senderPeer = {};
  memcpy(senderPeer.peer_addr, senderAddress, 6);
  senderPeer.channel = ESPNOW_CHANNEL;
  senderPeer.encrypt = false;
  if (esp_now_add_peer(&senderPeer) != ESP_OK) {
    Serial.println("Failed to add sender peer (check senderAddress[])");
  }

  Serial.println("Receiver ready: ESP-NOW -> CRSF; CRSF telem -> ESP-NOW");
}

void loop() {
  // 1) Drain CRSF UART for telemetry frames from FC
  while (CRSFSerial.available()) {
    parseCRSFByte((uint8_t)CRSFSerial.read());
  }

  // 2) Failsafe if PC comm died
  if (millis() - lastRecvTime > FAILSAFE_MS) {
    setSafeChannels();
  }

  // 3) Send RC channels packed to FC
  sendCRSF();

  // 4) Periodically forward latest attitude to laptop
  if (millis() - lastTelemSend >= TELEMETRY_PERIOD_MS) {
    lastTelemSend = millis();
    telem.seq++;
    esp_now_send(senderAddress, (uint8_t *)&telem, sizeof(telem));
  }

  delay(4); // ~250 Hz CRSF output cadence
}
