// Mocap PC <-> drone bridge (laptop-side ESP32, USB serial <-> ESP-NOW).
//
// Direction PC -> drone:
//   Reads "throttle,roll,pitch,yaw,armed\n" CSV from USB serial @ 115200,
//   forwards as a binary ControlPacket via ESP-NOW @ 50 Hz to the drone.
//
// Direction drone -> PC (NEW vs your manual-control sender):
//   ESP-NOW receives a binary TelemetryPacket (CRSF attitude relayed by the
//   drone ESP32-C3), and prints "H<yaw_rad>\n" over USB serial so the Python
//   backend can fuse heading.
//
// Also: prints this board's STA MAC at boot so you can paste it into the
// drone firmware's `senderAddress[]`.

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// ================= USER SETTINGS =================
uint8_t receiverAddress[] = { 0x10, 0x00, 0x3B, 0xB1, 0x5B, 0x8C };
#define ESPNOW_CHANNEL 1
#define SEND_PERIOD_MS 20   // 50 Hz from transmitter to receiver
#define DEBUG_CMD_PRINT 0
// =================================================

typedef struct __attribute__((packed)) {
  uint16_t throttle_us;
  uint16_t roll_us;
  uint16_t pitch_us;
  uint16_t yaw_us;
  uint8_t  armed;
  uint32_t seq;
} ControlPacket;

// MUST match the struct in drone_receiver_crsf_espnow.ino exactly.
typedef struct __attribute__((packed)) {
  int16_t  pitch_centirad;   // 1/10000 rad (CRSF native units)
  int16_t  roll_centirad;
  int16_t  yaw_centirad;
  uint32_t seq;
} TelemetryPacket;

ControlPacket cmd = {1000, 1500, 1500, 1500, 0, 0};
uint32_t lastSend = 0;
String inputLine = "";

uint16_t clampUS(long v) {
  if (v < 1000) return 1000;
  if (v > 2000) return 2000;
  return (uint16_t)v;
}

void parseSerialCommand(String line) {
  line.trim();
  if (line.length() == 0) return;

  long vals[5];
  int start = 0;
  for (int i = 0; i < 5; i++) {
    int comma = line.indexOf(',', start);
    String part;
    if (comma == -1) {
      part = line.substring(start);
      if (i < 4) return;
    } else {
      part = line.substring(start, comma);
    }
    part.trim();
    vals[i] = part.toInt();
    start = comma + 1;
  }

  cmd.throttle_us = clampUS(vals[0]);
  cmd.roll_us     = clampUS(vals[1]);
  cmd.pitch_us    = clampUS(vals[2]);
  cmd.yaw_us      = clampUS(vals[3]);
  cmd.armed       = vals[4] ? 1 : 0;
  if (!cmd.armed) cmd.throttle_us = 1000;

#if DEBUG_CMD_PRINT
  Serial.print("CMD ");
  Serial.print(cmd.throttle_us); Serial.print(',');
  Serial.print(cmd.roll_us); Serial.print(',');
  Serial.print(cmd.pitch_us); Serial.print(',');
  Serial.print(cmd.yaw_us); Serial.print(',');
  Serial.println(cmd.armed);
#endif
}

void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  // Quiet on purpose. Uncomment for debugging.
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "ESP-NOW OK" : "ESP-NOW FAIL");
}

// Drone -> PC: decode TelemetryPacket and print yaw as "H<float>\n"
// The Python backend (api/index.py) reads this in its heading reader thread.
void OnDataRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
  if (len != sizeof(TelemetryPacket)) return;
  TelemetryPacket t;
  memcpy(&t, data, sizeof(t));
  float yaw_rad = (float)t.yaw_centirad / 10000.0f;
  Serial.print("H");
  Serial.println(yaw_rad, 4);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_CHANNEL);

  // Print our MAC so the user can paste it into drone_receiver_crsf_espnow.ino
  uint8_t staMac[6];
  esp_wifi_get_mac(WIFI_IF_STA, staMac);
  Serial.printf("[sender] STA MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                staMac[0], staMac[1], staMac[2],
                staMac[3], staMac[4], staMac[5]);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add ESP-NOW peer");
    return;
  }

  Serial.println("Transmitter ready: Python Serial -> ESP-NOW; ESP-NOW -> H<yaw>");
  Serial.println("Format: throttle,roll,pitch,yaw,armed");
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      parseSerialCommand(inputLine);
      inputLine = "";
    } else if (c != '\r') {
      inputLine += c;
      if (inputLine.length() > 80) inputLine = "";
    }
  }

  if (millis() - lastSend >= SEND_PERIOD_MS) {
    lastSend = millis();
    cmd.seq++;
    esp_now_send(receiverAddress, (uint8_t *)&cmd, sizeof(cmd));
  }
}
