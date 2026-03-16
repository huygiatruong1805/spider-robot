#include <WiFi.h>
#include <esp_now.h>

/* =========================================================
   ROBOT MAC (ESP32-C3)
   ========================================================= */

uint8_t robotMAC[] = {0x58, 0x8C, 0x81, 0xA4, 0x04, 0xD4};


/* =========================================================
   BUTTON
   ========================================================= */

#define BUTTON_PIN 4


/* =========================================================
   COMMAND
   ========================================================= */

#define CMD_TOGGLE_STATE 1

typedef struct {
  uint8_t cmd;
} ControlPacket;

ControlPacket packet;


/* =========================================================
   DEBOUNCE
   ========================================================= */

bool lastButtonState = HIGH;
unsigned long lastPressTime = 0;
const int debounceDelay = 200;


/* =========================================================
   ESP-NOW SEND CALLBACK (IDF 5.x)
   ========================================================= */

void onSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {

  Serial.print("Send Status: ");

  if (status == ESP_NOW_SEND_SUCCESS)
    Serial.println("SUCCESS");
  else
    Serial.println("FAIL");
}


/* =========================================================
   SETUP
   ========================================================= */

void setup() {

  Serial.begin(115200);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  Serial.print("Remote MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {

    Serial.println("ESP-NOW INIT FAILED");
    return;
  }

  esp_now_register_send_cb(onSent);

  esp_now_peer_info_t peerInfo = {};

  memcpy(peerInfo.peer_addr, robotMAC, 6);

  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {

    Serial.println("Add peer failed");
    return;
  }

  packet.cmd = CMD_TOGGLE_STATE;
}


/* =========================================================
   LOOP
   ========================================================= */

void loop() {

  bool reading = digitalRead(BUTTON_PIN);

  if (reading == LOW && lastButtonState == HIGH) {

    if (millis() - lastPressTime > debounceDelay) {

      lastPressTime = millis();

      esp_now_send(robotMAC, (uint8_t *)&packet, sizeof(packet));

      Serial.println("Command sent");
    }
  }

  lastButtonState = reading;
}