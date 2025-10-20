#include "WiFi.h"
#include <esp_now.h>

#define RELAY8 18
#define RELAY7 19
#define RELAY6 21
#define RELAY5 27
#define RELAY4 26
#define RELAY3 25
#define RELAY2 33
#define RELAY1 32

// Control Buttons 
// 5 Inch ESP32: 30:30:F9:57:61:D8
uint8_t broadcastAddress1[] = {0x30, 0x30, 0xF9, 0x57, 0x61, 0xD8};
esp_now_peer_info_t peerInfo;

typedef struct espNowStruct {
    int16_t type;
    int16_t id;
    int16_t value;
} espNowStruct;

espNowStruct espNowPacket;

void informStatus(int16_t id, int16_t value) {
  espNowPacket.type = 2; //Buttons
  espNowPacket.id = id;
  espNowPacket.value = value; 
  esp_err_t result = esp_now_send(0, (uint8_t *) &espNowPacket, sizeof(espNowStruct));
}

void informStatusAll() {
  informStatus(1, digitalRead(RELAY1));
  informStatus(2, digitalRead(RELAY2));
  informStatus(3, digitalRead(RELAY3));
  informStatus(4, digitalRead(RELAY4));
  informStatus(5, digitalRead(RELAY5));
  informStatus(6, digitalRead(RELAY6));
  informStatus(7, digitalRead(RELAY7));
  informStatus(8, digitalRead(RELAY8));
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {

  memcpy(&espNowPacket, incomingData, sizeof(espNowPacket));
  Serial.print("Bytes received: ");
  Serial.println(len);
  int16_t type = espNowPacket.type;
  int16_t id = espNowPacket.id;
  int16_t value = espNowPacket.value;

  if (type == 2) { // Buttons
    switch (id) {
      case 0: informStatusAll(); break;
      case 1: digitalWrite(RELAY1, value); informStatus(id, value); break;
      case 2: digitalWrite(RELAY2, value); informStatus(id, value); break;
      case 3: digitalWrite(RELAY3, value); informStatus(id, value); break;
      case 4: digitalWrite(RELAY4, value); informStatus(id, value); break;
      case 5: digitalWrite(RELAY5, value); informStatus(id, value); break;
      case 6: digitalWrite(RELAY6, value); informStatus(id, value); break;
      case 7: digitalWrite(RELAY7, value); informStatus(id, value); break;
      case 8: digitalWrite(RELAY8, value); informStatus(id, value); break;
    }
  }
}

/*
// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  Serial.print("Packet to: ");
  // Copies the sender mac address to a string
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
  Serial.print(" send status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}*/
 
void setup() {
  Serial.println("Initializing...");

  // Serial port for debugging purposes
  Serial.begin(115200);
  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);
  pinMode(RELAY3, OUTPUT);
  pinMode(RELAY4, OUTPUT);
  pinMode(RELAY5, OUTPUT);
  pinMode(RELAY6, OUTPUT);
  pinMode(RELAY7, OUTPUT);
  pinMode(RELAY8, OUTPUT);

  digitalWrite(RELAY1, LOW);
  digitalWrite(RELAY2, LOW);
  digitalWrite(RELAY3, LOW);
  digitalWrite(RELAY4, LOW);
  digitalWrite(RELAY5, LOW);
  digitalWrite(RELAY6, LOW);
  digitalWrite(RELAY7, LOW);
  digitalWrite(RELAY8, LOW);

  // Get Mac Address
  //WiFi.mode(WIFI_MODE_STA);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  Serial.print("MAC address: ");
  Serial.println(WiFi.macAddress());

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  //esp_now_register_send_cb(OnDataSent);

  // register peer
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  // register first peer  
  memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

  informStatusAll();

  Serial.println("Done");
}
 
void loop(){
  delay(10);
}
