#include <Arduino.h>

// Configuration
int LED = 13;
bool start = false;
unsigned long lastHeartbeat = 0;
const unsigned long HEARTBEAT_INTERVAL = 1000; 

String readString;
String vx;
String vy;
String omega;
int ind1; // , locations
int ind2;
int ind3;
int ind4;

void establishContact();
void sendHeartbeat();

void setup() {

  pinMode(LED, OUTPUT);

  Serial.begin(115200);

}

void loop() {
  // Read incoming serial data character by character
  while (Serial.available()) {
    char c = Serial.read();
    readString += c;
    
    if (c == '\n') {
      // Parse the command: "vx,vy,omega\n"
      ind1 = readString.indexOf(',');
      vx = readString.substring(0, ind1);
      ind2 = readString.indexOf(',', ind1 + 1);
      vy = readString.substring(ind1 + 1, ind2);
      ind3 = readString.indexOf('\n', ind2 + 1);
      omega = readString.substring(ind2 + 1, ind3);
      
      Serial.print("vx: ");
      Serial.print(vx);
      Serial.print(" vy: ");
      Serial.print(vy);
      Serial.print(" omega: ");
      Serial.println(omega);
      
      readString = "";
    }
  }
  
  digitalWrite(LED, start ? HIGH : LOW);

  unsigned long currentTime = millis();
  if (currentTime - lastHeartbeat >= HEARTBEAT_INTERVAL) {
    Serial.print("OK\n");
    lastHeartbeat = currentTime;
  }
}
