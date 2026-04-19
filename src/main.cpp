#include <Arduino.h>
#include <Wire.h>
#include "imu.h"
#include "motor.h"

void envoyer_position() {
  Serial.print("POS ");
  Serial.print(x_robot, 2);
  Serial.print(" ");
  Serial.print(y_robot, 2);
  Serial.print(" ");
  Serial.println(AngleYaw, 2);
}

void traiter_commande(String cmd) {
  cmd.trim();
  if (cmd.startsWith("AC")) {
    float x, y; sscanf(cmd.c_str(), "AC %f %f", &x, &y);
    aller_a_coord(x / 100.0, y / 100.0);
  } else if (cmd.startsWith("TVA")) {
    float a; sscanf(cmd.c_str(), "TVA %f", &a);
    tourner_vers_angle(a);
  } else if (cmd.startsWith("A")) {
    float d; sscanf(cmd.c_str(), "A %f", &d); avancer(d);
  } else if (cmd.startsWith("R")) {
    float d; sscanf(cmd.c_str(), "R %f", &d); reculer(d);
  } else if (cmd.startsWith("D")) {
    float d; sscanf(cmd.c_str(), "D %f", &d); droite(d);
  } else if (cmd.startsWith("G")) {
    float d; sscanf(cmd.c_str(), "G %f", &d); gauche(d);
  } else if (cmd.startsWith("DG")) {
    float d; sscanf(cmd.c_str(), "DG %f", &d); diagonale_gauche(d);
  } else if (cmd.startsWith("DD")) {
    float d; sscanf(cmd.c_str(), "DD %f", &d); diagonale_droite(d);
  } else if (cmd.startsWith("RH")) {
    float d; sscanf(cmd.c_str(), "RH %f", &d);
    rotation_gauche(d);
  } else if (cmd.startsWith("RAH")) {
    float d; sscanf(cmd.c_str(), "RAH %f", &d);
    rotation_droite(d);
  }
  envoyer_position();
}

void setup() {
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  init_imu();
  init_motors();
  Serial.println("Pret !");
}

void loop() {
  update_imu();

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    traiter_commande(cmd);
  }

  static unsigned long last_send = 0;
  if (millis() - last_send > 50) {
    envoyer_position();
    last_send = millis();
  }
}