#include <Arduino.h>
#include "A4988.h"

const float L = 1.15;     
const float LAMBDA = 1.15;
const float R = 2.4;      
const int STEPS_PER_REV = 200;
const int MICROSTEPPING = 16;
const float NB_PAS_PAR_CM = (STEPS_PER_REV * MICROSTEPPING) / (2 * PI * R);

float x_robot = 0.0;
float y_robot = 0.0;
float angle_robot = 0.0;

A4988 step_1(STEPS_PER_REV, D2,  D3);
A4988 step_2(STEPS_PER_REV, D6,  D7);
A4988 step_3(STEPS_PER_REV, D8,  D9);
A4988 step_4(STEPS_PER_REV, D10, D11);

long calcul_steps(float distance_cm) {
  return (long)(distance_cm * NB_PAS_PAR_CM);
}

void envoyer_position() {
  Serial.print("POS ");
  Serial.print(x_robot, 2);
  Serial.print(" ");
  Serial.print(y_robot, 2);
  Serial.print(" ");
  Serial.println(angle_robot * 180.0 / PI, 2);
}

void sync_4_driver(long s1, long s2, long s3, long s4) {
    step_1.startMove(s1);
    step_2.startMove(s2);
    step_3.startMove(s3);
    step_4.startMove(s4);

    while (step_1.getStepsRemaining() || step_2.getStepsRemaining() ||
           step_3.getStepsRemaining() || step_4.getStepsRemaining()) {
        step_1.nextAction();
        step_2.nextAction();
        step_3.nextAction();
        step_4.nextAction();
    }
}

void avancer(float distance) {
    long s = calcul_steps(distance);
    sync_4_driver(s, -s, s, -s);
    x_robot += distance * cos(angle_robot);
    y_robot += distance * sin(angle_robot);
}

void reculer(float distance) {
    avancer(-distance);
}

void gauche(float distance) {
    long s = calcul_steps(distance);
    sync_4_driver(-s, -s, s, s);
    x_robot -= distance * sin(angle_robot);
    y_robot += distance * cos(angle_robot);
}

void droite(float distance) {
    gauche(-distance);
}

void rotation_gauche(float angle_deg) {
    float angle_rad = angle_deg * PI / 180.0;
    float arc = angle_rad * (L + LAMBDA);
    long s = calcul_steps(arc);
    sync_4_driver(-s, -s, -s, -s);
    angle_robot += angle_rad;
    angle_robot = atan2(sin(angle_robot), cos(angle_robot));
}

void rotation_droite(float angle_deg) {
    float angle_rad = angle_deg * PI / 180.0;
    float arc = angle_rad * (L + LAMBDA);
    long s = calcul_steps(arc);
    sync_4_driver(s, s, s, s);
    angle_robot -= angle_rad;
    angle_robot = atan2(sin(angle_robot), cos(angle_robot));
}

void diagonale_droite(float distance) {
    long s = calcul_steps(distance);
    sync_4_driver(s, 0, 0, -s);
    x_robot += distance * cos(angle_robot + PI / 4.0);
    y_robot += distance * sin(angle_robot + PI / 4.0);
}

void diagonale_gauche(float distance) {
    long s = calcul_steps(distance);
    sync_4_driver(0, -s, s, 0);
    x_robot += distance * cos(angle_robot - PI / 4.0);
    y_robot += distance * sin(angle_robot - PI / 4.0);
}

void go_to_coord(float x_cible, float y_cible) {
    float dx = x_cible - x_robot;
    float dy = y_cible - y_robot;
    float angle_cible = atan2(dy, dx);

    float diff = angle_cible - angle_robot;
    diff = atan2(sin(diff), cos(diff));

    if (diff > 0.01) {
        rotation_gauche(diff * 180.0 / PI);
    } else if (diff < -0.01) {
        rotation_droite(-diff * 180.0 / PI);
    }

    float distance = sqrt(dx * dx + dy * dy);
    avancer(distance);
}

void tourner_vers_angle(float angle_cible_deg) {
    float angle_cible_rad = angle_cible_deg * PI / 180.0;
    float diff = angle_cible_rad - angle_robot;
    diff = atan2(sin(diff), cos(diff));

    if (diff > 0.01) {
        rotation_gauche(diff * 180.0 / PI);
    } else if (diff < -0.01) {
        rotation_droite(-diff * 180.0 / PI);
    }
}

void traiter_commande(String cmd) {
  cmd.trim();

  Serial.print("[STM32] CMD reçue: [");
  Serial.print(cmd);
  Serial.println("]");

  if (cmd.startsWith("AC")) {
    int sep = cmd.indexOf(' ', 3);
    float x_cm = cmd.substring(3, sep).toFloat(); 
    float y_cm = cmd.substring(sep + 1).toFloat();
    go_to_coord(x_cm, y_cm);

  } else if (cmd.startsWith("TVA")) {
    float angle_deg = cmd.substring(4).toFloat();
    tourner_vers_angle(angle_deg);

  } else if (cmd.startsWith("RAH")) {
    float d = cmd.substring(4).toFloat();
    rotation_gauche(d);

  } else if (cmd.startsWith("RH")) {
    float d = cmd.substring(3).toFloat();
    rotation_gauche(d);

  } else if (cmd.startsWith("DG")) {
    float d = cmd.substring(3).toFloat();  
    diagonale_gauche(d);

  } else if (cmd.startsWith("DD")) {
    float d = cmd.substring(3).toFloat();  
    diagonale_droite(d);

  } else if (cmd.startsWith("A")) {
    float d = cmd.substring(2).toFloat();  
    avancer(d);

  } else if (cmd.startsWith("R")) {
    float d = cmd.substring(2).toFloat();  
    reculer(d);

  } else if (cmd.startsWith("D")) {
    float d = cmd.substring(2).toFloat();  
    droite(d);

  } else if (cmd.startsWith("G")) {
    float d = cmd.substring(2).toFloat();  
    gauche(d);
  }

  envoyer_position();
}

void setup() {

  pinMode(LED, OUTPUT);

  Serial.begin(115200);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  step_1.begin(100, MICROSTEPPING);
  step_2.begin(100, MICROSTEPPING);
  step_3.begin(100, MICROSTEPPING);
  step_4.begin(100, MICROSTEPPING);

  step_1.setSpeedProfile(step_1.LINEAR_SPEED, 1000, 1000);
  step_2.setSpeedProfile(step_2.LINEAR_SPEED, 1000, 1000);
  step_3.setSpeedProfile(step_3.LINEAR_SPEED, 1000, 1000);
  step_4.setSpeedProfile(step_4.LINEAR_SPEED, 1000, 1000);

  Serial.println("Pret !");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    traiter_commande(cmd);
  }
  
  digitalWrite(LED, start ? HIGH : LOW);

  unsigned long currentTime = millis();
  if (currentTime - lastHeartbeat >= HEARTBEAT_INTERVAL) {
    Serial.print("OK\n");
    lastHeartbeat = currentTime;
  }
}
