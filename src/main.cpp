#include <Arduino.h>
#include <Wire.h>
#include "A4988.h"

// ─── MÉCANIQUE ────────────────────────────────────────────────────────────────
const float L = 1.15;
const float LAMBDA = 1.15;
const float R = 2.4;
const int STEPS_PER_REV = 200;
const int MICROSTEPPING = 16;
const float NB_PAS_PAR_CM = (STEPS_PER_REV * MICROSTEPPING) / (2 * PI * R);

// ─── POSITION ROBOT ───────────────────────────────────────────────────────────
float x_robot = 0.0;
float y_robot = 0.0;
float angle_robot = 0.0;  // vient exclusivement de l'IMU (rad)

// ─── IMU ──────────────────────────────────────────────────────────────────────
float RateYaw = 0.0;
float RateCalibrationYaw = 0.0;
unsigned long imu_prev_time = 0;
unsigned long lastPosUpdate = 0;
const unsigned long POS_INTERVAL = 100; // toutes les 100ms


void gyro_signals() {
    Wire.beginTransmission(0x68);
    Wire.write(0x1A); Wire.write(0x05); Wire.endTransmission();
    Wire.beginTransmission(0x68);
    Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission();
    Wire.beginTransmission(0x68);
    Wire.write(0x43); Wire.endTransmission();
    Wire.requestFrom((int)0x68, (int)6);
    Wire.read(); Wire.read(); // GyroX ignoré
    Wire.read(); Wire.read(); // GyroY ignoré
    int16_t GyroZ = Wire.read() << 8 | Wire.read();
    RateYaw = (float)GyroZ / 65.5f;  // °/s
}

void imu_calibrate(int samples = 2000) {
    Serial.println("Calibration IMU... ne bougez pas !");
    RateCalibrationYaw = 0;
    for (int i = 0; i < samples; i++) {
        gyro_signals();
        RateCalibrationYaw += RateYaw;
        delay(1);
    }
    RateCalibrationYaw /= samples;
    imu_prev_time = micros();
    Serial.println("Calibration IMU terminee !");
}

void imu_update() {
    gyro_signals();
    RateYaw -= RateCalibrationYaw;

    unsigned long now = micros();
    float dt = (now - imu_prev_time) / 1000000.0f;
    imu_prev_time = now;

    angle_robot += (RateYaw * PI / 180.0f) * dt;
    angle_robot = atan2(sin(angle_robot), cos(angle_robot));
    
}

// ─── MOTEURS ──────────────────────────────────────────────────────────────────
A4988 step_1(STEPS_PER_REV, D2,  D3);
A4988 step_2(STEPS_PER_REV, D6,  D7);
A4988 step_3(STEPS_PER_REV, D8,  D9);
A4988 step_4(STEPS_PER_REV, D10, D11);

long calcul_steps(float distance_cm) {
    return (long)(distance_cm * NB_PAS_PAR_CM);
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
        imu_update();  // ← IMU intégré pendant chaque mouvement
    }
}

// ─── MOUVEMENTS ───────────────────────────────────────────────────────────────
void avancer(float distance) {
    long s = calcul_steps(distance);
    sync_4_driver(s, -s, s, -s);
    x_robot += distance * cos(angle_robot);
    y_robot += distance * sin(angle_robot);
}

void reculer(float distance) { avancer(-distance); }

void gauche(float distance) {
    long s = calcul_steps(distance);
    sync_4_driver(-s, -s, s, s);
    x_robot -= distance * sin(angle_robot);
    y_robot += distance * cos(angle_robot);
}

void droite(float distance) { gauche(-distance); }

void rotation_gauche(float angle_deg) {
    float arc = (angle_deg * PI / 180.0f) * (L + LAMBDA);
    long s = calcul_steps(arc);
    sync_4_driver(-s, -s, -s, -s);
    // angle_robot mis à jour par imu_update() dans sync_4_driver
}

void rotation_droite(float angle_deg) {
    float arc = (angle_deg * PI / 180.0f) * (L + LAMBDA);
    long s = calcul_steps(arc);
    sync_4_driver(s, s, s, s);
    // angle_robot mis à jour par imu_update() dans sync_4_driver
}

void diagonale_droite(float distance) {
    long s = calcul_steps(distance);
    sync_4_driver(s, 0, 0, -s);
    x_robot += distance * cos(angle_robot + PI / 4.0f);
    y_robot += distance * sin(angle_robot + PI / 4.0f);
}

void diagonale_gauche(float distance) {
    long s = calcul_steps(distance);
    sync_4_driver(0, -s, s, 0);
    x_robot += distance * cos(angle_robot - PI / 4.0f);
    y_robot += distance * sin(angle_robot - PI / 4.0f);
}

// ─── NAVIGATION ───────────────────────────────────────────────────────────────
void envoyer_position() {
    Serial.print("POS ");
    Serial.print(x_robot, 2);
    Serial.print(" ");
    Serial.print(y_robot, 2);
    Serial.print(" ");
    Serial.println(angle_robot * 180.0f / PI, 2);
}

void go_to_coord(float x_cible, float y_cible) {
    float dx = x_cible - x_robot;
    float dy = y_cible - y_robot;
    float diff = atan2(sin(atan2(dy, dx) - angle_robot), cos(atan2(dy, dx) - angle_robot));

    if (diff > 0.01f)       rotation_gauche(diff * 180.0f / PI);
    else if (diff < -0.01f) rotation_droite(-diff * 180.0f / PI);

    avancer(sqrt(dx * dx + dy * dy));
}

void tourner_vers_angle(float angle_cible_deg) {
    float angle_cible_rad = angle_cible_deg * PI / 180.0f;
    float diff = atan2(sin(angle_cible_rad - angle_robot), cos(angle_cible_rad - angle_robot));

    if (diff > 0.01f)       rotation_gauche(diff * 180.0f / PI);
    else if (diff < -0.01f) rotation_droite(-diff * 180.0f / PI);
}

// ─── COMMANDES SÉRIE ──────────────────────────────────────────────────────────
void traiter_commande(String cmd) {
    cmd.trim();
    Serial.print("[STM32] CMD reçue: [");
    Serial.print(cmd);
    Serial.println("]");

    if      (cmd.startsWith("AC"))  { int sep = cmd.indexOf(' ', 3); go_to_coord(cmd.substring(3, sep).toFloat(), cmd.substring(sep + 1).toFloat()); }
    else if (cmd.startsWith("TVA")) { tourner_vers_angle(cmd.substring(4).toFloat()); }
    else if (cmd.startsWith("RAH")) { rotation_gauche(cmd.substring(4).toFloat()); }
    else if (cmd.startsWith("RH"))  { rotation_gauche(cmd.substring(3).toFloat()); }
    else if (cmd.startsWith("DG"))  { diagonale_gauche(cmd.substring(3).toFloat()); }
    else if (cmd.startsWith("DD"))  { diagonale_droite(cmd.substring(3).toFloat()); }
    else if (cmd.startsWith("A"))   { avancer(cmd.substring(2).toFloat()); }
    else if (cmd.startsWith("R"))   { reculer(cmd.substring(2).toFloat()); }
    else if (cmd.startsWith("D"))   { droite(cmd.substring(2).toFloat()); }
    else if (cmd.startsWith("G"))   { gauche(cmd.substring(2).toFloat()); }

    envoyer_position();
}

// ─── SETUP / LOOP ─────────────────────────────────────────────────────────────
unsigned long lastHeartbeat = 0;
const unsigned long HEARTBEAT_INTERVAL = 1000;

void setup() {
    Serial.begin(115200);
    while (!Serial);
    delay(500);

    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);

    Wire.setSDA(PB9);
    Wire.setSCL(PB8);
    Wire.setClock(400000);
    Wire.begin();
    delay(250);
    Wire.beginTransmission(0x68);
    Wire.write(0x6B); Wire.write(0x00);
    Wire.endTransmission();

    imu_calibrate();

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
    imu_update();

    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        traiter_commande(cmd);
    }

    unsigned long now = millis();
    
    if (now - lastPosUpdate >= POS_INTERVAL) {
        envoyer_position();
        lastPosUpdate = now;
    }
}
