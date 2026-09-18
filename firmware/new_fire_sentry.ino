#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include "esp_bt.h"

namespace Config {
  constexpr int PIN_SENSOR_FRONT = 32;
  constexpr int PIN_SENSOR_RIGHT = 33;
  constexpr int PIN_SENSOR_BACK  = 34;
  constexpr int PIN_SENSOR_LEFT  = 35;
  constexpr int PIN_PAN  = 26;
  constexpr int PIN_TILT = 25;
  constexpr int PIN_PUMP = 19;

  constexpr bool FIRE_IS_HIGHER = false;  //read LOWER on fire
  constexpr int FIRE_ON_THRESHOLD  = 600; // start reacting above this
  constexpr int FIRE_OFF_THRESHOLD = 450; // stop reacting below this (hysteresis)

  constexpr float SENSOR_SMOOTH = 0.30f;

  constexpr int   PAN_MIN = 0;
  constexpr int   PAN_MAX = 180;
  constexpr float PAN_CENTER = 90.0f;

  constexpr int   TILT_MIN = 0;
  constexpr int   TILT_MAX = 180;
  constexpr float TILT_STANDBY  = 90.0f;  // rest: hose straight up
  constexpr float TILT_FORWARD  = 55.0f;  // aim a fire in FRONT
  constexpr float TILT_BACKWARD = 125.0f; // flip over top, aim a fire BEHIND

  constexpr float AZI_HYSTERESIS = 12.0f;

  constexpr float PAN_SMOOTH    = 0.15f;
  constexpr float TILT_SMOOTH   = 0.12f;
  constexpr float RETURN_SMOOTH = 0.05f;
  constexpr float PAN_DEADBAND  = 2.0f;
  constexpr float TILT_DEADBAND = 2.0f;

  constexpr float AIM_TOL      = 5.0f;    // pan degrees from target
  constexpr float TILT_AIM_TOL = 6.0f;    // tilt degrees from target
  constexpr unsigned long AIM_HOLD_MS = 300;

  //   true  = HIGH turns pump ON
  //   false = LOW  turns pump ON
  constexpr bool PUMP_ACTIVE_HIGH = false;  //relay LOW = ON

  constexpr unsigned long LOOP_INTERVAL_MS = 20;
}

enum Direction { DIR_FRONT = 0, DIR_RIGHT, DIR_LEFT, DIR_BACK, DIR_COUNT };

struct SensorArray {
  int         pin[DIR_COUNT];
  float       azimuth[DIR_COUNT];   // compass angle each sensor faces (deg)
  const char* name[DIR_COUNT];
  float       intensity[DIR_COUNT]; // smoothed + normalised (higher = fire)
};
SensorArray sensors;

int normaliseReading(int raw) {
  return Config::FIRE_IS_HIGHER ? raw : (4095 - raw);
}

void sensorsInit() {
  sensors.pin[DIR_FRONT] = Config::PIN_SENSOR_FRONT;
  sensors.pin[DIR_RIGHT] = Config::PIN_SENSOR_RIGHT;
  sensors.pin[DIR_LEFT]  = Config::PIN_SENSOR_LEFT;
  sensors.pin[DIR_BACK]  = Config::PIN_SENSOR_BACK;

  // Azimuth: 0 = front, 90 = right, 180 = back, 270 = left.
  sensors.azimuth[DIR_FRONT] = 0.0f;
  sensors.azimuth[DIR_RIGHT] = 90.0f;
  sensors.azimuth[DIR_LEFT]  = 270.0f;
  sensors.azimuth[DIR_BACK]  = 180.0f;

  sensors.name[DIR_FRONT] = "FRONT";
  sensors.name[DIR_RIGHT] = "RIGHT";
  sensors.name[DIR_LEFT]  = "LEFT";
  sensors.name[DIR_BACK]  = "BACK";

  analogReadResolution(12);

  for (int i = 0; i < DIR_COUNT; i++) {
    sensors.intensity[i] = normaliseReading(analogRead(sensors.pin[i]));
  }
}

void sensorsUpdate() {
  for (int i = 0; i < DIR_COUNT; i++) {
    float reading = normaliseReading(analogRead(sensors.pin[i]));
    sensors.intensity[i] += (reading - sensors.intensity[i]) * Config::SENSOR_SMOOTH;
  }
}

bool  g_fire        = false;
float g_targetAzi   = 0.0f;   // signed azimuth
int   g_strongest   = -1;
bool  g_fireLatched = false;

void computeTarget() {
  float maxIntensity = 0.0f;
  g_strongest = -1;
  for (int i = 0; i < DIR_COUNT; i++) {
    if (sensors.intensity[i] > maxIntensity) {
      maxIntensity = sensors.intensity[i];
      g_strongest  = i;
    }
  }

  if (!g_fireLatched && maxIntensity >= Config::FIRE_ON_THRESHOLD)  g_fireLatched = true;
  if ( g_fireLatched && maxIntensity <  Config::FIRE_OFF_THRESHOLD) g_fireLatched = false;
  g_fire = g_fireLatched;
  if (!g_fire) return;

  float sinSum = 0.0f, cosSum = 0.0f;
  for (int i = 0; i < DIR_COUNT; i++) {
    if (sensors.intensity[i] >= Config::FIRE_OFF_THRESHOLD) {
      float r = radians(sensors.azimuth[i]);
      sinSum += sin(r) * sensors.intensity[i];
      cosSum += cos(r) * sensors.intensity[i];
    }
  }
  float azi = degrees(atan2(sinSum, cosSum));
  g_targetAzi = azi;
}

float g_targetPan  = Config::PAN_CENTER;
float g_targetTilt = Config::TILT_STANDBY;
bool  g_backMode   = false;

void updateKinematics() {
  float A  = g_targetAzi;
  float aa = fabs(A);

  if (!g_backMode && aa > 90.0f + Config::AZI_HYSTERESIS) g_backMode = true;
  if ( g_backMode && aa < 90.0f - Config::AZI_HYSTERESIS) g_backMode = false;

  float baseHeading; // the azimuth the BASE points at
  if (!g_backMode) {
    baseHeading  = A;
    g_targetTilt = Config::TILT_FORWARD;
  } else {
    baseHeading  = A - (A >= 0 ? 180.0f : -180.0f); // mirror to front range
    g_targetTilt = Config::TILT_BACKWARD;
  }

  // Base azimuth -> pan servo angle. 90 = front, 0 = hard right, 180 = hard left.
  g_targetPan = constrain(90.0f - baseHeading, (float)Config::PAN_MIN, (float)Config::PAN_MAX);
}

Servo panServo, tiltServo;
float g_panPos  = Config::PAN_CENTER;
float g_tiltPos = Config::TILT_STANDBY;

void panWrite(float a) {
  a = constrain(a, (float)Config::PAN_MIN, (float)Config::PAN_MAX);
  g_panPos = a; panServo.write((int)round(a));
}
void tiltWrite(float a) {
  a = constrain(a, (float)Config::TILT_MIN, (float)Config::TILT_MAX);
  g_tiltPos = a; tiltServo.write((int)round(a));
}
void panEaseTo(float target, float smooth) {
  if (fabs(target - g_panPos) <= Config::PAN_DEADBAND) return; // hold -> no drift
  panWrite(g_panPos + (target - g_panPos) * smooth);
}
void tiltEaseTo(float target, float smooth) {
  if (fabs(target - g_tiltPos) <= Config::TILT_DEADBAND) return;
  tiltWrite(g_tiltPos + (target - g_tiltPos) * smooth);
}

void actuatorsInit() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  panServo.attach(Config::PIN_PAN,  500, 2500);
  tiltServo.attach(Config::PIN_TILT, 500, 2500);
  panWrite(Config::PAN_CENTER);
  tiltWrite(Config::TILT_STANDBY);
  delay(400);
}

bool g_pumpOn = false;
void pumpSet(bool on) {
  g_pumpOn = on;
  bool level = Config::PUMP_ACTIVE_HIGH ? on : !on;
  digitalWrite(Config::PIN_PUMP, level ? HIGH : LOW);
}
void pumpInit() { pinMode(Config::PIN_PUMP, OUTPUT); pumpSet(false); }

unsigned long g_aimStart = 0;
bool          g_aiming   = false;

bool aimConfirmed() {
  bool panOK  = fabs(g_targetPan  - g_panPos)  <= Config::AIM_TOL;
  bool tiltOK = fabs(g_targetTilt - g_tiltPos) <= Config::TILT_AIM_TOL;
  if (!(panOK && tiltOK)) { g_aiming = false; return false; }
  if (!g_aiming) { g_aiming = true; g_aimStart = millis(); }
  return (millis() - g_aimStart) >= Config::AIM_HOLD_MS;
}

void debugPrint() {
  Serial.printf("F:%4.0f R:%4.0f B:%4.0f L:%4.0f | fire:%s",
    sensors.intensity[DIR_FRONT], sensors.intensity[DIR_RIGHT],
    sensors.intensity[DIR_BACK],  sensors.intensity[DIR_LEFT],
    g_fire ? "YES" : "no ");
  if (g_fire)
    Serial.printf(" | azi:%4.0f %-4s", g_targetAzi, g_backMode ? "BACK" : "FRNT");
  else
    Serial.printf(" |               ");
  Serial.printf(" | pan:%3.0f tilt:%3.0f | pump:%s\n",
    g_panPos, g_tiltPos, g_pumpOn ? "ON " : "off");
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_OFF);
  btStop();
  sensorsInit();
  actuatorsInit();
  pumpInit();
  Serial.println("=== Fire Sentry ready (pan-tilt 360) ===");
}

void loop() {
  static unsigned long last = 0;
  if (millis() - last < Config::LOOP_INTERVAL_MS) return;
  last = millis();

  sensorsUpdate();   // 2. read
  computeTarget();   // 3. is there fire, and at what azimuth?

  if (g_fire) {
    updateKinematics();                          // 4. azimuth -> pan + tilt
    panEaseTo(g_targetPan,  Config::PAN_SMOOTH);  // 5. move
    tiltEaseTo(g_targetTilt, Config::TILT_SMOOTH);
    pumpSet(aimConfirmed());                      // 6+7. pump only when locked
  } else {
    pumpSet(false);
    g_aiming = false;
    panEaseTo(Config::PAN_CENTER,  Config::RETURN_SMOOTH);
    tiltEaseTo(Config::TILT_STANDBY, Config::RETURN_SMOOTH);
  }

  debugPrint();
}
