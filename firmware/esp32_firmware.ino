// ===== Rover ESP32: motors + gas + IMU (MPU6050, baseline-calibrated) =====
// ENA/ENB hardwired to 3.3V (full speed). PWM speed control later.
// IMU captures resting orientation at startup; detects tilt relative to it.
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// Motor driver (L298N) - control pins only
#define IN1 27
#define IN2 26
#define IN3 25
#define IN4 33

// Gas sensor
#define MQ2_DO 4

// IMU
Adafruit_MPU6050 mpu;
bool imuOK = false;
float basePitch = 0, baseRoll = 0;          // resting orientation baseline
const float TILT_THRESHOLD = 20.0;          // degrees from resting = unstable

unsigned long lastReport = 0;
const unsigned long REPORT_INTERVAL = 500;  // ms

void computeAngles(float &pitch, float &roll) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  pitch = atan2(a.acceleration.x,
          sqrt(a.acceleration.y*a.acceleration.y + a.acceleration.z*a.acceleration.z)) * 180.0/PI;
  roll  = atan2(a.acceleration.y, a.acceleration.z) * 180.0/PI;
}

void setup() {
  Serial.begin(9600);
  Wire.begin(21, 22);   // SDA=21, SCL=22

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(MQ2_DO, INPUT);
  stop();

  // IMU init + baseline capture (keep rover level & still at power-on)
  if (mpu.begin()) {
    imuOK = true;
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    float p, r, sumP = 0, sumR = 0;
    for (int i = 0; i < 20; i++) {
      computeAngles(p, r);
      sumP += p; sumR += r;
      delay(50);
    }
    basePitch = sumP / 20.0;
    baseRoll  = sumR / 20.0;
    Serial.println("MPU6050 ready (baseline captured)");
  } else {
    Serial.println("MPU6050 NOT FOUND");
  }

  Serial.println("Rover ready: motors + gas + IMU");
}

void loop() {
  // 1. Motor commands from Pi
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    switch (cmd) {
      case 'F': move_forward();  break;
      case 'B': move_backward(); break;
      case 'L': turn_left();     break;
      case 'R': turn_right();    break;
      case 'S': stop();          break;
    }
  }

  // 2. Periodic sensor report
  unsigned long now = millis();
  if (now - lastReport >= REPORT_INTERVAL) {
    lastReport = now;

    // --- Gas ---
    int gas = digitalRead(MQ2_DO);
    Serial.println(gas == LOW ? "GAS:1" : "GAS:0");

    // --- IMU tilt (relative to resting baseline) ---
    if (imuOK) {
      float pitch, roll;
      computeAngles(pitch, roll);
      float dPitch = pitch - basePitch;
      float dRoll  = roll  - baseRoll;

      Serial.print("TILT:");
      Serial.print(dPitch, 1);
      Serial.print(",");
      Serial.println(dRoll, 1);

      if (abs(dPitch) > TILT_THRESHOLD || abs(dRoll) > TILT_THRESHOLD) {
        Serial.println("UNSTABLE:1");
      } else {
        Serial.println("UNSTABLE:0");
      }
    }
  }
}

// ===== Motor functions (confirmed-working directions) =====
void move_forward() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}
void move_backward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
}
void turn_left() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}
void turn_right() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
}
void stop() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}
