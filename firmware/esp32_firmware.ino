// ===== Rover ESP32: motor control + gas sensor =====

// Motor driver (L298N)
#define ENA 14
#define IN1 27
#define IN2 26
#define IN3 25
#define IN4 33
#define ENB 32

// Gas sensor
#define MQ2_DO 4

unsigned long lastGasReport = 0;
const unsigned long GAS_INTERVAL = 500;  // ms

void setup() {
  Serial.begin(9600);

  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  digitalWrite(ENA, HIGH);   // full speed (PWM later)
  digitalWrite(ENB, HIGH);

  pinMode(MQ2_DO, INPUT);

  stop();
  Serial.println("Rover ready: motors + gas");
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

  // 2. Periodic gas report
  unsigned long now = millis();
  if (now - lastGasReport >= GAS_INTERVAL) {
    lastGasReport = now;
    int gas = digitalRead(MQ2_DO);
    if (gas == LOW) {
      Serial.println("GAS:1");   // LOW = gas detected on most modules
    } else {
      Serial.println("GAS:0");
    }
  }
}

// ===== Motor functions =====
void turn_right() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}
void turn_left() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
}
void move_backward() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}
void move_forward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
}
void stop() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}
