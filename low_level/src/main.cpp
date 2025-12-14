#include <Arduino.h>

// ====== SỬA CHÂN CHO ĐÚNG WIRING ======
static const int PIN_PWMA = 25;   // PWMA
static const int PIN_AIN1 = 26;   // AIN1
static const int PIN_AIN2 = 27;   // AIN2
static const int PIN_STBY = 14;   // STBY

static void printPins() {
  Serial.printf("STBY=%d AIN1=%d AIN2=%d PWMA=%d\n",
    digitalRead(PIN_STBY),
    digitalRead(PIN_AIN1),
    digitalRead(PIN_AIN2),
    digitalRead(PIN_PWMA)
  );
}

static void coast() {
  digitalWrite(PIN_PWMA, LOW);
  digitalWrite(PIN_AIN1, LOW);
  digitalWrite(PIN_AIN2, LOW);
}

static void forward_full_no_pwm() {
  digitalWrite(PIN_STBY, HIGH);
  digitalWrite(PIN_AIN1, HIGH);
  digitalWrite(PIN_AIN2, LOW);
  digitalWrite(PIN_PWMA, HIGH);   // FULL (không PWM)
}

static void reverse_full_no_pwm() {
  digitalWrite(PIN_STBY, HIGH);
  digitalWrite(PIN_AIN1, LOW);
  digitalWrite(PIN_AIN2, HIGH);
  digitalWrite(PIN_PWMA, HIGH);   // FULL (không PWM)
}

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(PIN_PWMA, OUTPUT);
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);

  digitalWrite(PIN_STBY, LOW);
  coast();

  Serial.println("TB6612 NO-PWM FULL TEST");
  Serial.println("Wiring must be: VM, VCC, GND common, STBY, AIN1, AIN2, PWMA, A01/A02->motor");
}

void loop() {
  Serial.println("\n[1] Forward FULL (PWMA=HIGH) 3s");
  forward_full_no_pwm();
  printPins();
  delay(3000);

  Serial.println("[2] Coast 1s");
  coast();
  printPins();
  delay(1000);

  Serial.println("[3] Reverse FULL (PWMA=HIGH) 3s");
  reverse_full_no_pwm();
  printPins();
  delay(3000);

  Serial.println("[4] Coast 1s");
  coast();
  printPins();
  delay(1000);
}
