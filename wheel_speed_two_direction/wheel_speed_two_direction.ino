#include <DueTimer.h>

volatile double motor_speed = 0;
volatile long last_time = 0;
volatile long this_time = 0;
volatile bool rising_trend = true;
volatile int A_trend_first = 1;
double last_motor_speed = 0;

void enA_rise_ISR1() {
  this_time = micros();
  motor_speed = double(281.846 / (this_time - last_time));
  last_time = this_time;
  if (!rising_trend) {
    rising_trend = true;
    A_trend_first = 1;
  }
}

void enB_rise_ISR1() {
  this_time = micros();
  motor_speed = double(281.846 / (this_time - last_time));
  last_time = this_time;
  if (!rising_trend) {
    rising_trend = true;
    A_trend_first = -1;
  }
}

void enA_fall_ISR1() {
  this_time = micros();
  motor_speed = double(281.846 / (this_time - last_time));
  last_time = this_time;
  if (rising_trend) {
    rising_trend = false;
    A_trend_first = 1;
  }
}

void enB_fall_ISR1() {
  this_time = micros();
  motor_speed = double(281.846 / (this_time - last_time));
  last_time = this_time;
  if (rising_trend) {
    rising_trend = false;
    A_trend_first = -1;
  }
}

void timeOut1Handler() {
  if (micros() - last_time > 5000) motor_speed = 0;
}

volatile double motor_speed2 = 0;
volatile long last_time2 = 0;
volatile long this_time2 = 0;
volatile bool rising_trend2 = true;
volatile int A_trend_first2 = 1;
double last_motor_speed2 = 0;

void enA_rise_ISR2() {
  this_time2 = micros();
  motor_speed2 = double(281.846 / (this_time2 - last_time2));
  last_time2 = this_time2;
  if (!rising_trend2) {
    rising_trend2 = true;
    A_trend_first2 = 1;
  }
}

void enB_rise_ISR2() {
  this_time2 = micros();
  motor_speed2 = double(281.846 / (this_time2 - last_time2));
  last_time2 = this_time2;
  if (!rising_trend2) {
    rising_trend2 = true;
    A_trend_first2 = -1;
  }
}

void enA_fall_ISR2() {
  this_time2 = micros();
  motor_speed2 = double(281.846 / (this_time2 - last_time2));
  last_time2 = this_time2;
  if (rising_trend2) {
    rising_trend2 = false;
    A_trend_first2 = 1;
  }
}

void enB_fall_ISR2() {
  this_time2 = micros();
  motor_speed2 = double(281.846 / (this_time2 - last_time2));
  last_time2 = this_time2;
  if (rising_trend2) {
    rising_trend2 = false;
    A_trend_first2 = -1;
  }
}

void timeOut2Handler() {
  if (micros() - last_time2 > 5000) motor_speed2 = 0;
}

void setup() {
  Serial.begin(115200);
  pinMode(5, OUTPUT);
  Timer.getAvailable().attachInterrupt(timeOut1Handler).start(1000);
  attachInterrupt(digitalPinToInterrupt(50), enA_rise_ISR1, RISING);
  attachInterrupt(digitalPinToInterrupt(51), enA_fall_ISR1, FALLING);
  attachInterrupt(digitalPinToInterrupt(52), enB_rise_ISR1, RISING);
  attachInterrupt(digitalPinToInterrupt(53), enB_fall_ISR1, FALLING);
  Timer.getAvailable().attachInterrupt(timeOut2Handler).start(1000);
  attachInterrupt(digitalPinToInterrupt(22), enA_rise_ISR2, RISING);
  attachInterrupt(digitalPinToInterrupt(23), enA_fall_ISR2, FALLING);
  attachInterrupt(digitalPinToInterrupt(24), enB_rise_ISR2, RISING);
  attachInterrupt(digitalPinToInterrupt(25), enB_fall_ISR2, FALLING);
}

void loop() {
  digitalWrite(5, LOW);
  digitalWrite(5, HIGH);
  Serial.print(motor_speed * A_trend_first);
  Serial.print(' ');
  Serial.println(motor_speed2 * A_trend_first2);
}
