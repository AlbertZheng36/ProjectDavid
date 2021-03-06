/* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
 */

#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter
#include <DueTimer.h>

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

// TODO: Make calibration routine

// Motor1 direction 12 and speed is 10;
// Motor2 direction 13 and speed is 11;

int enA = 10;
int dirA = 12;
int enB = 11;
int dirB = 13;
int test = 6;
double current_angle = 0;
double target_angle = 6;
int moving_factor = 0;
double Kp = 45;
double Kd = 270;
double Ki = 0.15;
int integrated_error;
int p1, p2;
double robot_angle;
double last_error = 0;
int control_decision = 0;
double error = 0;
double delta_error = 0;

//================================ don't see this section ========================//
// volatile interrupt data
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

// ========================================= don't see above section =======================================//


void setup() {
  Serial.begin(115200);
  pinMode(21, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  for (int i = 0; i < 8; i++) {
    digitalWrite(21, HIGH);
    delayMicroseconds(3);
    digitalWrite(21, LOW);
    delayMicroseconds(3);
  }
  pinMode(21, INPUT);
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
  Wire.begin();

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (i2cData[0] << 8) | i2cData[1];
  accY = (i2cData[2] << 8) | i2cData[3];
  accZ = (i2cData[4] << 8) | i2cData[5];

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();
}

void motorControl(int pwm1, int pwm2, int dir1, int dir2) {
  p1 = max(min(pwm1, 255),0);
  p2 = max(min(pwm2, 255),0);
  if (dir1 == 0) {
    digitalWrite(dirA, LOW);
  }
  if (dir1 == 1) {
    digitalWrite(dirA, LOW);
  }
  if (dir1 == 2) {
    digitalWrite(dirA, HIGH);
  }
  if (dir2 == 0) {
    digitalWrite(dirB, LOW);
  }
  if (dir2 == 1) {
    digitalWrite(dirB, HIGH);
  }
  if (dir2 == 2) {
    digitalWrite(dirB, LOW);
  }
  if (!(dir1 == 0) & !(dir2 == 0)) {
    analogWrite(enA, p1);
    analogWrite(enB, p2);
  } else {
    analogWrite(enA, 0);
    analogWrite(enB, 0);
  }
}

void loop() {
  digitalWrite(5, LOW);
  Serial.print(motor_speed * A_trend_first);
  Serial.print(' ');
  Serial.println(motor_speed2 * A_trend_first2);
  digitalWrite(5, HIGH);
  /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  //tempRaw = (i2cData[6] << 8) | i2cData[7];
  gyroX = (i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = (i2cData[12] << 8) | i2cData[13];

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  /* Print Data */
#if 0 // Set to 1 to activate
  Serial.print(accX); Serial.print("\t");
  Serial.print(accY); Serial.print("\t");
  Serial.print(accZ); Serial.print("\t");

  Serial.print(gyroX); Serial.print("\t");
  Serial.print(gyroY); Serial.print("\t");
  Serial.print(gyroZ); Serial.print("\t");

  Serial.print("\t");
#endif

  //Serial.print(roll); Serial.print("\t");
  //Serial.print(gyroXangle); Serial.print("\t");
  //Serial.print(compAngleX); Serial.print("\t");
  
  robot_angle = kalAngleX;
  //Serial.print(robot_angle); Serial.print("\t");
  error = robot_angle - target_angle;
  integrated_error += error;
  if (integrated_error > 255) integrated_error = 255;
  if (integrated_error < -255) integrated_error = -255;
  delta_error = error - last_error;
  control_decision = int(Kp*error+Kd*delta_error+Ki*integrated_error);
  Serial.println(control_decision); Serial.print("\t");
  if (control_decision > 0){
    motorControl(control_decision, control_decision,1,1);
    }else{
    motorControl(-control_decision, -control_decision, 2,2);
      }
  last_error = error;
  
  

  //Serial.print("\t");

  //Serial.print(pitch); Serial.print("\t");
  //Serial.print(gyroYangle); Serial.print("\t");
  //Serial.print(compAngleY); Serial.print("\t");
  //Serial.print(kalAngleY); Serial.print("\t");

#if 0 // Set to 1 to print the temperature
  Serial.print("\t");

  double temperature = (double)tempRaw / 340.0 + 36.53;
  Serial.print(temperature); Serial.print("\t");
  Serial.print("\r\n");
#endif
  //delay(2);
}
