#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter
#include <DueTimer.h>
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;
#include <Servo.h>
//Servo Command
int servoPin = 9;
Servo cataServo;

// Sensor definitions
int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;
int16_t tempRaw;
double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

// Pin definitions
int enA = 10;
int dirA = 12;
int enB = 11;
int dirB = 13;
int test = 6;

// angle PID control definitions
double current_angle = 0;
double target_angle = -1;
double error = 0;
double last_error = 0;
double delta_error = 0;
double integrated_error = 0;
double Kp = 30;
double Kd = 350;
double Ki = 0.5;

// speed PID control definitions
double current_speed = 0;
double target_speed = 0;
double speed_error = 0;
double last_speed_error = 0;
double delta_speed_error = 0;
double speed_error_sum = 0;
double sp_Kp = 5;
double sp_Kd = 0.5;
double sp_Ki = 0;
double starting_bias = -1;

// other definitions
int p1, p2;
double robot_angle;
int control_decision = 0;
double factor1 = 1;
double factor2 = 1;
bool toggle = false;



// receive info
enum state {idle, forward, backup, left_turn, right_turn, catapult};
state gesture_state = idle;
state command = idle;
state preState = idle;

// servo info
int servo_starting_position = 60;
long cataTimer = 0;
double servo_activation_angle_threshold = 8;
bool servo_ready_for_activation = true;
int servo_target_position = servo_starting_position;
int servo_p = 12;
int servo_return_position = servo_starting_position;

//========================================== don't see this section ===========================================//
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

void receive_info() {
  int raw_command = Serial.read() - 49;
  // Tuning starting_bias
  float first;
  float second;
  if (raw_command == 7) {
    first = Serial.read() - 48;
    second = Serial.read() - 48;
    starting_bias = - first - second / 10;
    Serial.print(starting_bias);
    return;
  }
  if (raw_command == 8) {
    first = Serial.read() - 48;
    second = Serial.read() - 48;
    starting_bias = first + second / 10;
    Serial.print(starting_bias);
    return;
  }

  // Integrity checking
  if (raw_command < idle || raw_command > right_turn) {
    Serial.print("Fail state");
    return;
  }
  command = (state)raw_command;
  Serial.print(command);
}

void servo_activate(){
  //this function activates servo to be at a specific angle to help balance the robot
  servo_target_position = servo_starting_position + servo_p * error;
  cataServo.write(servo_target_position);
}
 
void servo_deactivate(int cataTimer){
  //this function deactivates beam, this will be a slow change, called 500 times before servo is ready for action again 
  //cataTimer goes from 250 to 750, at 250 we just begin deactivation servo_return = servo_target_position
  //, at 750 we should have returned to servo_starting_position
  servo_return_position = int (servo_target_position - (servo_target_position - servo_starting_position)*(cataTimer-250)/500.0); 
  cataServo.write(servo_return_position);
}
void setup() {
  Serial.begin(115200);
  digitalWrite(8, LOW);
  pinMode(21, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  cataServo.attach(servoPin);
  cataServo.write(servo_starting_position);
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
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;
  timer = micros();
}

void motorControl(int pwm1, int pwm2, int dir1, int dir2) {
  p1 = max(min(pwm1, 255), 0);
  p2 = max(min(pwm2, 255), 0);
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
  if (Serial.available()) {
    receive_info();
  }
  digitalWrite(5, LOW);

  // =========================================== 1. FSM state actions ============================================== //
  /*
  if (gesture_state == catapult) {
    cataServo.write(30);
    cataTimer += 1;
    factor1 = 1;
    factor2 = 1;
  }
  */

  if (gesture_state == idle) {
    if (target_speed > 0) {
      target_speed = max(0, target_speed - 0.001);
    } else {
      target_speed = min(0, target_speed + 0.001);
    }
    factor1 = 1;
    factor2 = 1;
  }
  if (gesture_state == forward) {
    target_speed = min(0.5, target_speed + 0.001);
    factor1 = 1;
    factor2 = 1;
  }
  if (gesture_state == backup) {
    target_speed = max(-0.5, target_speed - 0.001);
    factor1 = 1;
    factor2 = 1;
  }
  if (gesture_state == left_turn) {
    target_speed = max(-0.5, target_speed - 0.001);
    factor1 = 0.8;
    factor2 = 1.3;
    toggle = false;
  }
  if (gesture_state == right_turn) {
    target_speed = max(-0.5, target_speed - 0.001);
    factor1 = 1.3;
    factor2 = 0.8;
    toggle = true;
  }
  // ========================================= 2. FSM state transitions ============================================ //
  /*
  if (gesture_state == catapult && cataTimer >= 250) {
    gesture_state = preState;
    cataTimer = 0;
    cataServo.write(0);
    command = preState;
  }
  */
  
  if (gesture_state != command) {
    if (gesture_state == catapult) {
      cataTimer = 0;
      cataServo.write(0);
    }
    preState = gesture_state;
    gesture_state = command;
  }
  if (0) {
    Serial.print(gesture_state); Serial.print("   ");
    Serial.print(cataTimer);
    Serial.print("  now previous state=");
    Serial.println(preState);
  }

  // =========================================== 3. setpoint pid control =========================================== //
  if (!toggle) {
    current_speed = motor_speed * A_trend_first;
  } else {
    current_speed = motor_speed2 * A_trend_first;
  }
  speed_error = -(current_speed - target_speed);
  delta_speed_error = speed_error - last_speed_error;
  speed_error_sum = speed_error_sum + speed_error;
  target_angle = starting_bias + sp_Kp * speed_error + sp_Kd * delta_speed_error + sp_Ki * speed_error_sum;
  if (target_angle > 20) {
    target_angle = 20;
    speed_error_sum -= speed_error;
  }
  else if (target_angle < -20) {
    target_angle = -20;
    speed_error_sum -= speed_error;
  }
  last_speed_error = speed_error;
  if (1) {
    Serial.print("  current speed ="); Serial.print(current_speed);
    Serial.print("  current setpoint ="); Serial.println(target_angle);
  }
  digitalWrite(5, HIGH);

  // =========================================== 4. update sensor data ============================================== //
  while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  gyroX = (i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = (i2cData[12] << 8) | i2cData[13];
  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s
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
  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;
  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;
  /* Print Sensor Data */
  if (0) { // Set to 1 to activate
    Serial.print(accX); Serial.print("\t");
    Serial.print(accY); Serial.print("\t");
    Serial.print(accZ); Serial.print("\t");
    Serial.print(gyroX); Serial.print("\t");
    Serial.print(gyroY); Serial.print("\t");
    Serial.print(gyroZ); Serial.print("\t");
    Serial.println();
  }

  // =========================================== 5. angle pid control ============================================== //
  robot_angle = kalAngleX;
  error = robot_angle - target_angle;
  delta_error = error - last_error;
  integrated_error += error;
  control_decision = int(Kp * error + Kd * delta_error + Ki * integrated_error);
  if (control_decision > 255) {
    control_decision = 255;
    integrated_error -= error;
  }
  else if (control_decision < -255) {
    control_decision = -255;
    integrated_error -= error;
  }
  if (control_decision > 0) {
    motorControl(control_decision * factor1, control_decision * factor2, 1, 1);
  } else {
    motorControl(-control_decision * factor1, -control_decision * factor2, 2, 2);
  }
  last_error = error;
  if (0) { // set to 1 to activate
    Serial.print("  robot_angle="); Serial.println(robot_angle);
    Serial.print("  control decision="); Serial.println(control_decision);
  }
  
  // ======================================== 6. servo counter disturbance control ================================== // 
  /*We would like to use servo beam to help balance our robot only when error angle is too large
  For example, our robot is perfectly able to balance itself within a 5 degree range, however, it may
  struggle to balance itself from 10 degrees. When the error angle exceeds an angle threshold, servo beam
  control will kick in and help balance the bot. After that, beam will slowly return to its original position waiting
  for activation again*/
  if (abs(error) > servo_activation_angle_threshold && servo_ready_for_activation){
    servo_activate();
    servo_ready_for_activation = false;
  }

  if (!servo_ready_for_activation){
    if (cataTimer < 250){
      //increase cataTimer until 250
      cataTimer += 1 ;
    }else if(cataTimer >= 250 && cataTimer <= 750){
      //cataTimer has been incremented to 250, servo will be deactivated, slowly returning to original position and ready for activation again
      cataTimer += 1;
      servo_deactivate(cataTimer);
    }else if(cataTimer >= 750){
      //should already be deactivated 
      cataTimer = 0;
      servo_ready_for_activation = true;
    }
    
  }
  
}
