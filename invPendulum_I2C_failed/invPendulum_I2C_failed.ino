#include "I2Cdev.h"
#include "0x686050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

0x686050 0x68;
//const int 0x68=0x68;  // I2C address of the 0x68-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
//int16_t accel_X_reading = 0; //lpf of AcX
//double AcX, AcY, AcZ;
int enA = 10;
int dirA = 12;
int enB = 11;
int dirB = 13;
int test = 6;
double current_angle = 0;
int target_angle = 5;
int moving_factor = 0;
double Kp = 45;
double Ki = 0.15;
double Kd = 160;
int p1, p2;
unsigned long last_time = 0;
unsigned long current_time = 0; 
double Gyro_amount = 0.994;
double robot_angle;
double last_error = 0;
double ITerm = 0;
int GyX_offset = 0;
enum bangBangStates {FULL_FORCE_1, FULL_FORCE_2, NO_FORCE};
bangBangStates myState = NO_FORCE;
//enum states{FORWARD, BALANCE, BACKWARD};
//states state = BALANCE;
int control_decision = 0;

//Sensor output scaling
#define ACCEL_CONFIG 0x1C         //Accelerometer configuration address
#define GYRO_CONFIG 0x1B          //Gyro configuration address
#define accSens 0             // 0 = 2g, 1 = 4g, 2 = 8g, 3 = 16g
#define gyroSens 1            // 0 = 250rad/s, 1 = 500rad/s, 2 1000rad/s, 3 = 2000rad/s

void writeTo(byte device, byte address, byte value) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(true);
}

void calibrateGyro(){
  for (int i = 0; i<200; i++){
    Wire.beginTransmission(0x68);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 2, true);  // request a total of 2 registers
    GyX = (Wire.read() << 8 | Wire.read());
    GyX_offset += GyX;
  }
  GyX_offset = GyX_offset/200;
  
}

void setup(){
  Wire.begin();
  0x68.initialize();
  0x68.dmpInitialize();
  delay (100);
  writeTo(0x68, ACCEL_CONFIG, accSens << 3); // Specifying output scaling of accelerometer
  writeTo(0x68, GYRO_CONFIG, gyroSens << 3); // Specifying output scaling of gyroscope
  delay (100);
  pinMode(test, OUTPUT);
  pinMode(dirA, OUTPUT);
  pinMode(dirB, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  delay(1000);
  calibrateGyro();
  Serial.begin(115200);
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
  if (!(dir1 == 0) && !(dir2 == 0)) {
    analogWrite(enA, p1);
    analogWrite(enB, p2);
  } else {
    analogWrite(enA, 0);
    analogWrite(enB, 0);
  }
}

void loop(){
  digitalWrite(test, HIGH);
  current_time = micros();
  //if (current_time%5000>1000) {target_angle = 11;} else {target_angle = 15;}
//  Wire.beginTransmission(0x68);
//  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
//  Wire.endTransmission(false);
//  Wire.requestFrom(0x68,12,true);  // request a total of 12 registers
//  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
//  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
//  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
//  GyX=Wire.read()<<8|Wire.read();  // 0x41 (ACCEL_XOUT_H) & 0x42 (ACCEL_XOUT_L)     
//  GyY=Wire.read()<<8|Wire.read();  // 0x43 (ACCEL_YOUT_H) & 0x44 (ACCEL_YOUT_L)
//  GyZ=Wire.read()<<8|Wire.read();  // 0x45 (ACCEL_ZOUT_H) & 0x46 (ACCEL_ZOUT_L)
  Wire.beginTransmission(0x68);
  Wire.write(0x3D);                  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 4, true);  // request a total of 4 registers
  AcY = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)

  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 2, true);  // request a total of 2 registers
  GyX = (Wire.read() << 8 | Wire.read()) - GyX_offset; // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  digitalWrite(test, LOW);
  //Serial.println(GyX_offset);
  //Serial.println(GyX);
  //Wire.beginTransmission(0x68);
  //Wire.write(0x1B);
  //Wire.endTransmission(false);
  //Wire.requestFrom(0x68, 1, true);  // request a total of 2 registers
  //int mode = Wire.read() >> 3; // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  //AcX = (analogRead(A5)-342)/68.0;
  //AcY = (analogRead(A4)-341)/67.0;
  //AcZ = (analogRead(A3)-350)/70.0;
  //Serial.print("  accel_X_reading is ");Serial.print(AcX);
  //Serial.print("  accel_Y_reading is ");Serial.print(AcY);
  //Serial.print("  accel_Z_reading is ");Serial.print(AcZ);
  //Serial.print("  gyro_X_reading is ");Serial.println(GyX);
  //Serial.print("  gyro_Y_reading is ");Serial.print(GyY);
  //Serial.print("  gyro_Z_reading is ");Serial.println(GyZ);
  //Serial.println(round(atan2(AcZ, AcY)/3.14*180));
  if (GyX != -1) {
    current_angle = round(atan2(AcY, AcZ)/3.14*180);
    robot_angle += GyX*((double)(current_time-last_time))/1000000.0/65.536;
    robot_angle = robot_angle * Gyro_amount + current_angle *(1 - Gyro_amount);
  }
  double error = robot_angle - target_angle;
  double delta_error = error - last_error;
  ITerm += (Ki * error);
  if (ITerm > 255) ITerm = 255;
  else if (ITerm < -255) ITerm = -255;
 
  control_decision = int(Kp * error + Kd * delta_error + ITerm);

  if (control_decision > 255) control_decision = 255;
  else if (control_decision < -255) control_decision = -255;

  
  //if (abs(error) < 2 && state == BALANCE) state = FORWARD;
  //else if (state == FORWARD && abs(error) > 5) state = BALANCE;
  //if (state == BALANCE){
  //  control_decision = int(Kp * error + Kd * delta_error);
  //  }
  //else if (state == FORWARD){
  //  control_decision = 50;
  //  }
  //Serial.print("  time=");Serial.print(current_time-last_time);
  //Serial.print("  disp=");Serial.print(GyX*((double)(current_time-last_time))*4/65536,10);
  //Serial.print("  mode=");Serial.print(mode);
  //Serial.print("  gyro angle=");Serial.println(robot_angle);
  //Serial.print("  accel angle=");Serial.println(current_angle);
  //Serial.print("  calculated angle=");Serial.println(actual_angle);
  //Serial.print("  robot_angle= ");Serial.println(robot_angle);
  //Serial.print("  control=");Serial.print(control_decision);
  //Serial.println(error);  
  
  if (control_decision > 0) {
    motorControl(control_decision, control_decision, 1, 1);
  } else {
    motorControl(-control_decision, -control_decision, 2, 2);
  }
  
  /*
  if (error > 8){
    myState = FULL_FORCE_1;
  }else if (error < -8){
    myState = FULL_FORCE_2;
  }else if ((myState == FULL_FORCE_1 || myState == FULL_FORCE_2) && abs(error)<3){
    myState = NO_FORCE;
  }
  //Serial.print("  state=");Serial.println(myState); 

  if (myState == FULL_FORCE_1){
    motorControl(180, 180, 1, 1);
  }else if (myState == FULL_FORCE_2){
    motorControl(180, 180, 2, 2);
  }else if (myState == NO_FORCE){
    motorControl(0, 0, 0, 0);
  }
  */
  
  last_time = current_time;
  last_error = error;
  
}
