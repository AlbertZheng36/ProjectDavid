//#include "I2Cdev.h"
//#include "MPU6050_6Axis_MotionApps20.h"
//#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
//    #include "Wire.h"
//#endif

//MPU6050 mpu;
//const int MPU=0x68;  // I2C address of the MPU-6050
//int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
//int16_t accel_X_reading = 0; //lpf of AcX
double AcX, AcY, AcZ;
int enA = 6;
int in1 = 4;
int in2 = 5;
int enB = 10;
int in3 = 8;
int in4 = 9;
int test = 12;
int current_angle = 0;
int target_angle = 0;
bool flipper = false;
int Kp = 3;

// Infomation from glove
enum state {idle,catapult,forward_drive, backup, left_turn,right_turn};
state command = idle;



void setup(){
  //Wire.begin();
  //mpu.initialize();
  //mpu.dmpInitialize();
  //mpu.setIntMotionEnabled(1);
  //mpu.setMotionDetectionThreshold(2);
  //mpu.setMotionDetectionDuration(1);
  
  Serial.begin(115200);
}

void receive_info() {
  int raw_command = Serial.read() - 49;
  // Integrity checking
  if (raw_command < idle || raw_command > right_turn) {
    Serial.print("Fail state");
    return;
  }
  command = (state)raw_command;
  Serial.print(command);
}

void motorControl(int pwm1, int pwm2, int dir1, int dir2) {
  int p1, p2;
  if (pwm1 >= 255) {p1 = 255;}
  else {p1 = pwm1;}
  if (pwm2 >= 255) {p2 = 255;}
  else {p2 = pwm2;}
  if (dir1 == 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
  if (dir1 == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  if (dir1 == 2) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  if (dir2 == 0) {
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }
  if (dir2 == 1) {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }
  if (dir2 == 2) {
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
  if (!(dir1 == 0) & !(dir2 == 0)) {
    analogWrite(enA, p1);
    analogWrite(enB, p2);
  } else {
    analogWrite(enA, 0);
    analogWrite(enB, 0);
  }
}

void loop(){
  while(Serial.available()>0){
    receive_info();
    Serial.print("has serial");
    }
  Serial.print("#\n");
//  flipper = !flipper;
//  if (flipper) {digitalWrite(test, HIGH);}
//  else {digitalWrite(test, LOW);} 
//  //Wire.beginTransmission(MPU);
//  //Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
//  //Wire.endTransmission(false);
//  //Wire.requestFrom(MPU,6,true);  // request a total of 6 registers
//  //AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
//  //AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
//  //AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
//  //Calibration of Ax, Ay, and Az
//  AcX = (analogRead(A5)-342)/68.0;
//  //AcY = (analogRead(A4)-341)/67.0;
//  AcZ = (analogRead(A3)-350)/70.0;
//  //Serial.print("  accel_X_reading is ");Serial.print(AcX);
//  // Don't use y,  y is not reliable 
//  //Serial.print("  accel_Y_reading is ");Serial.print(AcY);
//  //Serial.print("  accel_Z_reading is ");Serial.print(AcZ);
//  //Serial.println(round(atan2(AcZ, AcY)/3.14*180));
//  current_angle = round(atan2(AcX, AcZ)/3.14*180);
//  int error = current_angle - target_angle;
//  //Serial.print("  error=");Serial.println(error);
//  if (error > 0) {
//    motorControl(Kp * error, Kp * error, 1, 1);
//  } else {
//    motorControl(- Kp * error, - Kp * error, 2, 2);
//  }
delay(200);
}
