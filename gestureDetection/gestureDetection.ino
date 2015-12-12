/*setup() is the initilization stuff typically encountered in c and loop() is a while(true) infinite loop, 
 *these two functions can be replaced by standard C code
 *include necessary fils at the top of this file such as the <Wire.h>, be careful of including large libraries since memory is limited
 *Gesture detection framework:
 *sensor inputs: flex sensor input via ADC reading of voltage + acceleromoter input via I2C from accelerometer chip
 *program output: send state information via Xbee radio ONLY WHEN there is a state change (for now, we can use LED to verify the correctness of the program)
 *
 *There is on-chip capabiity to compute accelerometer data but it's hidden from developer so for now we rely on Arduino processor for all computing
 *Inputs: flex sensor reading and accel reading 
 *Outputs: state transition
 *States: idle, forward, backward, left turn in place , right turn in place (add differential drive on two wheels to realize forward left and forward right if necessary)
 *Stop and Balance: Fingers all relax
 *Forward: index and middle finger out  
 *Bakcup: fist
 *Left Turn: rotate hand left (turn speed determined by angle of rotation)
 *Right Turn: rotate hand right (turn speed determined by angle of rotation)
 *Catapult Throw: shake hand to trigger accelerometer 
 *
 *In each execution loop, we first acquire the accel readings (need to determine freq (John's job)) and ADC reading of flex sensor
 *use switch cases to see which state we should be in, if there is a state change, we go into that state
 *
 *Albert's task: set up different states and action in those states(now just use distinguishable LED pattern so we can debug. Use serial.print for prelimanary testing but not in final testing)
 *For example: use LED intensity to show degress of rotation of hand, use distinct blinking patterns to indicate forward_drive and idle state
 */
 
/*notes on unit conversion*/

/*in order to save on expensive floating point conversion, we will use ADC read value instead,
 * this is more challenging in thinking about these quantities intuitively so here is a guideline
 * as to what accel reading means and what voltage ADC readout means.
 * Accel reading: signed 16-bit value for a full range of +- 4g. Example: at rest, reading is x=0, y=0, z=8000
 * flex sensor reading: ADC maps 0-5v to 0 - 1023, 
 * at rest, index reading is 562, middle reading 587, ring reading 600
 * at fist, index reading 766, middle reading 759, ring reading 796
 * choose middle value 650 to compare and determine if finers are relaxed or curled up
 * 
 * data acquisition rate: 755Hz loop speed without any computation, most time spent in writing 
 * to accelerometer. 
 */


#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#define relax(a) (((a)<(650))? (true):(false)) //whehter a finger is in relax state
MPU6050 mpu;
int8_t threshold;
const int MPU=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
int16_t accel_X_reading = 0; //lpf of AcX
bool indexRelax, middleRelax, ringRelax, should_catapult;
enum state {idle,catapult,forward_drive, backup, left_turn,right_turn}; 
state gestureState = idle;
uint8_t turning_speed; //in forward drive and backup mode, turning_speed is set to 0
                       //in turning state, turning speed is a function of accel reading
int16_t x; //test variable for interrupt

void ISR_interrupt(){
  /*upon an interrupt, send catapult throw command*/
  /*right now, we can turn on LED to indicate this event*/
  digitalWrite(13,HIGH);
  should_catapult = true;
  x++;
}

void send_info(){
    // Sample sent message: 1
    int command;
    if (should_catapult) {
      command = catapult;
      should_catapult = false;
    } else {
      command = gestureState;
    }
    // This "+1" are for integrity checking. Ask Albert for details.
    Serial.print(command + 1);
}

void setup(){
  Wire.begin();
  mpu.initialize();
  mpu.dmpInitialize();
  mpu.setIntMotionEnabled(1);
  mpu.setMotionDetectionThreshold(2);
  mpu.setMotionDetectionDuration(1);
  pinMode(7,OUTPUT);
  pinMode(13, OUTPUT);
  digitalWrite(13,LOW);
  attachInterrupt(0,ISR_interrupt,FALLING); // Used 0 here because digitalPinToInterrupt(2) returns 0
  Serial.begin(115200);
}
void loop(){
  /*data acquisition code*/
  /*data acquired: AcX, AcY, AcZ, index, middle and ring Finger reading.*/
  digitalWrite(7,HIGH);
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,6,true);  // request a total of 6 registers
  digitalWrite(7, LOW);
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  accel_X_reading = AcX + 900; // 900 is the bias 
  Serial.print("accel_X_reading is ");Serial.println(accel_X_reading);
  //Serial.print("number of interrupts is ");Serial.println(x);
  int indexFingerReading = analogRead(A0);
  indexRelax = relax(indexFingerReading);
  int middleFingerReading = analogRead(A1);
  middleRelax = relax(middleFingerReading);
  int ringFingerReading = analogRead(A2);
  ringRelax = relax(ringFingerReading);
  /* data acquisition code end*/

  /*forward and backup take precedence over turns*/
  if (indexRelax && middleRelax && ringRelax && abs(accel_X_reading) < 9000){
    //now we use AcX as the only reference to turning angle, 
    //might be more useful to calculate angle instead but this is what we have now
    gestureState = idle;
    turning_speed = 0; 
  }else if (!indexRelax && !middleRelax && !ringRelax){
    gestureState = backup;
    turning_speed = 0;
  }else if (indexRelax && middleRelax && !ringRelax){
    gestureState = forward_drive;
    turning_speed = 0;
  }else if (accel_X_reading >= 9000){
    gestureState = left_turn;
    turning_speed = (int8_t)accel_X_reading;//extract top 8 bits of accel_X_reading
  }else if (accel_X_reading <= 90000){
    gestureState = right_turn;
    turning_speed = (int8_t)accel_X_reading;//extract top 8 bits of accel_X_reading
  }

  //Serial.print("state is "); Serial.println(gestureState);
  //Serial.print("turningSpeed is "); Serial.println(turning_speed);
  delay(333);
  /*Xbee send packets to inform*/
  send_info();


  /*
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  */

  /*
  Serial.print("index reading = "); Serial.print(indexFingerReading);
  Serial.print(" |middle reading = "); Serial.print(middleFingerReading);
  Serial.print(" |ring reading = "); Serial.print(ringFingerReading);
  Serial.print(" |AcX = "); Serial.print(AcX);
  Serial.print(" | AcY = "); Serial.print(AcY);
  Serial.print(" | AcZ = "); Serial.println(AcZ);
  */
  
  /*
  Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  Serial.print(" | GyX = "); Serial.print(GyX);
  Serial.print(" | GyY = "); Serial.print(GyY);
  Serial.print(" | GyZ = "); Serial.println(GyZ);
  */
}
