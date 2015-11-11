int dir1PinA = 13;
int speedPinA = 11;
int buze = 4;
double AcX, AcY, AcZ;
double error1;
double e1 = 0;
double e2 = 0;
double e3 = 0;
double error_average;
int current_angle = 0;
int target_angle = 0;
int Kp = 3;

void motorControl(int pwm1, int dir1){
  int p1;
  if (pwm1 >= 255) {p1 = 255;}
  else {p1 = pwm1;}
  if (dir1 == 1){
    digitalWrite(dir1PinA, LOW);
    analogWrite(speedPinA,p1);
    }
  if (dir1 == 2){
    digitalWrite(dir1PinA, HIGH);
    analogWrite(speedPinA, p1);
    }
}

void setup() {
  
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(dir1PinA, OUTPUT);
  pinMode(speedPinA, OUTPUT);
  pinMode(buze, OUTPUT);
}

void loop() {
  AcX = (analogRead(A5)-342)/68.0;
  AcZ = (analogRead(A3)-350)/70.0;
  error1 = round(atan2(AcX, AcZ)/3.14*180);
  e1 = e2;
  e2 = e3;
  e3 = error1;
  error_average = (e1 + e2 + e3)/3;
  current_angle = error_average;
  int error = current_angle - target_angle;
  if (error > 0) {
    motorControl(Kp * error, 1);
  } else {
    motorControl(- Kp * error, 2);
  }
  
  
  Serial.print("  accel_X_reading is ");Serial.print(AcX);
  Serial.print("  accel_Z_reading is ");Serial.print(AcZ);
  Serial.print("error is");Serial.println(error_average);
  digitalWrite(buze, LOW);
  // put your main code here, to run repeatedly:
  delay(20);

}
