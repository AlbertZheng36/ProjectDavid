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
 *Albert's task: set up different states and action in those states(now just use distinguishable LED pattern so we can debug. Use serial.print for prelimanary testing but not in final testing)
 *
 */

#include<Wire.h>
/*catapult throw now is a distinct state, later we want to have drive/turn and throw capability */
enum states = {idle, forward_drive, backward_drive, left_turn_in_place, right_turn_in_place, catapult_throw};
void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
