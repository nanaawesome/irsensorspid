#include <Wire.h>
#define I2C_SLAVE_ADDR 0x04 // 4 in hexadecimal

#define LEFT1 35 //pin 35 working
#define LEFT2 4 //pin 4 working
#define LEFT3 25 //pin 25 working
#define RIGHT1 13 //pin 13 working
#define RIGHT2 26 //pin 26 working
#define RIGHT3 27 //pin 27 working
double Kp=5,Ki=0,Kd=0,K=0.5, baseSpeed=150;
double centreAngle = 83,cumulative_error=0, prev_error=0;
int16_t servoAngle,leftMotorSpeed, rightMotorSpeed;


void setup() {
  Serial.begin(115200);
  //Serial.begin(9600);
  Wire.begin();   // join i2c bus 
}

void loop() {
  int left_1 = map(analogRead(LEFT1), 0, 4096, 10, 0);
  int left_2 = map(analogRead(LEFT2), 0, 4096, 10, 0);
  int left_3 = map(analogRead(LEFT3), 0, 4096, 10, 0);

  int right_1 = map(analogRead(RIGHT1), 0, 4096, 10, 0);
  int right_2 = map(analogRead(RIGHT2), 0, 4096, 10, 0);
  int right_3 = map(analogRead(RIGHT3), 0, 4096, 10, 0);

  
  int sensor_val = left_1+left_2+left_3+right_1+right_2+right_3;
  double w_average = ((left_1*-8)+(left_2*-23)+(left_3*-37)+(right_1*8)+(right_2*23)+(right_3*37))/sensor_val;

  int setpoint = 0;
  double error = setpoint-w_average;
  Serial.print("Error: ");
  Serial.println(error);
  
  double u = (Kp*error)+(Ki*cumulative_error)+(Kd*(error-prev_error));
  Serial.print("PID: ");
  Serial.println(u);
  servoAngle = centreAngle+u;
  Serial.print("Servo Angle: ");
  Serial.println(servoAngle);
  leftMotorSpeed = baseSpeed+(K*u);
  rightMotorSpeed = baseSpeed-(K*u);
  transmit_to_arduino();
  delay(30);
  cumulative_error +=error;
  prev_error = error;

}

void transmit_to_arduino(){
  Wire.beginTransmission(I2C_SLAVE_ADDR); // transmit to device #4
    Wire.write((byte)((leftMotorSpeed & 0x0000FF00) >> 8));    // first byte of x, containing bits 16 to 9
    Wire.write((byte)(leftMotorSpeed & 0x000000FF));           // second byte of x, containing the 8 LSB - bits 8 to 1
    Wire.write((byte)((rightMotorSpeed & 0x0000FF00) >> 8));    // first byte of y, containing bits 16 to 9
    Wire.write((byte)(rightMotorSpeed & 0x000000FF));           // second byte of y, containing the 8 LSB - bits 8 to 1
    
    Wire.write((byte)((servoAngle & 0x0000FF00) >> 8));    // first byte of y, containing bits 16 to 9
    Wire.write((byte)(servoAngle & 0x000000FF));           // second byte of y, containing the 8 LSB - bits 8 to 1
  Wire.endTransmission();   // stop transmitting 
  
}
