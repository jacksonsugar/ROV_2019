#include "Arduino.h"
#include <Adafruit_NeoPixel.h>
#include <Servo.h> 
#include <ros.h>
#include <sensor_msgs/Joy.h>
#include <ArduinoHardware.h>
#include <geometry_msgs/Twist.h>

#ifdef __AVR__
#include <avr/power.h>
#endif

ros::NodeHandle nh;
geometry_msgs::Twist msg;
//ros::NodeHandle_<ArdunioHardware, 2, 2, 150, 150> nh;

//Neopixel
#define PIN 4

// Full Bridge 1
#define enA 9
#define in1 27
#define in2 28
#define enB 10
#define in3 39
#define in4 29

// Full Bridge 2
#define enC 7
#define in5 23
#define in6 24
#define enD 8
#define in7 26
#define in8 25

// MOSFET Pins
#define D1 49
#define D2 48
#define D3 47
#define D4 43

// ESC Pins
#define E1 2
#define E2 5
#define E3 11
#define E4 12
#define E5 13
#define E6 46
#define E7 45
#define E8 44

//Servo Pins
#define S1 7
#define S2 6
#define S3 3

Adafruit_NeoPixel strip = Adafruit_NeoPixel(12, PIN, NEO_GRB + NEO_KHZ800);

int updown = 1500;

int cam_ser = 1500;

int m1_dir = 0;

Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;
Servo esc5;
Servo esc6;
Servo esc7;
Servo esc8;

// Camera Servo PWM

Servo cam1;
int cam_pos = 50;

void twist(const geometry_msgs::Twist& cmd_vel)
{
  // What do autonomous

  int Xdir = cmd_vel.linear.x * 100;
  int Ydir = cmd_vel.linear.y * 100;
  int look = cmd_vel.angular.z * 100;

  int theta = atan(Xdir/Ydir);

  int M1 = (Ydir) - (Xdir) - (look); 
  int M2 = (Ydir) + (Xdir) + (look); 
  int M3 = (Ydir) + (Xdir) - (look); 
  int M4 = (Ydir) - (Xdir) + (look); 

  M1 = constrain(M1, -100, 100);
  M2 = constrain(M2, -100, 100);
  M3 = constrain(M3, -100, 100);
  M4 = constrain(M4, -100, 100);

  int m1 = map(M1, -100, 100, 1000, 2000);
  int m2 = map(M2, -100, 100, 1000, 2000);
  int m3 = map(M3, -100, 100, 1000, 2000);
  int m4 = map(M4, -100, 100, 1000, 2000);

  float ud = cmd_vel.linear.z * 100;

  int updown = map(ud, -100, 100, 1000, 2000);  

  // REMAP to slow shit way way way down

  m1 = map(m1, 1000, 2000, 1250, 1750);
  m2 = map(m2, 1000, 2000, 1250, 1750);
  m3 = map(m3, 1000, 2000, 1250, 1750);
  m4 = map(m4, 1000, 2000, 1250, 1750);
  updown = map(updown, 1000, 2000, 1250, 1750);


  esc1.writeMicroseconds(m1);
  esc2.writeMicroseconds(m2);
  esc3.writeMicroseconds(m3);
  esc4.writeMicroseconds(m4);
  esc5.writeMicroseconds(updown);   
  esc6.writeMicroseconds(updown);

}

void esc_cb(const sensor_msgs::Joy& cmd_msg)
{

  // Camera Servo Control

  int cam_input = cmd_msg.axes[7];
  if (cam_input != 0){
    if(cam_input == 1){
      cam_pos = cam_pos + 10;
    }
    else if(cam_input == -1){
      cam_pos = cam_pos + -10;
    }
    else{
    }

    cam_pos = constrain(cam_pos, 0, 100);
    cam_ser = map(cam_pos, 0, 100, 1900, 1100);
  }
  else{
  }

  cam1.writeMicroseconds(cam_ser);


  // Movement + Look

  int Xdir = cmd_msg.axes[0] * 100;
  int Ydir = cmd_msg.axes[1] * 100;
  int look = cmd_msg.axes[3] * 100;

  int theta = atan(Xdir/Ydir);

  int M1 = (Ydir) - (Xdir) - (look); 
  int M2 = (Ydir) + (Xdir) + (look); 
  int M3 = (Ydir) + (Xdir) - (look); 
  int M4 = (Ydir) - (Xdir) + (look); 

  M1 = constrain(M1, -100, 100);
  M2 = constrain(M2, -100, 100);
  M3 = constrain(M3, -100, 100);
  M4 = constrain(M4, -100, 100);

  int m1 = map(M1, -100, 100, 1000, 2000);
  int m2 = map(M2, -100, 100, 1000, 2000);
  int m3 = map(M3, -100, 100, 1000, 2000);
  int m4 = map(M4, -100, 100, 1000, 2000);

  float down = cmd_msg.axes[2] + 1;

  float  up = cmd_msg.axes[5] + 1;

  down = down * 100;

  up = up * 100;

  down = map(down, 0, 200, 0, 500);

  up = map(up, 0, 200, 0, 500);  

  int updown = 1500 - down + up;

  int superman = cmd_msg.buttons[0];  

  if(superman == 1){

  }

  else{
    m1 = map(m1, 1000, 2000, 1250, 1750);
    m2 = map(m2, 1000, 2000, 1250, 1750);
    m3 = map(m3, 1000, 2000, 1250, 1750);
    m4 = map(m4, 1000, 2000, 1250, 1750);
    updown = map(updown, 1000, 2000, 1250, 1750);

  }

  esc1.writeMicroseconds(m1);
  esc2.writeMicroseconds(m2);
  esc3.writeMicroseconds(m3);
  esc4.writeMicroseconds(m4);
  esc5.writeMicroseconds(updown);   
  esc6.writeMicroseconds(updown);

  // DC Motors

  int m1_dir = cmd_msg.axes[6] + 1;

  switch(m1_dir){
  case 0:
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, 250);
    break;

  case 2:
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, 250);
    break;

  default:
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(enA, 0);
    break;

  }

}


ros::Subscriber<sensor_msgs::Joy> sub("joy", esc_cb);
ros::Subscriber<geometry_msgs::Twist> subb("/cmd_vel", twist);

void setup(){

  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(subb);

  strip.begin();
  strip.setBrightness(20);
  strip.show(); // Initialize all pixels to 'off'
  colorWipe(strip.Color(255, 255, 255), 50); // Red
  colorWipe(strip.Color(0, 0, 255), 50); // Blue
  colorWipe(strip.Color(255, 255, 255), 50); // Blue
  rainbowCycle(5);

  esc1.attach(E1); //attach it to pin 44
  esc2.attach(E2); //attach it to pin 45
  esc3.attach(E3); //attach it to pin 46
  esc4.attach(E4); //attach it to pin 13
  esc5.attach(E5); //attach it to pin 12
  esc6.attach(E6); //attach it to pin 11
  esc7.attach(E7); //attach it to pin 5
  esc8.attach(E8); //attach it to pin 2

  cam1.attach(S1); //attach forward camera servo

  // Full Bridge 1
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Full Bridge 2
  pinMode(enC, OUTPUT);
  pinMode(enD, OUTPUT);
  pinMode(in5, OUTPUT);
  pinMode(in6, OUTPUT);
  pinMode(in7, OUTPUT);
  pinMode(in8, OUTPUT);

  //Mosfets
  pinMode(D1, OUTPUT);  
  pinMode(D2, OUTPUT);  
  pinMode(D3, OUTPUT);  
  pinMode(D4, OUTPUT);

}

void loop(){
  nh.spinOnce();
  delay(1);
}


// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}

void rainbow(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((i+j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}






