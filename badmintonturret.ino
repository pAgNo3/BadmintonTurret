#include <Servo.h>
#include <ArduinoJson.h>
#include <NeoSWSerial.h>

NeoSWSerial btSerial(A5, A4);
const int loading_angle = 133;
float current_speed;
float signaled_speed;
float ramp_rate;


Servo y;
Servo p;
Servo Rbldc;
Servo Lbldc;
Servo feeder;

float speed  = 0.0;    
int yaw      = 90;
int pitch    = 90;
int feed     = 87;
int shoot    = 0;

void setup() {
  btSerial.begin(9600);        // Default communication rate for HC-05 data mode
  btSerial.setTimeout(5);     // <--- ADD THIS LINE (Prevents 1000ms read timeout)
  y.attach(9);
  p.attach(10);
  Rbldc.attach(3, 1000, 2000);
  Lbldc.attach(5, 1000, 2000);
  feeder.attach(6);

  current_speed=0;
  delay(1000);
}

void loop() {
  if (btSerial.available() > 0) {
      StaticJsonDocument<200> doc;
    String incomingJson = btSerial.readStringUntil('\n');
    DeserializationError error = deserializeJson(doc, incomingJson);
    if (error) {
      return; // Skip the rest of the loop if data is corrupted
    }
    pitch = doc["p"] | pitch;
    yaw   = doc["y"] | yaw;
    speed = doc["s"] | speed;
    shoot = doc["sh"] | 0;
    feed  = doc["f"] | feed; 
  }
  y.write(yaw);
  p.write(pitch);
  if (speed > current_speed){
       if (speed < 20){  ramp_rate = 0.5;}
       else { ramp_rate = 1.5 ;}
  } 
  else if (speed < current_speed) {ramp_rate = -1.0;}
  if ( abs(current_speed-speed)<2 && abs(current_speed - speed) > 0.1){
    signaled_speed = speed;
    Lbldc.writeMicroseconds((10*signaled_speed)+1000);
    Rbldc.writeMicroseconds((10*signaled_speed)+1000);
    current_speed= speed;
    ramp_rate=0.0;
    return;
  }
  signaled_speed = current_speed + ramp_rate ;
  Lbldc.writeMicroseconds((10*signaled_speed)+1000);
  Rbldc.writeMicroseconds((10*signaled_speed)+1000);
  current_speed = signaled_speed;

  if (abs(current_speed - speed)<0.1 && shoot == 1){
    feeder.write(loading_angle);
    delay(150);
    feeder.write(0);
  }
  else{feeder.write(feed);}
  delay(10);
}
