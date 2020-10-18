import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

import processing.serial.*; 
import cc.arduino.*; 

import java.util.HashMap; 
import java.util.ArrayList; 
import java.io.File; 
import java.io.BufferedReader; 
import java.io.PrintWriter; 
import java.io.InputStream; 
import java.io.OutputStream; 
import java.io.IOException; 

public class MKRFox_GPS_Shield_high_consumption extends PApplet {




Arduino arduino;
int MotionLED = 6;

Serial myPort;
Float lat;
Float lon;

public void setup() {
  println(Arduino.list());
  arduino = new Arduino(this, Arduino.list()[0], 9600);
  arduino.pinMode(MotionLED, Arduino.OUTPUT);

  frameRate(5);
  println(Serial.list());
  myPort = new Serial(this, Serial.list()[0], 9600);
}

public void draw() {
  arduino.digitalWrite(MotionLED, Arduino.HIGH);
  delay(1000);
  arduino.digitalWrite(MotionLED, Arduino.LOW);
  delay(1000);

  while (myPort.available () > 0) {
    String inBuffer = myPort.readString();   
    if (inBuffer != null) {
      String[] nums = split(inBuffer, '|');
      //      Float lat = float(nums[0]);
      //      Float lon = float(nums[1]);
      println(nums);
    }
  }
}

  static public void main(String[] passedArgs) {
    String[] appletArgs = new String[] { "MKRFox_GPS_Shield_high_consumption" };
    if (passedArgs != null) {
      PApplet.main(concat(appletArgs, passedArgs));
    } else {
      PApplet.main(appletArgs);
    }
  }
}
