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
    size(200, 200);
  
  
  println(Arduino.list());
  //arduino = new Arduino(this, Arduino.list()[0], 9600);
  //arduino.pinMode(MotionLED, Arduino.OUTPUT);

  frameRate(5);
  println(Serial.list());
  myPort = new Serial(this, Serial.list()[0], 9600);
}

public void draw() {
    background(204);
//  arduino.digitalWrite(MotionLED, Arduino.HIGH);
//  delay(1000);
//  arduino.digitalWrite(MotionLED, Arduino.LOW);
//  delay(1000);

  while (myPort.available () > 0) {
    String inBuffer = myPort.readStringUntil('\n');   
    if (inBuffer != null) {
       inBuffer = trim(inBuffer);
      String[] nums = split(inBuffer, '|');
      //      Float lat = float(nums[0]);
      //      Float lon = float(nums[1]);
      
      if (nums.length == 13) {
        println(nums);
        
        int   timestamp = PApplet.parseInt(nums[1]);
        float lat = PApplet.parseFloat(nums[2]);
        float lon = PApplet.parseFloat(nums[3]);
        int   validGPS_signal = PApplet.parseInt(nums[4]);

        int VeloShaked = PApplet.parseInt(nums[8]);

        float AccelX = PApplet.parseFloat(nums[9]);
        float AccelY = PApplet.parseFloat(nums[10]);
        float AccelZ = PApplet.parseFloat(nums[11]);


        rect(0, 0, VeloShaked, 8);

        textSize(16);
        text("latitude:"+lat,  10, 16);
        text("longitude:"+lon, 10, 32);
        text("AccelX:"+AccelX, 10, 64);
        text("AccelY:"+AccelY, 10, 80);
        text("AccelZ:"+AccelZ, 10, 96);
      }
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
