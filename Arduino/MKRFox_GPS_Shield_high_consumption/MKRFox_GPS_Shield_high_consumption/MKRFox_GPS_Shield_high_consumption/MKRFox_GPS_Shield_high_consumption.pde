import processing.serial.*;
import cc.arduino.*;

Arduino arduino;
int MotionLED = 6;

Serial myPort;
Float lat;
Float lon;

void setup() {
    size(200, 200);
  
  
  println(Arduino.list());
  //arduino = new Arduino(this, Arduino.list()[0], 9600);
  //arduino.pinMode(MotionLED, Arduino.OUTPUT);

  frameRate(5);
  println(Serial.list());
  myPort = new Serial(this, Serial.list()[0], 9600);
}

void draw() {
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
        
        int   timestamp = int(nums[1]);
        float lat = float(nums[2]);
        float lon = float(nums[3]);
        int   validGPS_signal = int(nums[4]);

        int VeloShaked = int(nums[8]);

        float AccelX = float(nums[9]);
        float AccelY = float(nums[10]);
        float AccelZ = float(nums[11]);


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

