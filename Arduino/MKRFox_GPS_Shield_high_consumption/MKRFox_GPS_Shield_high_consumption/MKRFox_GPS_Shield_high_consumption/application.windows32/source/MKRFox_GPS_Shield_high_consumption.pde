import processing.serial.*;
import cc.arduino.*;

Arduino arduino;
int MotionLED = 6;

Serial myPort;
Float lat;
Float lon;

void setup() {
  println(Arduino.list());
  arduino = new Arduino(this, Arduino.list()[0], 9600);
  arduino.pinMode(MotionLED, Arduino.OUTPUT);

  frameRate(5);
  println(Serial.list());
  myPort = new Serial(this, Serial.list()[0], 9600);
}

void draw() {
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

