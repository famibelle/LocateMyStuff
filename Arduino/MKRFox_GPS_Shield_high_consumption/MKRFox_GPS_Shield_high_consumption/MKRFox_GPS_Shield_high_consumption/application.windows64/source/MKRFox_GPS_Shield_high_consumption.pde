import processing.serial.*;
import cc.arduino.*;

Arduino arduino;
int MotionLED = 6;

Serial myPort;
Float lat;
Float lon;

void setup() {
  size(640, 360, P3D);
  
  println(Arduino.list());
  //arduino = new Arduino(this, Arduino.list()[0], 9600);
  //arduino.pinMode(MotionLED, Arduino.OUTPUT);

  frameRate(5);
  println(Serial.list());
  myPort = new Serial(this, Serial.list()[0], 9600);
}

void draw() {
  background(0, 0, 26);
  lights();
  //arduino.digitalRead(MotionLED);

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

        int EssaiEnvoiSigFox = int(nums[5]);
        int SuccesEnvoiSigFox = int(nums[6]);

        int VeloShaked = int(nums[8]);

        float AccelX = float(nums[9]);
        float AccelY = float(nums[10]);
        float AccelZ = float(nums[11]);

        long seconds = timestamp / 1000;
        long minutes = seconds / 60;
        long hours = minutes / 60;
        long days = hours / 24;
        timestamp %= 1000;
        seconds %= 60;
        minutes %= 60;
        hours %= 24;

        int positionRec = 1; 
        rect(0, 0, VeloShaked, (positionRec++)*16);

        textSize(16);
        text("Uptime: " + days + "d " + hours + "h" + minutes + "mn" + seconds + "s",
                                 10, (positionRec++)*16);
        text("latitude:" + lat,  10, (positionRec++)*16);
        text("longitude:" + lon, 10, (positionRec++)*16);

        text("Tentatives SigFox:\t" + SuccesEnvoiSigFox, 10, (positionRec++)*16);
        text("Succes SigFox:\t" + SuccesEnvoiSigFox, 10, (positionRec++)*16);

        text("AccelX:" + AccelX, 10, (positionRec++)*16);
        text("AccelY:" + AccelY, 10, (positionRec++)*16);
        text("AccelZ:" + AccelZ, 10, (positionRec++)*16);

        // Use array of ints to set the color and height of each rectangle.
        rect(1*32+200, 100, 20, AccelX*50);
        rect(2*32+200, 100, 20, AccelY*50);
        rect(3*32+200, 100, 20, AccelZ*50);

        translate(width/2, height/2);
        pushMatrix();

        fill(112);
        
        rotateX(map(AccelX*50, 0, width, 0, TWO_PI));
        rotateY(map(AccelY*50, 0, width, 0, TWO_PI));
        rotateZ(map(AccelZ*50, 0, width, 0, TWO_PI));

        box(100,50,25);
        popMatrix();
        }
      }
    }
  }
