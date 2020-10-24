#include <ArduinoLowPower.h>
#include <Arduino_MKRGPS.h>
#include <SandTimer.h>
#include <SigFox.h>
#include <HeartBeat.h>

Heartbeat GPSSignalHeartbeat;

#define MOTIONLED 6 // the LED that will goes ON when a motion is detected
#define HEARTBEATLED 2 // the LED that will goes ON when GPS signal is received
#define MPU6050_HARD_INT 0 // la broche 0 du MKRFOX est connectée sur la broche INT du MPU6050
#define INTERRUPT_PIN 0  // use pin 0 on Arduino MKRFox

#define VEILLE_MIN 60 // durée du mode veille
#define WAITING_TIME_IN_MIN 60
#define FENETRE_ENVOI 3 //durée entre deux envois en minutes
#define GPS_ON_DURATION_IN_MN 10
#define AWAKE_TIME_IN_MN 10 // en minutes


SandTimer WaitingTimeForTheFullCycle;
SandTimer MinDelayBetween2SigfoxSending;
SandTimer MiseEnVeille;
//unsigned long LastTimeMessageWasSent = WAITING_TIME_IN_MIN*60*1000;        // will store last time SigFox message was sent
unsigned long LastTimeMessageWasSent = 0;        // will store last time SigFox message was sent
unsigned long HeureDernierEnvoi;
unsigned long FirstStartOfModule;

bool delayRunning = false; // true if still waiting for delay to finish

bool debug = true; // when set to true, the module expects a serial connection, otherwise, it gets stuck. The purpose is to sent to serial port generic debug information
bool gy_521 = true; // set to capture the signal from a gyroscope

////////////////// SigFox Variables
static String sigfoxId;
int essais = 0;
int succes = 0;

//////////////////  GPS Variables //////////////////
float lat = 12;
float lon = 34; // create variable for latitude and longitude object
float altitude = 567;
unsigned long fix_age; // returns last GPS fix in millisec
bool GPSPowerOn = false;
bool GPS_ValidSignalReceived = false;
SandTimer SablierGPS_On;
SandTimer SablierGPS_WakeUp;
SandTimer Sablier2mn;
/*
  source https://github.com/nicolsc/SmartEverything_SigFox_GPS
  - 25472d42 : hexadecimal value of latitude,  43.319477081299
  - 7f6ac43f : hexadecimal value of longitude, 1.534500002861
  - dc000000 : hexadecimal value of latitude,  220
*/

struct gpscoord {
  float a_latitude;  // 4 bytes
  float a_longitude; // 4 bytes
  float a_altitude;  // 4 bytes
  bool  validGPS_signal;
};

////////////////// Creation d'un troisième port serie Serial2() //////////////////
// source https://forum.arduino.cc/index.php?topic=483520.0
//Uart Serial2 (&sercom2, 5, 6, SERCOM_RX_PAD_1, UART_TX_PAD_0);
//
//void SERCOM2_Handler()
//{
//  Serial2.IrqHandler();
//}
//
//uint8_t start[] = { 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xEB, 0x90, 0xEB, 0x90, 0xEB, 0x90};
//uint8_t id = 0x16;
//uint8_t cmd[] = { 0xA0, 0x00, 0xB1, 0xA7, 0x7F };
//uint8_t buff[128];
//
///////////////////////////////////////////////////////////////////////////////////


//////////////////  Gyro variables and functions  //////////////////
const int MPU = 0x68; //I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ; //16-bit integers
int AcXcal, AcYcal, AcZcal, GyXcal, GyYcal, GyZcal, tcal; //calibration variables
double t, tx, tf;
int minVal = 265;
int maxVal = 402;
float AccelX, AccelY, AccelZ = 0;
float accelXangle, accelYangle = 0;
float pitch, roll = 0;
struct Quake {
  int Secousses = 0;
  unsigned long timeStamp = millis();
}; // compteur du nombre de fois que le vélo a été secoué

Quake VeloShaked;
float SENSIBILITE_EN_G = 1.5; // sensibilité de l'accéléromètre
/*
  source https://create.arduino.cc/projecthub/Nicholas_N/how-to-use-the-accelerometer-gyroscope-gy-521-6dfc19
  Connections to the GY-521
  VCC -> 3.3 V / 5 V (better)
  GND -> GND
  SCL -> SCL
  SDA -> SDA
  XDA ->
  XCL ->
  ADO ->
  INT -> 0
*/

//function to convert accelerometer values into pitch and roll

void setAccelSensitivity(uint8_t g) {
  //Config AFS_SEL[1:0] bits 4 and 3 in register 0x1C
  //0x00: +/-2g (default)
  //0x08: +/-4g
  //0x10: +/-8g
  //0x18: +/-16g

  Wire.beginTransmission(MPU);   //initialize comm with MPU @ 0x68
  Wire.write(0x1C);                   //write to register 0x1C
  Wire.write(g);                      //setting bit 7 to 1 resets all internal registers to default values
  Wire.endTransmission();             //end comm
}

void readAccel(float accelDivisor) {
  //NOTE: as you increase the accelerometer's range, the resolution decreases
  //+/-2g, use divisor of 16384 (14-bit resolution)
  //+/-4g, use divisor of 8192 (13-bit resolution)
  //+/-8g, use divisor of 4096 (12-bit resolution)
  //+/-16g, use divisor of 2048 (11-bit resolution)

  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(MPU, 6);    //read 6 consecutive registers starting at 0x3B
  if (Wire.available() >= 6) {
    int16_t temp0 = Wire.read() << 8;   //read upper byte of X
    int16_t temp1 = Wire.read();        //read lower byte of X
    AccelX = (float) (temp0 | temp1);
    AccelX = AccelX / accelDivisor;

    temp0 = Wire.read() << 8;           //read upper byte of Y
    temp1 = Wire.read();                //read lower byte of Y
    AccelY = (float) (temp0 | temp1);
    AccelY = AccelY / accelDivisor;

    temp0 = Wire.read() << 8;           //read upper byte of Z
    temp1 = Wire.read();                //read lower byte of Z
    AccelZ = (float) (temp0 | temp1);
    AccelZ = AccelZ / accelDivisor;

  }
  //You can only calculate roll and pitch from accelerometer data
  accelXangle = (atan2(AccelY, AccelZ)) * 180 / PI;                                   //calculate roll
  accelYangle = (atan2(-AccelX, sqrt(pow(AccelY, 2) + pow(AccelZ, 2)))) * 180 / PI;   //calculate pitch
}

void displaySerial(void) {
  Serial.print("AccelX: "); Serial.print(AccelX);
  Serial.print("\tAccelY: "); Serial.print(AccelY);
  Serial.print("\tAccelZ: "); Serial.print(AccelZ);
  Serial.println("\t");

  Serial.print(" Pitch: ");
  Serial.print(accelYangle);
  Serial.print(" Roll: ");
  Serial.println(accelXangle);
}

//////////////////    The Gyro part
float QuelleAccelaration() {      // that the gyroscope part
  readAccel(16384.0);     //read XYZ Accel data from registers 0x3B to 0x40
  //displaySerial();      // uncomment to see the logs

  String Pitch_Roll;
  float Xa;
  float Ya;
  float Za;

  float Xg;
  float Yg;
  float Zg;

  //  if (gy_521) { // that the gyroscope part
  //    // convertit les pitch et le roll en chaine de caractère pour remplacer les derniers caractères
  //    Pitch_Roll = PositionData.substring(0, 7) + String(pitch, HEX) + String(roll, HEX);
  //    if (debug) {
  //      Serial.print("Pitch = "); Serial.print(pitch);
  //      Serial.print(" Roll = "); Serial.println(roll);
  //      //Serial.print("Pitch et Roll en hexa: "); Serial.println(Pitch_Roll);
  //    }
  //  }

  Wire.beginTransmission(MPU); //begin transmission to I2C slave device
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false); //restarts transmission to I2C slave device
  Wire.requestFrom(MPU, 14, true); //request 14 registers in total

  //Acceleration data correction
  AcXcal = 0;
  AcYcal = 0;
  AcZcal = 0;

  //Temperature correction
  //tcal = -1600;

  //Gyro correction
  GyXcal = -256;
  GyYcal = -250;
  GyZcal = -197;

  //read accelerometer data
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) 0x40 (ACCEL_ZOUT_L)

  //read temperature data
  Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) 0x42 (TEMP_OUT_L)

  //read gyroscope data
  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) 0x48 (GYRO_ZOUT_L)

  // Correction de l'offset
  AcX = AcX - AcXcal;
  AcY = AcY - AcYcal;
  AcZ = AcZ - AcZcal;
  //Temperature correction
  //tcal = -1600;

  //Gyro correction
  GyX = GyX - GyXcal;
  GyY = GyY - GyYcal;
  GyZ = GyZ - GyZcal;

  int xAng = map(AcX, minVal, maxVal, -90, 90);
  int yAng = map(AcY, minVal, maxVal, -90, 90);
  int zAng = map(AcZ, minVal, maxVal, -90, 90);

  /*
    Formula to calculate x value in degree (0 to 360) is given below. Here we convert only x because the servo motor rotation is based on x value movement.
  */
  double x = RAD_TO_DEG * (atan2(-yAng, -zAng) + PI);
  double y = RAD_TO_DEG * (atan2(-zAng, -xAng) + PI);
  double z = RAD_TO_DEG * (atan2(-xAng, -yAng) + PI);

  // X angle value, from 0 to 360 deg, is converted into 0 to 180.
  int pos = map(x, 0, 180, 0, 180);

  Xa = AcX / 16384.0;
  Ya = AcY / 16384.0;
  Za = AcZ / 16384.0;

  Xg = GyX / 16.4;
  Yg = GyX / 16.4;
  Zg = GyZ / 16.4;

  //temperature calculation
  tx = Tmp + tcal;
  t = tx / 340 + 36.53; //equation for temperature in degrees C from datasheet
  tf = (t * 9 / 5) + 32; //fahrenheit

  /*
    int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
    définit que les variables mesurées sont des codés sur 16 bits et sont signées (peuvent être négatives).
    Elles peuvent donc prendre une valeur allant de -32 768 à 32 767,
    soit de -(2^{15}) à 2^{15}-1 (source wikipedia).
  */

  float Acceleration;
  Acceleration = sqrt(AccelX * AccelX + AccelY * AccelY + AccelZ * AccelZ);
  return Acceleration;
}


//////////////////  GPS On/Off to save current  //////////////////
// source https://ukhas.org.uk/guides:ublox_psm
// Send a byte array of UBX protocol to the GPS
void sendUBX(uint8_t *MSG, uint8_t len) {
  for (int i = 0; i < len; i++) {
    Serial1.write(MSG[i]);
  }
}

/*
  //Set GPS to backup mode (sets it to never wake up on its own)
  uint8_t GPSoff[] = {0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x4D, 0x3B};
  void TurnOffGPS() {
  sendUBX(GPSoff, sizeof(GPSoff) / sizeof(uint8_t));
  }

  //Restart GPS
  uint8_t GPSon[] = {0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x4C, 0x37};
  void TurnOnGPS() {
  sendUBX(GPSon, sizeof(GPSon) / sizeof(uint8_t));
  }
*/
//////////////////  Convert GPS function  //////////////////
/* Converts GPS float data to Char data */
String ConvertGPSdata(const void* data, uint8_t len) {
  uint8_t* bytes = (uint8_t*)data;
  String cadena;
  if (debug) {
    //Serial.print("Length: "); Serial.println(len);
  }

  //  if (debug) {
  //    Serial.print("Chain: ");
  //  }

  for (uint8_t i = len - 1; i < len; --i) {
    if (bytes[i] < 12) { // previously 12
      cadena.concat(byte(0)); // Not tested
    }
    cadena.concat(char(bytes[i]));

    //    if (debug) {
    //      Serial.print("\t");
    //      Serial.print("0x"); Serial.print(bytes[i], HEX); Serial.print(" ");
    //    }
  }

  //  if (debug) {
  //    Serial.println("");
  //    Serial.println("From function ConvertGPSdata: ");
  //    Serial.print("\tLength: "); Serial.println(len);
  //    Serial.print("\tString to send: "); Serial.println(cadena);
  //    //Serial.print("\tData: "); Serial.println(data);
  //  }

  return cadena;
}
////////////////////////// Get GPS position function /////////////////////
gpscoord GetGPSpositon() {
  // GPS coordinate structure, 12 bytes size on 32 bits platforms
  float fn_lat;
  float fn_lng;
  float fn_alt;
  int   fn_sat;
  bool GPSSignal;

  // read GPS values
  if (GPS.available()) {
    GPS_ValidSignalReceived = true;

    fn_lat   = GPS.latitude();
    fn_lng  = GPS.longitude();
    fn_alt   = GPS.altitude();
    fn_sat = GPS.satellites();
    GPSSignal = true;

    if (debug) {
      // print GPS values
      Serial.println();
      Serial.print("Location: ");
      Serial.print(fn_lat, 7);
      Serial.print(", ");
      Serial.println(fn_lng, 7);
      Serial.print("Altitude: ");
      Serial.print(fn_alt);
      Serial.println("m");
      Serial.print("Number of satellites: ");
      Serial.println(fn_sat);
      Serial.println();
    }
  } else {
    GPS_ValidSignalReceived = false;
    fn_lat = 0;
    fn_lng = 0;
    GPSSignal = false;
    //Serial.println("\tNo valid GPS data found, check coverage");
  }

  gpscoord coords = {fn_lat, fn_lng, fn_alt, GPSSignal};
  return coords;
}

/////////////////// Sigfox Send Data function ////////////////
bool SendSigfox(String data) {
  bool PacketSent = false;
  if (debug) {
    Serial.print("Sending: "); Serial.println(data);
    if (data.length() > 12) {
      Serial.println("Message too long, only first 12 bytes will be sent");
    }
  }
  // Remove EOL
  //data.trim();

  // Start the module
  SigFox.begin();

  // Wait at least 30mS after first configuration (100mS before)
  delay(100);

  // Clears all pending interrupts
  SigFox.status();
  delay(1);
  if (debug) SigFox.debug();
  delay(100);

  SigFox.beginPacket();
  SigFox.print(data);

  if (debug) {
    int ret = SigFox.endPacket(true);  // send buffer to SIGFOX network and wait for a response
    PacketSent = ret;
    if (ret > 0) {
      Serial.println("No transmission");
    } else {
      Serial.println("Transmission ok");
    }

    Serial.println(SigFox.status(SIGFOX));
    Serial.println(SigFox.status(ATMEL));

    if (SigFox.parsePacket()) {
      Serial.println("Response from server:");
      while (SigFox.available()) {
        Serial.print("0x");
        Serial.println(SigFox.read(), HEX);
      }
    } else {
      Serial.println("Could not get any response from the server");
      Serial.println("Check the SigFox coverage in your area");
      Serial.println("If you are indoor, check the 20dB coverage or move near a window");
    }

    Serial.println();
  } else {
    PacketSent = SigFox.endPacket();
  }
  SigFox.end();
  return PacketSent;
}

////////////////// Miscelaneous functions ////////
void progressBar(int tick, String message) {
  Serial.print(message);
  for (int i = 1; i < tick; i++) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("[OK]");
}

void GPSdeboutladedans() {
  GPS.wakeup();
  progressBar(10, "Waking up the GPS");
}

//////////////////    SETUP    ///////////////////
void setup() {
  progressBar(10, "Initialisation");
  progressBar(5, "vérification de la porte opposée");
  progressBar(5, "désarmement des tobogans");

  essais = 0;
  succes = 0;
  pinMode(MOTIONLED, OUTPUT);
  pinMode(HEARTBEATLED, OUTPUT);

  pinMode(MPU6050_HARD_INT, INPUT);

  //GPSSignalHeartbeat.begin(HEARTBEATLED,1);

  LowPower.attachInterruptWakeup(MPU6050_HARD_INT, GPSdeboutladedans, CHANGE);
  //    pin: the pin used as external wakeup
  //    callback: the function to call on wakeup
  //    mode: the transitions to sense on the indicated pin. Can be one between:
  //        FALLING
  //        RISING
  //        CHANGE
  Serial.begin(9600); // connect serial

  // If you are using the MKR GPS as shield, change the next line to pass
  // the GPS_MODE_SHIELD parameter to the GPS.begin(...)
  bool initializeGPS = false;
  initializeGPS = GPS.begin(GPS_MODE_SHIELD);
  progressBar(30, "Initialisation du GPS");

  if (initializeGPS == false) {
    Serial.println("Failed to initialize GPS!");
  }

  FirstStartOfModule = millis();

  //////////////////    Setting up the SigFox
  HeureDernierEnvoi = millis() - FENETRE_ENVOI * 60 * 1000;
  if (!SigFox.begin()) {
    Serial.println("Shield error or not present!");
    return;
  }

  // Enable debug led and disable automatic deep sleep
  if (debug) {
    SigFox.debug();
    String version = SigFox.SigVersion();
    sigfoxId = SigFox.ID();
    String PAC = SigFox.PAC();

    // Display module informations
    Serial.println("SigFox FW version " + version);
    Serial.println("\tID  = " + sigfoxId);
    Serial.println("\tPAC = " + PAC);
    Serial.print(F("\tModule temperature: "));
    Serial.println(SigFox.internalTemperature());

  } else {
    SigFox.end(); // Send the module to the deepest sleep.
    // Est ce nécessaire de mettre le SigFox en deep sleep dès le démarrage ?
  }

  //////////////////    Setting up the Gyro
  if (gy_521) { // that the gyroscope part
    Wire.begin(); //initiate wire library and I2C
    Wire.beginTransmission(MPU); //begin transmission to I2C slave device
    Wire.write(0x6B); // PWR_MGMT_1 register, Puts MPU6050 in Sleep Mode
    Wire.write(0); // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true); //ends transmission to I2C slave device
    Serial.begin(9600); //serial communication at 9600 bauds
  }

  ////////////////// Turning On GPS
  /*
    TurnOffGPS();
    Serial.println("GPS turned Off");
    GPSPowerOn = false;
  */
  GPS.wakeup();
  progressBar(10, "Waking up the GPS");

  ///////////////// Premier envoi sur le réseau SigFox
  String PositionData;
  gpscoord MyCoord;

  MyCoord = GetGPSpositon();
  gpscoord coords = {MyCoord.a_latitude, MyCoord.a_longitude, (float)VeloShaked.Secousses}; //  gpscoord coords = {fn_lat, fn_lng, fn_alt, GPSSignal};
  PositionData = ConvertGPSdata(&coords, sizeof(gpscoord)); //Send data
  SendSigfox(PositionData);
  progressBar(10, "Premier envoi sur le réseaux SigFox");

  HeureDernierEnvoi = millis();
  if (debug) {
    Serial.print("Position sent: "); Serial.println(PositionData);
    Serial.print("\tGPS: "); Serial.print(coords.a_latitude); Serial.print(", "); Serial.print(coords.a_longitude); Serial.print("\tSignal Valide: "); Serial.println(coords.validGPS_signal);
    Serial.print("\tessais: "); Serial.print(essais); Serial.print("; succes: "); Serial.println(succes);
  }
}

//////////////////    LOOP    ///////////////////
void loop() {
  //////////////// Les sables du temps
  WaitingTimeForTheFullCycle.start(WAITING_TIME_IN_MIN * 60 * 1000);
  MinDelayBetween2SigfoxSending.start(FENETRE_ENVOI * 60 * 1000);
  SablierGPS_On.start(GPS_ON_DURATION_IN_MN * 60 * 1000);
  SablierGPS_WakeUp.start(WAITING_TIME_IN_MIN * 60 * 1000);
  Sablier2mn.start(2 * 60 * 1000);
  MiseEnVeille.start(VEILLE_MIN * 60 * 1000);



  String PositionData;
  gpscoord MyCoord;

  MyCoord = GetGPSpositon();

  /*
    GPSSignalHeartbeat.beat();
    float f = 5 * (1 + sin(millis()/10000));
    GPSSignalHeartbeat.set(f);
  */

  //analogWrite(HEARTBEATLED, VeloShaked.Secousses);

  if (MyCoord.validGPS_signal == true) {
    //GPSSignalHeartbeat.set(5);
    //analogWrite(HEARTBEATLED, 255);
    digitalWrite(MOTIONLED, HIGH);
  } else {
    //analogWrite(HEARTBEATLED, 0);
    digitalWrite(MOTIONLED, LOW);
    //GPSSignalHeartbeat.set(1);
  }

  gpscoord coords = {MyCoord.a_latitude, MyCoord.a_longitude, (float)VeloShaked.Secousses}; //  gpscoord coords = {fn_lat, fn_lng, fn_alt, GPSSignal};

  PositionData = ConvertGPSdata(&coords, sizeof(gpscoord)); //Send data

  unsigned long currentMillis = millis();
  if (WaitingTimeForTheFullCycle.finished() == true) {
    essais++;
    succes = succes + (int)SendSigfox(PositionData);
    if (debug) {
      Serial.print("Position sent: "); Serial.println(PositionData);
      Serial.print("\tGPS: "); Serial.print(coords.a_latitude); Serial.print(", "); Serial.print(coords.a_longitude); Serial.print("\tSignal Valide: "); Serial.println(coords.validGPS_signal);
      Serial.print("essais: "); Serial.print(essais); Serial.print("; succes: "); Serial.println(succes);
    }
    HeureDernierEnvoi = millis();
    LastTimeMessageWasSent = currentMillis;
  }

  float AccelerationSysteme;
  AccelerationSysteme = QuelleAccelaration();
  if (AccelerationSysteme < SENSIBILITE_EN_G) {
    digitalWrite(MOTIONLED, LOW);
  }

  if (AccelerationSysteme > SENSIBILITE_EN_G) {
    VeloShaked.Secousses++;
    VeloShaked.timeStamp =  millis();
    Serial.print("Le vélo a bougé, l'accélération est de "); Serial.print(AccelerationSysteme ); Serial.print(" g. ");
    Serial.print("Le vélo a été secoué  "); Serial.print(VeloShaked.Secousses); Serial.println(" fois.");

    MiseEnVeille.startOver();

    digitalWrite(MOTIONLED, HIGH);
    if (MinDelayBetween2SigfoxSending.finished() == true) {
      VeloShaked.Secousses = 0;
      succes = succes + (int)SendSigfox(PositionData);
      if (debug) {
        Serial.print("Position sent: "); Serial.println(PositionData);
        Serial.print("\tGPS: "); Serial.print(coords.a_latitude); Serial.print(", "); Serial.print(coords.a_longitude); Serial.print("\tSignal Valide: "); Serial.println(coords.validGPS_signal);
        Serial.print("essais: "); Serial.print(essais); Serial.print("; succes: "); Serial.println(succes);
      }

      HeureDernierEnvoi = millis();
    } else {
      Serial.print("Prochaine fenêtre d'envoi dans "); Serial.print((HeureDernierEnvoi + FENETRE_ENVOI * 60 * 1000 - millis()) / 60 / 1000); Serial.println(" mn");
    }
  }

  // Les valeurs sont séparées par des "|" pour être ensuite transmises à Processing
  Serial.print("Pipe separated values: GPS::latitude GPS:longitude GPS::signal  sigfox::essai sigfox::essais Velo::acceleration  Velo::secousses"); Serial.println("|");
  Serial.print("\n");
  Serial.print("Valeurs");              Serial.print("|");
  Serial.print(millis());               Serial.print("|");
  Serial.print(coords.a_latitude );     Serial.print("|");
  Serial.print(coords.a_longitude);     Serial.print("|");
  Serial.print(coords.validGPS_signal); Serial.print("|");

  Serial.print(essais);                 Serial.print("|");
  Serial.print(succes);                 Serial.print("|");

  Serial.print(AccelerationSysteme);    Serial.print("|");
  Serial.print(VeloShaked.Secousses);   Serial.print("|");

  Serial.print(AccelX);                 Serial.print("|");
  Serial.print(AccelY);                 Serial.print("|");
  Serial.print(AccelZ);                 Serial.print("|");

  Serial.println();

  if (MiseEnVeille.finished()) {
    LowPower.sleep(1000 * 60 * VEILLE_MIN); // mise en veille du système pour VEILLE_MIN
  }
}
