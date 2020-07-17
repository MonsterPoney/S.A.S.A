#include <SD.h>
#define SDcard 53
File logFile;
bool cardPresent = true;
bool isLaunched = false;
int nullAccel = 0;
const String logHeader = "temps;orientationX;orientationY;orientationZ;accelerationX;accelerationY;accelerationZ;altitude";
String fileName = "LOG";

// Bluetooth
#include <SoftwareSerial.h>
#define blueTx 11
#define blueRx 10
SoftwareSerial bluetooth(blueRx,blueTx);

#include <Wire.h>
#include <SPI.h>

// BNO055
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

//BMP280
#include <Adafruit_BMP280.h>
float seaLevelhPa = 1013;

//Maestro
#include <PololuMaestro.h>
#define maestroSerial Serial2
#define chanROLL 0
#define chanYAW 2
#define chanPITCH 1
const int midPITCH = 5940;
const int minPITCH = 5520;
const int maxPITCH = 6360;
const int rangePITCH = (maxPITCH-minPITCH)/2;
double Ytarget=0;
const int midYAW = 5940;
const int minYAW = 5400;
const int maxYAW = 6520;
const int rangeYAW = (maxYAW-minYAW)/2;
double Ztarget=0;
const int midROLL = 5960;
const int minROLL = 4992;
const int maxROLL = 6976;
const int rangeROLL = (maxROLL-minROLL)/2;
double Xtarget=0;
// sensibility in degree
const int sensibility = 1;
MicroMaestro maestro(maestroSerial);

#define chipSelect 53
#define ledR 5
#define ledG 6
#define ledB 7
#define buzzer 12

#include <Adafruit_Sensor.h>
#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)
Adafruit_BMP280 bmp;

//BMP280 var
float bmp_temp, bmp_pressure, bmp_altit;

int iRef = 0;
double ref_orientX = 0, ref_orientY = 0, ref_orientZ = 0;

int iRef2 = 0;
double ref_accelX = 0, ref_accelY = 0, ref_accelZ = 0;


sensors_event_t orientData, accelData;

void beep(String _type = "SHORT", int _iteration = 1, int _delay = 100){
  for(int f=0;f++;f<_iteration){
    digitalWrite(buzzer,HIGH);
    delay((_type == "LONG") ? 300 : 100);
    digitalWrite(buzzer,LOW);
    delay(_delay);
  }
}

void setup() {
  Serial.begin(9600);

  Wire.begin();

  pinMode(ledR,OUTPUT);
  pinMode(ledG,OUTPUT);
  pinMode(ledB,OUTPUT);
  pinMode(buzzer,OUTPUT);
  pinMode(blueRx, INPUT);
  pinMode(blueTx, OUTPUT);
  
  bluetooth.begin(9600);
  delay(500);

  bluetooth.print("SD card test \r\n");
  pinMode(SDcard,OUTPUT);
  if(!SD.begin(SDcard)){ 
    beep("SHORT",2);
    bluetooth.print("SD card not detected \r\n");
    cardPresent = false;
  }else{
    bluetooth.print("SD card OK");
    for(int i = 0;i<200;i++){
      if(!SD.exists(fileName + (i)+".txt")){
        fileName = fileName + i + ".txt";
        bluetooth.print("fileName : " + fileName + "\r\n");
        logFile = SD.open(fileName,FILE_WRITE);
        logFile.println("I (" + getTimestamp() + ") : log start");
        break;
      }
    }
  }

  bluetooth.print("BMP280 test\r\n");
  if (!bmp.begin()) {
    digitalWrite(ledR,HIGH);
    beep("LONG",3);
    bluetooth.print("Could not find a valid BMP280 sensor, check wiring!\r\n");
    if(cardPresent) logFile.println("E : BMP280 sensor not find");
    while (1);
  }else{
    bluetooth.print("BMP280 ok\r\n");
    if(cardPresent)logFile.println("I : BMP280 sensor operational");
  }

  bluetooth.print("Orientation Sensor Test\r\n");
  if (!bno.begin()){
    digitalWrite(ledR,HIGH);
    beep("LONG",3);
    bluetooth.print("No BNO055 detected... Check your wiring or I2C address!\r\n");
    if(cardPresent) logFile.println("E : BNO055 sensor not found");
    while (1);
  }else
    if(cardPresent)logFile.println("I : BNO055 sensor operational");

  adafruit_bno055_offsets_t Calibration;
  Calibration.accel_offset_x = -32;
  Calibration.accel_offset_y = -87;
  Calibration.accel_offset_z = -46;

  Calibration.mag_offset_x = -152;
  Calibration.mag_offset_y = -545;
  Calibration.mag_offset_z = 441;

  Calibration.gyro_offset_x = 0;
  Calibration.gyro_offset_y = -2;
  Calibration.gyro_offset_z = -1;

  Calibration.accel_radius = 1000;
  Calibration.mag_radius = 683;

  bno.setSensorOffsets(Calibration);
  /* Use external crystal for better accuracy */
  bno.setExtCrystalUse(true);
  if(cardPresent)logFile.println("I : BNO055 calibration imported");
  bluetooth.print("BNO055 calibration imported\r\n");

  maestroSerial.begin(9600);
  maestro.setAcceleration(1,0);
  maestro.setAcceleration(2,0);
  maestro.setSpeed(1,0);
  maestro.setSpeed(2,0);
  maestro.setTarget(chanPITCH,midPITCH);
  maestro.setTarget(chanYAW,midYAW);
  maestro.setTarget(chanROLL,midROLL);

   /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
          Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
          Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
          Adafruit_BMP280::FILTER_X16,      /* Filtering. */
          Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */


  digitalWrite(ledR,HIGH);
  delay(300);
  digitalWrite(ledG,HIGH);
  delay(300);
  digitalWrite(ledB,HIGH);
  delay(300);

  logFile.println("BNO temp : " + String(bno.getTemp()) + "BMP temp : " + String(bmp.readTemperature()));
  bluetooth.print("Setup finish\r\n");

   bno.getEvent(&orientData,Adafruit_BNO055::VECTOR_EULER);
   bno.getEvent(&accelData,Adafruit_BNO055::VECTOR_ACCELEROMETER);
// Define orientation default values
    while(iRef < 10){
      ref_orientX = ref_orientX + orientData.orientation.x;
      ref_orientY = ref_orientY + orientData.orientation.y;
      ref_orientZ = ref_orientZ + orientData.orientation.z;
      iRef++;
      delay(200);
      if(iRef >= 10){
        ref_orientX = ref_orientX/10;
        ref_orientY = ref_orientY/10;
        ref_orientZ = ref_orientZ/10;
        bluetooth.print("FINAL orientation: " + String(ref_orientX) + "y: " + String(ref_orientY) + "z: " + String(ref_orientZ) + "\r\n");
      }
    }
// Define acceleration default values
    while(iRef2 < 10){
      ref_accelX = ref_accelX + accelData.orientation.x;
      ref_accelY = ref_accelY + accelData.orientation.y;
      ref_accelZ = ref_accelZ + accelData.orientation.z;
      iRef2++;
      delay(200);
      if(iRef2 >= 10){
        Serial.println("Acceleration -> x: " + String(ref_accelX) + "y: " + String(ref_accelY) + "z: " + String(ref_accelZ));
        ref_accelX = ref_accelX/10;
        ref_accelY = ref_accelY/10;
        ref_accelZ = ref_accelZ/10;
        bluetooth.print("FINAL acceleration: " + String(ref_accelX) + "y: " + String(ref_accelY) + "z: " + String(ref_accelZ) + "\r\n");
      }
    }
  
  if(cardPresent)logFile.println("I : Setup finish \r\n"+ logHeader);
  if(cardPresent)logFile.flush();
  
  beep("SHORT",1);
  beep("LONG",1);
  ledsState(false);
}

void loop() {
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  
  if (system > 1){
    bno.getEvent(&orientData,Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&accelData,Adafruit_BNO055::VECTOR_ACCELEROMETER);
  
    double orientX = orientData.orientation.x - ref_orientX;
    double orientY = orientData.orientation.y - ref_orientY;
    double orientZ = orientData.orientation.z - ref_orientZ;
  
    double accelX = accelData.orientation.x - ref_accelX;
    double accelY = accelData.orientation.y - ref_accelY;
    double accelZ = accelData.orientation.z - ref_accelZ;

    if (accelZ > 1){
      isLaunched = true;
      ledsState(false);
    }
    if (isLaunched){
      
      if (abs(accelZ) <0.5)
        nullAccel++;
      else
        nullAccel = 0;
      if (nullAccel >= 10)
        safeState();
        
      logFile.print(getTimestamp() + String(orientX) + ';' + String(orientY) + ';' + String(orientZ) + ';' + String(accelX) + ';' + String(accelY) + ';' + String(accelZ) + ';');

      if(orientY > 2){
        if(orientY>=sensibility)orientY=sensibility;
        // angle % on 45° || (range * R%)/100 || add min range
        Ytarget = (rangePITCH*((orientY*100)/sensibility))/100+midPITCH;
        maestro.setTarget(chanPITCH,Ytarget);
      }else if(orientY < -2){
        orientY = abs(orientY);
        if(orientY>=sensibility)orientY=sensibility;
        // angle % on 45° || (range * R%)/100 || add min range
        Ytarget = abs((rangePITCH*((orientY*100)/sensibility))/100-midPITCH);
        maestro.setTarget(chanPITCH,Ytarget);
      }else{
        maestro.setTarget(chanPITCH,midPITCH);
      }
      if(orientZ > 2){
        if(orientZ>=sensibility)orientZ=sensibility;
        Ztarget = (rangeYAW*((orientZ*100)/sensibility))/100+midYAW;
        maestro.setTarget(chanYAW,Ztarget);
      }else if (orientZ<-2){
        orientZ=abs(orientZ);
       if(orientZ>=sensibility)orientZ=sensibility;
        Ztarget = abs((rangeYAW*((orientZ*100)/sensibility))/100-midYAW);
        maestro.setTarget(chanYAW,Ztarget);
      }else{
        maestro.setTarget(chanYAW,midYAW);
      }
      if(orientX>2){
        if(orientX>=sensibility)orientX=sensibility;
        Xtarget = (rangeROLL*((orientX*100)/sensibility)/100+midROLL);
        maestro.setTarget(chanROLL,Xtarget);
      }else if (orientX<-2){
        orientX=abs(orientX);
        if(orientX>=sensibility)orientX=sensibility;
        Xtarget = abs(rangeROLL*((orientX*100)/sensibility)/100-midROLL);
        maestro.setTarget(chanROLL,Xtarget);
      }else{
        maestro.setTarget(chanROLL,midROLL);
      }

      logFile.println(String(bmp.readAltitude(seaLevelhPa)) + ";");
    
      logFile.flush();
      delay(25);
    }
  }else{
    Serial.println(F("W : Bad calibration, data dumped"));
    if(cardPresent)logFile.println("Bad;calibration;,;data;dumped");
  }
}

String getTimestamp(){
  String t_min = (String) (micros()/60000000);
  String t_sec =(String) (micros()%60000000/1000000);
  String t_ms =(String) (micros()%1000000/1000);
  return t_min + ':' + t_sec + '.' + t_ms + ';';
}

void ledsState(bool _on){
  if(_on){
    digitalWrite(ledR,HIGH);
    digitalWrite(ledG,HIGH);
    digitalWrite(ledB,HIGH);
  }else{
    digitalWrite(ledR,LOW);
    digitalWrite(ledG,LOW);
    digitalWrite(ledB,LOW);
  }
}

void safeState(){
  ledsState(true);
  
  while (1){
    digitalWrite(buzzer,HIGH);
    delay(200);
    digitalWrite(buzzer,LOW);
    delay(100);
    digitalWrite(buzzer,HIGH);
    delay(200);
    digitalWrite(buzzer,LOW);
    delay(2000);
  }
}
