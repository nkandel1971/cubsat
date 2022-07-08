
#include <Wire.h>            // I2C Library
#include <QMC5883LCompass.h> // QMC5883L GY271 Compass Library
#include <MPU6050_tockn.h>   // MPU6050 Library
#include <Adafruit_BMP085.h> // BMP180 Library
#include "DHT.h"             // DHT sensor Library
#include <ESP8266WiFi.h>     // ESP wifi
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>       // WebSerial print
//definations and varibles                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              
//ESP wifi
const char *WIFI_SSID = "hello world_2.4";
const char *WIFI_PASSWORD = "CLB2765574";
//
//const char *WIFI_SSID = "test";
//const char *WIFI_PASSWORD = "hellosatellite";
AsyncWebServer server(80);
void callback(unsigned char* data, unsigned int length)
{
  data[length] = '\0';
  WebSerial.println((char*) data);
}
//DHT
#define DHTTYPE DHT11   // DHT 11
#define dht_dpin 3
DHT dht(dht_dpin, DHTTYPE);
//BMP180
#define seaLevelPressure_hPa 1013.25
Adafruit_BMP085 bmp;
//MPU_6050
MPU6050 mpu6050(Wire);
long timer = 0;
//Compass
QMC5883LCompass compass;
void setup() {
  //  pinMode(A0, OUTPUT);
  pinMode(15, OUTPUT);
  // Initialize the WebSerial port.
  Serial.begin(115200); //initialize WebSerial monitor
  // Initialize I2C.
  Wire.begin();
  dht.begin(); //Initializing DHT11
  if (!bmp.begin()) { //Initializing BMP180
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {}
  }
  mpu6050.begin();   //Initializing MPU
  mpu6050.calcGyroOffsets(true);
  compass.init();    //Initializing Magnetomete
  //connect to wifi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(WiFi.localIP());
  WebSerial.begin(&server);
  WebSerial.msgCallback(callback);
  server.begin();
}
void loop() {
  //  digitalWrite(A0, 1);
  digitalWrite(15, 1);
  //read DHt
  int dht_humi = dht.readHumidity();
  int dht_temp = dht.readTemperature();
  //read BMP
  int bmp_temp =  bmp.readTemperature();
  int   pressure = bmp.readPressure();
  float my_altitude = bmp.readAltitude();
  int32_t pressure_at_sealevel = bmp.readSealevelPressure();
  int  real_altitude = bmp.readAltitude(seaLevelPressure_hPa * 100);
  //read MPU6050
  float mpu_temp, accX, accY, accZ, gyroX, gyroY, gyroZ, angleX, angleY, angleZ;
  String test;
  mpu6050.update();
  // Read compass values
  compass.read();
  mpu_temp = mpu6050.getTemp();
  accX = mpu6050.getAccX();
  accY = mpu6050.getAccY();
  accZ = mpu6050.getAccZ() * 10;
  gyroX = mpu6050.getGyroX();
  gyroY = mpu6050.getGyroY();
  gyroZ = mpu6050.getGyroZ();
  angleX = mpu6050.getAngleX();
  angleY = mpu6050.getAngleY();
  angleZ = mpu6050.getAngleZ();
  //read compass
  int x = 0, y = 0, z = 0, az = 0;
  x = compass.getX();
  y = compass.getY();
  z = compass.getZ();
  az = compass.getAzimuth();
  String set = "\n ";
  set = "\n"+ String(dht_temp) + ", " + String(dht_humi) + ", " + String(pressure) + ", " + String(real_altitude) + ", " + String(accX) + ", " + String(accY) + ", " + String(accZ) + ", " + String(gyroX) + ", " + String(gyroY) + ", " + String(gyroZ) + ", " + String(angleX) + ", " + String(angleY) + ", " + String(angleZ);
  if (az != 0){set += ", " + String(az);}
  WebSerial.print(set);
}
