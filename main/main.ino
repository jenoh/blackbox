//#include <DateTime.h>
#include <Wire.h>
#include <rpcWiFi.h>
#include <MPU6050.h>
#include "Ultrasonic.h"
#include <HTTPClient.h>
#include <SPI.h>
#include <Seeed_FS.h>
#include "SD/Seeed_SD.h"
#include <string>     // std::string, std::to_string


File myFile;

MPU6050 mpu;

//const char* ssid = "iPhone X";
//const char* password =  "iotiotiot";
//const char* serverName = "http://d273-163-5-2-42.ngrok.io";

unsigned long lastTime = 0;
unsigned long timerDelay = 5000;

const int shockSensorPin = 4;

bool shockSensorSate = 0;

// Définition des numéros de port
const int trigPin = 2;  // Trigger (emission)
const int echoPin = 3;  // Echo    (réception)

// Variables utiles
long duree;   // durée de l'echo
int distance; // distance

void setup()
{
  pinMode(WIO_MIC, INPUT);
  pinMode(shockSensorPin, INPUT);
  shockSensorSate = digitalRead(shockSensorPin);

  pinMode(trigPin, OUTPUT); // Configuration du port du Trigger comme une SORTIE
  pinMode(echoPin, INPUT);  // Configuration du port de l'Echo  comme une ENTREE


  Serial.begin(115200);
  while (!Serial); // Wait for Serial to be ready

  //initWifi();


  // Initialize MPU6050
  Serial.println("Initialize MPU6050");
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(1000);
  }

  mpu.calibrateGyro();

  mpu.setThreshold(3);

  checkSettings();

  checkSd();
}

void checkSd() {
  Serial.print("Initializing SD card...");
  if (!SD.begin(SDCARD_SS_PIN, SDCARD_SPI)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");
}

void checkSettings()
{
  Serial.println();

  Serial.print(" * Sleep Mode:        ");
  Serial.println(mpu.getSleepEnabled() ? "Enabled" : "Disabled");

  Serial.print(" * Clock Source:      ");
  switch (mpu.getClockSource())
  {
    case MPU6050_CLOCK_KEEP_RESET:     Serial.println("Stops the clock and keeps the timing generator in reset"); break;
    case MPU6050_CLOCK_EXTERNAL_19MHZ: Serial.println("PLL with external 19.2MHz reference"); break;
    case MPU6050_CLOCK_EXTERNAL_32KHZ: Serial.println("PLL with external 32.768kHz reference"); break;
    case MPU6050_CLOCK_PLL_ZGYRO:      Serial.println("PLL with Z axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_YGYRO:      Serial.println("PLL with Y axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_XGYRO:      Serial.println("PLL with X axis gyroscope reference"); break;
    case MPU6050_CLOCK_INTERNAL_8MHZ:  Serial.println("Internal 8MHz oscillator"); break;
  }

  Serial.print(" * Gyroscope:         ");
  switch (mpu.getScale())
  {
    case MPU6050_SCALE_2000DPS:        Serial.println("2000 dps"); break;
    case MPU6050_SCALE_1000DPS:        Serial.println("1000 dps"); break;
    case MPU6050_SCALE_500DPS:         Serial.println("500 dps"); break;
    case MPU6050_SCALE_250DPS:         Serial.println("250 dps"); break;
  }

  Serial.print(" * Gyroscope offsets: ");
  Serial.print(mpu.getGyroOffsetX());
  Serial.print(" / ");
  Serial.print(mpu.getGyroOffsetY());
  Serial.print(" / ");
  Serial.println(mpu.getGyroOffsetZ());

  Serial.println();
}
/*
  void initWifi()
  {

  // Set WiFi to station mode and disconnect from an AP if it was previously connected
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  Serial.println("Connecting to WiFi..");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
    WiFi.begin(ssid, password);
  }
  Serial.println("Connected to the WiFi network");
  Serial.print("IP Address: ");
  Serial.println (WiFi.localIP()); // prints out the device's IP address
  }

  void scanWifi()
  {
  // WiFi.scanNetworks will return the number of networks found
  int n = WiFi.scanNetworks();
  Serial.println("scan done");
  if (n == 0) {
    Serial.println("no networks found");
  } else {
    Serial.print(n);
    Serial.println(" networks found");
    for (int i = 0; i < n; ++i) {
      // Print SSID and RSSI for each network found
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" (");
      Serial.print(WiFi.RSSI(i));
      Serial.print(")");
      Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? " " : "*");
      delay(10);
    }
  }
  Serial.println("");
  }*/

int  ultrasons()
{
  // Émission d'un signal de durée 10 microsecondes
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Écoute de l'écho
  duree = pulseIn(echoPin, HIGH);
  // Calcul de la distance
  distance = duree * 0.034 / 2;
  // Affichage de la distance dans le Moniteur Série
  Serial.print("Distance : ");
  Serial.print(distance);
  Serial.println("cm");
  return distance;
}

int niveau_sonore()
{
  int val = analogRead(WIO_MIC);
  Serial.println("Db : ");
  Serial.print(String(val));
  return val;
}

Vector imu()
{
  Vector rawGyro = mpu.readRawGyro();

  Serial.print(" Xraw = ");
  Serial.print(rawGyro.XAxis);
  Serial.print(" Yraw = ");
  Serial.print(rawGyro.YAxis);
  Serial.print(" Zraw = ");
  Serial.println(rawGyro.ZAxis);

  return rawGyro;
}
/*
  void post(int dist, int db, Vector rawGyro)
  {
  if (WiFi.status() == WL_CONNECTED) {
    WiFiClient client;
    HTTPClient http;

    // Your Domain name with URL path or IP address with path
    http.begin(client, serverName);

    // If you need an HTTP request with a content type: application/json, use the following:
    http.addHeader("Content-Type", "application/json");

    int httpResponseCode = http.POST("{\"imu\": \"toto \" }");

    // int httpResponseCode = http.POST("{\"imu\": {\"Xraw\": \""+ std::to_string(rawGyro.XAxis)+"\", \"Yraw\": \""+ std::to_string(rawGyro.YAxis)+"\", \"Zraw\": \""+ std::to_string(rawGyro.ZAxis)+"\"}, {\"ultrasons\": {\"cm\": \""+ std::to_string(dist)+"\"},{\"microphone\": {\"db\": \""+ std::to_string(db)+"\"}}");

    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);

    // Free resources
    http.end();
  }
  else {
    Serial.println("WiFi Disconnected");
  }
  lastTime = millis();
  }
*/
void write_file(int db, int dist, Vector rawGyro) {

  myFile = SD.open("microphone.txt", FILE_APPEND);
  if (myFile)
  {
    myFile.print(String(db));
    myFile.println("");
    myFile.close();
  }
  else
    Serial.println("Error opening microphone.txt");

  myFile = SD.open("distance.txt", FILE_APPEND);
  if (myFile)
  {

    myFile.print(String(dist));
    myFile.println("");
    myFile.close();
  }
  else
    Serial.println("Error opening distance.txt");

  myFile = SD.open("gyro.txt", FILE_APPEND);
  if (myFile)
  {
    Serial.println(String(rawGyro.XAxis));
    myFile.print(String(rawGyro.XAxis)+";"+String(rawGyro.YAxis)+";"+String(rawGyro.ZAxis));
    myFile.println("");
    myFile.close();
  }
  else
    Serial.println("Error opening gyro.txt");
}

void loop()
{

  //Serial.print(shockSensorSate);

  /* Capteur ultrasons */
  int dist = ultrasons();

  /* Microphone */
  int db = niveau_sonore();

  /* Gyroscopre */
  Vector rawGyro = imu();

  /* Post */
  //post(dist, db, rawGyro);

  write_file(db, dist, rawGyro);

  /* Pause de 1 seconde */
  delay(10000);
}
