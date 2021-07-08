#include <Arduino.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

#define CONST 3.3 / 4095

// =====================================================================================================

#define WIFI_SSID "Iot"
#define WIFI_PASSWORD "Iot@1111"
#define GPS_RX 16
#define GPS_TX 17
#define SENS_ONE 34 // CO SENSOR
#define SENS_TWO 35 // LPG SENSOR
// ======================================================================================================

#define FIREBASE_HOST "https://i-air-7444a-default-rtdb.firebaseio.com/"
#define FIREBASE_AUTH "xh0YHX9CuohUm8eruJpEnrHEGWWDmpwQlunMSXDU"
#define API_KEY "AIzaSyByMN4xHZdbyTd-jatJfx4uNku5yFAj9CU"
#define FIREBASE_PROJECT_ID "i-air-7444a"
#define USER_EMAIL "geekzilla.iium@gmail.com"
#define USER_PASSWORD "neofox@123"

FirebaseData FData;
FirebaseAuth auth;
FirebaseConfig config;
FirebaseJsonArray Array;
TinyGPSPlus gps;
SoftwareSerial SerialGPS(GPS_RX, GPS_TX);

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "in.pool.ntp.org");

void INIT_WIFI()
{
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("\t[WIFI]\t\t\t\t[STARTING] ");
  //  Loading Animation
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print("\b◢");
    delay(100);
    Serial.print("\b◣");
    delay(100);
    Serial.print("\b◤");
    delay(100);
    Serial.print("\b◥");
    delay(100);
  }
  Serial.println("\b\b\n\t[WIFI]\t\t\t\t[CONNECTED]");
  timeClient.begin();
}

void INTI_ESP()
{
  Serial.begin(115200);
  SerialGPS.begin(9600);
  delay(1000);
  Serial.println("");
  Serial.println("\t[ESP]\t\t\t\t[INIT]");
}

void INIT_FIREBASE()
{
  config.host = FIREBASE_HOST;
  config.api_key = API_KEY;
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  // FData.setBSSLBufferSize(1024, 1024);
}

void setup()
{
  pinMode(SENS_ONE, INPUT);
  pinMode(SENS_TWO, INPUT);
  INTI_ESP();
  INIT_WIFI();
  INIT_FIREBASE();
}

void SET_DATA(double _long, double _lat, double _gas)
{
  Array.clear();
  Array.set("/[0]", _lat);
  Array.set("/[1]", _long);
  Firebase.RTDB.setArray(&FData, "gps", &Array);
  Firebase.RTDB.setDouble(&FData, "gas", _gas);
}

void STORE_DATA(double _long, double _lat, double _gas)
{
  FirebaseJson dict;
  String content;
  String location = "data/" + String(timeClient.getEpochTime());
  dict.set("fields/long/stringValue", String(_long, 4).c_str());
  dict.set("fields/lat/stringValue", String(_lat, 4).c_str());
  dict.set("fields/gas/stringValue", String(_gas, 2).c_str());
  dict.toString(content);
  Firebase.Firestore.createDocument(&FData, FIREBASE_PROJECT_ID, "", location.c_str(), content.c_str());
}

long counter = 0;

void firebase()
{
  double latitude = gps.location.lat();
  double longitude = gps.location.lng();

  // GAS CO
  double gas_1 = analogRead(SENS_ONE) / CONST; // CO
  if (gas_1 < 9)
    gas_1 *= 11.1111;
  else if (gas_1 < 15)
    gas_1 = 100 + ((gas_1 - 9) * 16.6667);
  else if (gas_1 < 30)
    gas_1 = 200 + ((gas_1 - 15) * 6.6667);
  else
    gas_1 = 300 + ((gas_1 - 30) * 10);

  // GAS NO
  double gas_2 = analogRead(SENS_TWO) / CONST;
  if (gas_2 < 0.17)
    gas_2 *= 588.23529;
  else if (gas_2 < 0.6)
    gas_2 = 100 + ((gas_2 - 0.17) * 232.56);
  else if (gas_2 < 1.2)
    gas_2 = 200 + ((gas_2 - 0.6) * 166.667);
  else
    gas_2 = 300 + ((gas_2 - 1.2) * 250);

  //gas_1 = 3.027 * pow(2.718, (1.0698 * gas_1));
  //gas_2 = 26.572 * pow(2.718, (1.2894 * gas_2));
  Serial.print("GAS 1 ");
  Serial.print(gas_1);
  Serial.print("\tGAS 2 ");
  Serial.print(gas_2);
  Serial.print("\tLatitude ");
  Serial.print(latitude);
  Serial.print("\tLongitude ");
  Serial.print(longitude);
  Serial.print("\tSatellites ");
  Serial.print(gps.satellites.value());
  double final_gas = gas_1 > gas_2 ? gas_1 : gas_2;
  if ((counter++) > 100) // change this value to store data in firebase low means very frequent
  {
    STORE_DATA(longitude, latitude, final_gas);
    counter = 0;
  }
  SET_DATA(longitude, latitude, final_gas);
  Serial.println("");
}

void loop()
{
  timeClient.update();
  while (SerialGPS.available() > 0)
  {
    gps.encode(SerialGPS.read());
  }
  firebase();
}
