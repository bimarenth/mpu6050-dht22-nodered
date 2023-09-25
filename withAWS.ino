#include "secrets.h"
#include "WiFi.h"
#include <WiFiClientSecure.h>
#include <Arduino_JSON.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

WiFiClientSecure net = WiFiClientSecure();
PubSubClient client(net);
Adafruit_MPU6050 mpu;
DHT dht(23, DHT22);

unsigned long publishTime = 0;

void connectAWS() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.println("Connecting to Wi-Fi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // Configure WiFiClientSecure to use the AWS IoT device credentials
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);

  // Connect to the MQTT broker on the AWS endpoint we defined earlier
  client.setServer(AWS_IOT_ENDPOINT, 8883);
  Serial.println("Connecting to AWS IOT");

  while (!client.connect(THINGNAME)) {
    Serial.print(".");
    delay(100);
  }

  if (!client.connected()) {
    Serial.println("AWS IoT Timeout!");
    return;
  }

  // Subscribe to a topic
  Serial.println("AWS IoT Connected!");
}

void dhtPub(float temp, float humi) {
  JSONVar json;
  char buff[2];
  sprintf(buff, "%.1f", humi);
  json["humidity"] = buff;
  sprintf(buff, "%.1f", temp);
  json["temperature"] = buff;

  client.publish("/sensor/dht22", buff);
}

void gyroPub(float x, float y, float z) {
  JSONVar json;
  char buff[3];
  sprintf(buff, "%.fd", x);
  json["gyroX"] = buff;
  sprintf(buff, "%.1f", y);
  json["gyroY"] = buff;
  sprintf(buff, "%1.f", z);
  json["gyroZ"] = buff;

  client.publish("/sensor/mpu6050/gyro", buff);
}

void accelPub(float x, float y, float z) {
  JSONVar json;
  char buff[3];
  sprintf(buff, "%.1f", x);
  json["accelX"] = buff;
  sprintf(buff, "%.1f", y);
  json["accelY"] = buff;
  sprintf(buff, "%.1f", z);
  json["accelZ"] = buff;

  client.publish("/sensor/mpu6050/accel", buff);
}

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10);  // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  connectAWS();
  dht.begin();
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:
      Serial.println("+-2G");
      break;
    case MPU6050_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case MPU6050_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case MPU6050_RANGE_16_G:
      Serial.println("+-16G");
      break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:
      Serial.println("+- 250 deg/s");
      break;
    case MPU6050_RANGE_500_DEG:
      Serial.println("+- 500 deg/s");
      break;
    case MPU6050_RANGE_1000_DEG:
      Serial.println("+- 1000 deg/s");
      break;
    case MPU6050_RANGE_2000_DEG:
      Serial.println("+- 2000 deg/s");
      break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ:
      Serial.println("260 Hz");
      break;
    case MPU6050_BAND_184_HZ:
      Serial.println("184 Hz");
      break;
    case MPU6050_BAND_94_HZ:
      Serial.println("94 Hz");
      break;
    case MPU6050_BAND_44_HZ:
      Serial.println("44 Hz");
      break;
    case MPU6050_BAND_21_HZ:
      Serial.println("21 Hz");
      break;
    case MPU6050_BAND_10_HZ:
      Serial.println("10 Hz");
      break;
    case MPU6050_BAND_5_HZ:
      Serial.println("5 Hz");
      break;
  }

  Serial.println("");
  delay(100);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float dhtTemp = dht.readTemperature();
  float dhtHumi = dht.readHumidity();

  if (millis() - publishTime >= 500) {
    dhtPub(dhtTemp, dhtHumi);
    gyroPub(g.gyro.x, g.gyro.y, g.gyro.z);
    accelPub(a.acceleration.x, a.acceleration.y, a.acceleration.z);

    // Serial.printf("Temperature = %.1f*c \tHumidity = %.1f% \n", dhtTemp, dhtHumi);
    // Serial.printf("X = %.1f \tY = %.1f \tZ = %.1f \n", g.gyro.x, g.gyro.y, g.gyro.z);
    publishTime = millis();
  }

  client.loop();
  if (!client.connected()) {
    connectAWS();
  }
}
