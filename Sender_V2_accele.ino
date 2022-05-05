//#include <Wire.h>  // Only needed for Arduino 1.6.5 and earlier
#include "SSD1306.h" // alias for `#include "SSD1306Wire.h"`
#include <SPI.h>
#include <LoRa.h>
#include<Arduino.h>
#include "TempHumid.h"

//Servo
#include <Servo.h>
Servo myservo;
//Servo

#define DHTPIN 17
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321


DHT dht(DHTPIN, DHTTYPE); //DHT22


//GPS
#include <TinyGPS++.h>
TinyGPSPlus gps;
//GPS

//SSD1306 display(0x3c, 4, 15);

//LoRa.setPins(SS,RST,DI0);
#define SS 18
#define RST 14
#define DI0 26
#define BAND 923000000.00//#define BAND 434500000.00
#define spreadingFactor 9
#define SignalBandwidth 125E3  //#define SignalBandwidth 31.25E3
#define preambleLength 8
#define codingRateDenominator 8
//LoRa

//CO2
#define MG_PIN (12) //define which analog input channel you are going to use
#define DC_GAIN (8.5) //define the DC gain of amplifier
#define READ_SAMPLE_INTERVAL (50) //define how many samples you are going to take in normal operation
#define READ_SAMPLE_TIMES (5) //define the time interval(in milisecond) between each samples in
#define ZERO_POINT_VOLTAGE (0.388) //define the output of the sensor in volts when the concentration of CO2 is 400PPM
#define REACTION_VOLTGAE (0.030) //define the voltage drop of the sensor when move the sensor from air into 1000ppm CO2
//CO2


//------Variable------

//GY86 Accelerometer

//GY86 Accelerometer

int counter = 0;
//CO2 Var
float CO2Curve[3] = {2.602, ZERO_POINT_VOLTAGE, (REACTION_VOLTGAE / (2.602 - 3))};
int percentage;
float volts;
//CO2 Var
//DHT22
float humi, tempc, tempf;
//DHT22
//GY86 Accelerometer
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69

MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
//GY86 Accelerometer

void setup() {
  //Servo
  myservo.attach(13);
  //Servo


  //DHT
  dht.begin();

  //DHT

  pinMode(25, OUTPUT); //Send success, LED will bright 1 second
  pinMode(16, OUTPUT);
  digitalWrite(16, LOW); // set GPIO16 low to reset OLED
  delay(50);
  digitalWrite(16, HIGH);

  
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 36, 37); //Rx=36,Tx=37 heltecLora32
  //GY86 Accelerometer

  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  // initialize serial I2C communication
  // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
  // it's really up to you depending on your project)

  //Serial.begin(115200);
  // initialize device


  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  //GY86 Accelerometer

  while (!Serial); //If just the the basic function, must connect to a computer

  //Initialising the UI will init the display too.
  /*display.init();
    display.flipScreenVertically();
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawString(5,5,"LoRa Sender");
    display.display();*/

  SPI.begin(5, 19, 27, 18);
  LoRa.setPins(SS, RST, DI0);
  Serial.println("LoRa Sender");
  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  Serial.print("LoRa Spreading Factor: ");
  Serial.println(spreadingFactor);
  LoRa.setSpreadingFactor(spreadingFactor);

  Serial.print("LoRa Signal Bandwidth: ");
  Serial.println(SignalBandwidth);
  LoRa.setSignalBandwidth(SignalBandwidth);

  LoRa.setCodingRate4(codingRateDenominator);

  LoRa.setPreambleLength(preambleLength);

  Serial.println("LoRa Initial OK!");
  /*display.drawString(5,20,"LoRa Initializing OK!");
    display.display();*/

  //DHT
  tempc, tempf = readTempAndHumid();
  //DHT

  delay(2000);


}

void loop() {

  //GY86 Accelerometer
  // read raw accel/gyro measurements from device
  readAccel(&ax, &ay, &az, &gx, &gy, &gz);
  Serial.print("Accel Z:");
  Serial.print(az); Serial.println("\t");
  //GY86 Accelerometer

  //DHT
  tempc, tempf = readTempAndHumid();
  //DHT


  co2_percentage = readCO2();

  while (Serial2.available()) {
    if (gps.encode(Serial2.read())) {
      Serial.println("Satellite Count:");
      Serial.println(gps.satellites.value());
      Serial.println("Latitude:");
      Serial.println(gps.location.lat(), 6);
      Serial.println("Longitude:");
      Serial.println(gps.location.lng(), 6);
      Serial.println("Speed MPH:");
      Serial.println(gps.speed.mph());
      Serial.println("Altitude Meters:");
      Serial.println(gps.altitude.meters());
      Serial.println("");
    }

  }
  /*
    //DTH22 to Serial
    Serial.print("Humidity: ");
    Serial.print(humi);
    Serial.print(" Temperature: ");
    Serial.print(tempc);
    Serial.print("°C ");
    Serial.print(tempf);
    Serial.println("°F");
    //DTH22  */


  //OLED
  /* display.clear();
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 5, "JOJOH SENDING ");
    display.drawString(50, 30, String(tempc)+"°C");
    display.display();*/

  // send packet
  LoRa.beginPacket();
  /* LoRa.print("T ");
    LoRa.print(tempc);
    LoRa.print("°c");
    LoRa.print(" H ");
    LoRa.print(gps.altitude.meters());
    LoRa.print("m");*/
  //LoRa.print(" C ");
  LoRa.print(" accel Z ");
  LoRa.print(az);
  counter++;
  LoRa.endPacket();


  //digitalWrite(300, HIGH); // turn the LED on (HIGH is the voltage level)
  delay(50); // wait for a second
  //digitalWrite(300, LOW); // turn the LED off by making the voltage LOW
  /*
     myservo.write(0);
     delay(1000);
     myservo.write(180);
     delay(1000);
  */
}
