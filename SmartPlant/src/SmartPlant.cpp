/* 
 * Project myProject
 * Author: Your Name
 * Date: 7/16/25
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */
// Include Particle Device OS APIs
#include "Particle.h"
#include"IoTTImer.h"
#include"Adafruit_SSD1306.h"

#include"Adafruit_GFX.h"

#include "Colors.h"
#include "neopixel.h"
#include "Adafruit_BME280.h"

#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
#include "Adafruit_MQTT/Adafruit_MQTT.h"
#include "credentials.h"
#include "math.h"

SYSTEM_MODE(AUTOMATIC);
/************ Global State (you don't need to change this!) ***   ***************/ 
TCPClient TheClient; 
// Run the application and system concurrently in separate threads
// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details. 
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 
/****************************** Feeds ***************************************/ 
// Setup Feeds to publish or subscribe 
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname> 
Adafruit_MQTT_Subscribe waterButtonFeed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/waterButtonFeed"); 
//Adafruit_MQTT_Publish pubFeed1 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/randomNumber");
Adafruit_MQTT_Publish tempFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/tempf");
Adafruit_MQTT_Publish inhgFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/inhg");
Adafruit_MQTT_Publish humidRHFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humidRH");
Adafruit_MQTT_Publish dSFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/dustSensor");
/************Declare Functions*************/
void MQTT_connect();
bool MQTT_ping();

void testdrawcircle(void);

//SYSTEM_THREAD(ENABLED);

//pins
const int DUSTSENSOR = A2;
const int AIRQUALITY = A1;
const int MOISTUREPIN = A0;//soil sensor 
const int WATERPUMP = D11;
//neolight
// const int NEOPIXRING= D2;
// int startpixel, endpixel;
// int pixelsLit;
// int bri= 200;
//class


int getMoistureReadings; //(int_pin)
int getBMEtemp;
int getBMEhumidity;
int getAirQuality;
Adafruit_BME280 bme;

struct Plantstatus {
  // float tempf;
  // float inhg;
  // float humidRH;
  // bool stat;
  // const int HEXADDY = 0x76;
};
 int tempf;
 int inhg;
 int humidRH;
 int moisture;
 int quality;
 bool status;
 const int HEXADDY = 0x76;
// Plantstatus myPlant;
// Plantstatus myPlant [4];
bool buttonState;
bool manualReading;
bool CheckAirState;

//class 
// IoTTimer relay;

//timers
unsigned int currentTime, startTime, lastTime,last;
// char dS []= {"good dust quality","medium air quality","bad dust quality"}
int previousMillis = 0;
const int interval = 1000;
int currentMillis;
bool needWater = false;
bool timerStart = false;
unsigned int getDustTimer,waterButtonTimer,waterTimer,pumpTimer;
unsigned int publishTime;

//dust sensor
unsigned int duration, dustStartTime;
unsigned int sampleTime_ms =2000;//sampe 30s&nbsp;;
unsigned int lowPulseOccupancy=0;
float ratio = 0;
float concentration = 0;

//display
const int OLED_RESET=-1;
Adafruit_SSD1306 display(OLED_RESET);



void setup() {
  pinMode(MOISTUREPIN, INPUT);
  pinMode(DUSTSENSOR,INPUT);
  pinMode(buttonState, OUTPUT);
Serial.begin(9600);
waitFor(Serial.isConnected,10000);

// Connect to Internet but not Particle Cloud
WiFi.on();
WiFi.connect();
while(WiFi.connecting()) {
  Serial.printf(".");
  display.display();
  delay(2000);
  display.clearDisplay();
}


// //getting data from BME280
// bool status;
// bool relay;
// status = bme.begin(HEXADDY);
// if (status == false) {
// Serial.printf("BME280 at address 0x%02 X failed to start", HEXADDY);
// relay.startTimer(500);
// }



// Serial.printf("\n\n");
// myplant.tempf;
// myPlant.inhg;
// myplant.humidRH;
// }

mqtt.subscribe(&waterButtonFeed);  
// draw mulitple circles
testdrawcircle();
display.display();
delay(2000);
display.clearDisplay();

//oled 
display.begin(SSD1306_SWITCHCAPVCC, 0x3c);
display.display;
delay(1500);
display.clearDisplay();
display.display;


//bme
bme.begin();
}
void loop() {
//make sure connection to cloud
  MQTT_connect();
  MQTT_ping();
 Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(100))) { // one while loop nested for all subs.
    //dashboard waterbutton
    if (subscription == &waterButtonFeed) {
      buttonState = atoi((char *)waterButtonFeed.lastread); //atoi = to int better then atof // address as asuch in declaration if 1 or 0.
      if(buttonState) {//turn on pump can do this for bool isteead of calling it buttonstate ==true
      }
    }
     if((millis()-lastTime > 6000)) {
    if(mqtt.Update()) {//nest all pubs in there
      tempFeed.publish(tempf);
      inhgFeed.publish(inhg);
      humidRHFeed.publish(humidRH);
      //aqFeed.publish(aqMessages[1]);
      display.printf("Publishing: tempf: %i, inhg: %i,humidRH: %i\n",tempf,inhg,humidRH); 
      } 
    lastTime = millis();
  }
    //dashboard dust sensor 
  }
tempf = getBMEtemp;
humidRH = getBMEhumidity;
moisture = getMoistureReadings;
quality = getAirQuality;
//   currentTime = millis () ;
// if( currentTime - lastTime > 10000 ) {
// 	lastTime = millis();
//    display.printf("Publishing: tempf: %f, inhg: %f,humidRH: %f\n",tempf,inhg,humidRH); 
//     // createEventPayLoad (myPlant);
//       } 
//loop for moisturePIn
// analogRead moisturePin; 
// //BME loop
// if (relay.isTimerReady()){
// ///
// tempf = bme.readTemperature (); // deg C
// pressPA  = bme.readPressure (); //pascals 
// humidRH = bme.readHumidity (); //%RH
// //convert to other temp and pressure

// //when two ints chnage 9 to 9.0 =float
// tempf= ((tempc*9.0/5)+32);
// inhg = (pressPA*.0003);
// //Serial.printf(%.1f,);
// //relay.startTimer(500);

// //Serial.print("*celciuis");
// relay.startTimer(500);
// }
}
void MQTT_connect() {
  int8_t ret;
  if (mqtt.connected()) {
    return;
  }
  Serial.print("Connecting to MQTT... ");
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.printf("Error Code %s\n",mqtt.connectErrorString(ret));
       Serial.printf("Retrying MQTT connection in 5 seconds...\n");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds and try again
  }
  Serial.printf("MQTT Connected!\n");
}
bool MQTT_ping() {
  static unsigned int last;
  bool pingStatus;
  if ((millis()-last)>120000) {
      Serial.printf("Pinging MQTT \n");
      pingStatus = mqtt.ping();
      if(!pingStatus) {
        Serial.printf("Disconnecting \n");
        mqtt.disconnect();
      }
      last = millis();
  }
  return pingStatus;
}

// void createEventPayLoad (Plantstatus){
// //356 bytes
//   JsonWriterStatic <256> jw;
//   {
//   JsonWriterAutoObject obj (& jw );
//   jw.insertKeyValue ("tempf", tempf);
//   jw.insertKeyValue ("inhg", inhg);
//   jw.insertKeyValue ("humidRH", humidRH);
//   float tempf;
//   float inhg;
//   float humidRH;
//   }
// pubFeed.publish (jw.getBuffer());

void testdrawcircle(void) {
  for (int16_t i=0; i<display.height(); i+=2) {
    display.drawCircle(display.width()/2, display.height()/2, i, WHITE);
    display.display();
  }
}
// void waterPlant();
