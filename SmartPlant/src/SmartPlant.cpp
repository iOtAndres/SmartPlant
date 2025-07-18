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
void pulseISR();
void testdrawcircle();
void waterPlant();
void screen();
void MQTT_connect();
void adafruitSubscribe();
void adafruitPublish();

SYSTEM_THREAD(ENABLED);

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


float getMoistureReadings; //(int_pin)
float getBMEtemp;
float getBMEhumidity;
float getAirQuality;
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

//dust sensor measurements
unsigned long sampleTime_ms =30000;//sample 30s&nbsp;;
unsigned long lowPulseOccupancy=0;
unsigned long dustStartTime =0;
float ratio = 0;
float concentration = 0;
// variables for pulse timing
unsigned long pulseStartTime =0;
unsigned long duration = 0;
bool pulseComplete = false;

//display
const int OLED_RESET=-1;
Adafruit_SSD1306 display(OLED_RESET);



void setup() {
  Serial.begin(9600);
  pinMode(MOISTUREPIN, INPUT);
  pinMode(DUSTSENSOR,INPUT);
  attachInterrupt(DUSTSENSOR,pulseISR, CHANGE);//call functionpulseisr to pin state
  dustStartTime = millis();
  pinMode(buttonState, OUTPUT);

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


mqtt.subscribe(&waterButtonFeed);  
// draw mulitple circles
testdrawcircle();
display.display();
delay(2000);
display.clearDisplay();
display.display();
//oled 
display.begin(SSD1306_SWITCHCAPVCC, 0x3c);
display.display();
delay(1500);
display.clearDisplay();
display.display();


//bme
bme.begin();
}
void loop() {
//make sure connection to cloud

  MQTT_connect();

tempf = getBMEtemp;
humidRH = getBMEhumidity;
moisture = getMoistureReadings;
quality = getAirQuality;
//moisture loop
if (moisture > 3000 && (millis()-waterTimer)> 6000){
  needWater = true;
  timerStart= true;
  waterTimer = millis();
}

 //water pump code
    if((millis()-lastTime > 6000)) {
    if(mqtt.Update()) {//nest all pubs in there
      tempFeed.publish(tempf);
      inhgFeed.publish(inhg);
      humidRHFeed.publish(humidRH);
      //aqFeed.publish(aqMessages[1]);
      display.printf("Publishing: tempf: %i, inhg: %i,humidRH: %i\n",tempf,inhg,humidRH); 
      display.display();
      delay(1500);
      display.clearDisplay();
      display.display();
      } 
    lastTime = millis();
  }
   waterPlant();

  MQTT_ping();

 Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(100))) { // one while loop nested for all subs.
    //dashboard waterbutton
    if (subscription == &waterButtonFeed) {
      buttonState = atoi((char *)waterButtonFeed.lastread); //atoi = to int better then atof // address as asuch in declaration if 1 or 0.
      if(buttonState) {//turn on pump can do this for bool isteead of calling it buttonstate ==true
      }
    }
  }


    //dashboard dust sensor every 90 seconds
    if(pulseComplete){
      noInterrupts();//prevent change while reading 
      lowPulseOccupancy == duration;
      pulseComplete = false;
      interrupts();
    }
    if((millis()-dustStartTime) >= sampleTime_ms){
    ratio = lowPulseOccupancy/(sampleTime_ms*10.0);//int %
    concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+.62;
    Serial.printf("concentration= %f \n", concentration);
    lowPulseOccupancy == 0;
    dustStartTime = millis();
  } //if sample time == 30s
  getDustTimer;
  }
float getBMEtemp() {

  unsigned int _bmeTimerTemp;
  unsigned int _updateTime = 30000;
  int _temp;
  if (millis()- _bmeTimerTemp > _updateTime){
    _temp = ((bme.readTemperature()*(1.8))+32);
    _bmeTimerTemp = millis();
  }
return _temp = millis();
}
float getBMEhumidity() {

  unsigned int _bmeTimerHumidity;
  unsigned int _updateTime = 30000;
  int _humidity;
  if (millis()- _bmeTimerHumidity > _updateTime){
    _humidity = bme.readHumidity();
    _bmeTimerHumidity = millis();
  }
return _humidity = millis();
}
float getSoilReading(float sensorPin){

  unsigned int _soilTimer;
  unsigned int _updateTimeSoil = 30000;
  int _moisture;
  if (millis()- _soilTimer > _updateTimeSoil){
    _moisture = analogRead(sensorPin);
    _soilTimer = millis();
  }
return _moisture= millis();
}
void waterPlant(){

if(needWater==true){  
 digitalWrite(WATERPUMP, HIGH);
   if (timerStart == true){
     pumpTimer=millis();
     timerStart=false;
   }
    if(millis()-pumpTimer > 300){
      needWater=false;
      if(needWater==false){
        digitalWrite(WATERPUMP, LOW);
        // glowBlue();
        timerStart=true;
      }  
    }
 }
}
void pulseISR (){
  if (digitalRead(DUSTSENSOR) == LOW){
    pulseStartTime = millis();//start timing
  }
  else{
    duration= millis();
    pulseStartTime; //end timing 
    pulseComplete = true;// a  sign that a pulse was measured
  }
}
void screen() {

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    // display.printf("Time is %s\n");
    display.printf("RoomTemp= %i F\n", tempf);
    display.printf("Humidity= %i\n", humidRH);
    display.printf("SoilMoisture= %i\n", moisture);
    display.printf("dust = %f pcs/0.01cf \n",concentration);
    display.display();
}
void testdrawcircle(void) {

  for (int16_t i=0; i<display.height(); i+=2) {
    display.drawCircle(display.width()/2, display.height()/2, i, WHITE);
    display.display();
  }
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

  unsigned int last;
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
void adafruitSubscribe(){
}
void adafruitPublish(){
}
//   currentTime = millis () ;
// if( currentTime - lastTime > 10000 ) {
// 	lastTime = millis();

// float  getAirQuality(){

//   int _airReading;
//   int _quality;
//   unsigned int _airQualityTimer;

//   if(millis()-_airQualityTimer > 3000){
    
//   _quality = sensor.slope();
//   //Serial.print("Sensor value: ");
//   //Serial.println(sensor.getValue());
//   _airReading= sensor.getValue();
//    if (_quality == AirQualitySensor::HIGH_POLLUTION) {
//     Serial.println("High pollution!");
//   }
//   else (_quality == AirQualitySensor::LOW_POLLUTION) {
//     Serial.println("Low pollution!");
//     }
//   else (_quality == AirQualitySensor::FRESH_AIR) {
//     Serial.println("Fresh air.");
//     //glowGreen();
//   }
//    _airQualityTimer = millis();
//   }
//   return _airReading;
// }