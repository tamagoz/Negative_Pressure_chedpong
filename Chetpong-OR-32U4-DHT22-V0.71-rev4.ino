/*
  Modbus-Arduino Example - Lamp (Modbus Serial)
  Copyright by André Sarmento Barbosa
  http://github.com/andresarmento/modbus-arduino
*/


#include <Modbus.h>
#include <ModbusSerial.h>
#include <Wire.h>
#include <Smoothed.h>
#include <Adafruit_Sensor.h>
#include "ETT_PCF8574.h"
#include "DHT.h"

#define DHTPIN 5     // Digital pin connected to the DHT sensor

#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
DHT dht(DHTPIN, DHTTYPE);

// Modbus Registers Offsets (0-9999)
const int LAMP1_COIL = 100;
const int LAMP2_COIL = 101;
const int LAMP3_COIL = 102;
const int LAMP4_COIL = 103;



//const int SWITCH1_ISTS = 100;
//const int SWITCH2_ISTS = 101;

const int SENSOR_IREG1 = 100;
const int SENSOR_IREG2 = 101;
const int SENSOR_IREG3 = 102;
const int SENSOR_IREG4 = 103;

// Used Pins ADC
const int analog_ISOLATE = A4;  // Analog input pin that the potentiometer is attached to
const int analog_ANTE    = A5;  // Analog input pin that the potentiometer is attached to

const int ledPin1 = 6;
const int ledPin2 = 7;

//const int switchPin1 = 5;
//const int switchPin2 = 3;

// ModbusSerial object
ModbusSerial mb;

long ts;

Smoothed <float> mySensor;
Smoothed <float> mySensor2;

float currentSensorValue;
float currentSensorValue1;
float smoothedSensorValueAvg;
float smoothedSensorValueExp;
float outputValue1;        // value read from the po
float outputValue2;        // value read from the po
float randNumber;
float randNumber1;

ETT_PCF8574 master_relay(PCF8574_ID_DEV0);


int coil1 = 0;
int coil2 = 0;
int coil3 = 0;
int coil4 = 0;


int currentButtonState = LOW;         // ค่าสถานะปัจจุบนของปุ่ม
int previousButtonState = LOW;        // ค่าสถานะของปุ่มกดครั้งที่แล้ว
bool isLedOn = false;                 // ค่าสถานะของ led เริ่มต้นเป้นหลอดปิด


void setup() {

  //===============================================================================================
  analogReference(DEFAULT);                                                                       // ADC Reference = +5V
  //===============================================================================================
  dht.begin();
  // Config Modbus Serial (port, speed, byte format)
  mb.config(&Serial1, 9600, SERIAL_8N1, 4);
  // Set the Slave ID (1-247)
  mb.setSlaveId(10);

  Wire.begin();
  Serial.begin(9600);
  Serial1.begin(9600);
  while (!Serial1);

  // Set ledPin mode
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);

  digitalWrite(ledPin1, HIGH);
  digitalWrite(ledPin2, HIGH);
  //delay(48000);

  // Add LAMP1_COIL register - Use addCoil() for digital outputs

  mb.addCoil(LAMP1_COIL);
  mb.addCoil(LAMP2_COIL);
  mb.addCoil(LAMP3_COIL);
  mb.addCoil(LAMP4_COIL);



  ts = millis();

  //Set ledPin mode
  //pinMode(switchPin1, INPUT);
  // Add SWITCH_ISTS register - Use addIsts() for digital inputs
  //mb.addIsts(SWITCH1_ISTS);

  // Add SENSOR_IREG register - Use addIreg() for analog Inputs
  mb.addIreg(SENSOR_IREG1);
  mb.addIreg(SENSOR_IREG2);
  mb.addIreg(SENSOR_IREG3);
  mb.addIreg(SENSOR_IREG4);

  mySensor.begin(SMOOTHED_AVERAGE, 10);
  mySensor2.begin(SMOOTHED_EXPONENTIAL, 10);
  mySensor.clear();

  master_relay.begin(0xFF);
  master_relay.writePin(RELAY_OUT0_PIN, RELAY_OFF);
  master_relay.writePin(RELAY_OUT1_PIN, RELAY_OFF);
  master_relay.writePin(RELAY_OUT2_PIN, RELAY_OFF);
  master_relay.writePin(RELAY_OUT3_PIN, RELAY_OFF);
  master_relay.writePin(RELAY_OUT4_PIN, RELAY_OFF);
  master_relay.writePin(RELAY_OUT5_PIN, RELAY_OFF);
  master_relay.writePin(RELAY_OUT6_PIN, RELAY_OFF);
  master_relay.writePin(RELAY_OUT7_PIN, RELAY_OFF);

}






void loop() {


  // อ่านค่าสถานะของ push switch (ว่ากด หรือ ไม่กด)
  currentButtonState = coil3;
  // ตรวจสอบกรณีที่สถานะปุ่มกดไม่เหมือนครั้งที่แล้วและครั้งนี้สถานะเป็นปล่อย
  // ซึ่งก็คือปุ่มถูกกดค้างไว้และได้ถูกปล่อยออกนั่นเอง
  if ((currentButtonState != previousButtonState) && previousButtonState == HIGH) {
    // สั่งให้ค่า led เป็นตรงกันข้าม (ถ้าเปิดให้ปิด ถ้าปิดให้เปิด)
    isLedOn = !isLedOn;
  }
  // จำค่าการกดปุ่ม ณ ปัจจุบันไว้ เพื่อนำไปใช้เปรียบเทียบครั้งต่อไป
  previousButtonState = currentButtonState;




  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)

  //Serial.print("Temperature :");
  //Serial.print(t, 1);
  //Serial.print(",");
  //Serial.println(" *C");

  //Serial.print("Humidity :");
  //Serial.print(h, 1);
  //Serial.print(",");
  //Serial.println(" %");

  // Read the value from the sensor
  currentSensorValue = analogRead(analog_ISOLATE);
  currentSensorValue1 = analogRead(analog_ANTE);

  // Add the new value to both sensor value stores
  mySensor.add(currentSensorValue);
  mySensor2.add(currentSensorValue1);

  // Get the smoothed values
  smoothedSensorValueAvg = mySensor.get();
  outputValue1 = map(smoothedSensorValueAvg, 0, 1023, 0, 250);

  smoothedSensorValueExp = mySensor2.get();
  outputValue2 = map(smoothedSensorValueExp, 0, 1023, 0, 250);

  // Output the smoothed values to the serial stream. Open the Arduino IDE Serial plotter to see the effects of the smoothing methods.



  //Serial.print(outputValue1);
  //Serial.print(",");
  //Serial.println(outputValue2);

  // If needed we can also return the last stored value which will be unsmoothed
  float lastValueStoredAvg = mySensor.getLast();
  float lastValueStoredExp = mySensor2.getLast();

  // Call once inside loop() - all magic here

  mb.task();

  mb.Coil(LAMP1_COIL);
  mb.Coil(LAMP2_COIL);
  mb.Coil(LAMP3_COIL);
  mb.Coil(LAMP4_COIL);

  coil1 = mb.Coil(LAMP1_COIL);
  coil2 = mb.Coil(LAMP2_COIL);
  coil3 = mb.Coil(LAMP3_COIL);
  coil4 = mb.Coil(LAMP4_COIL);

  if (coil1 == 1) {     //   NEGATIVE PRESSURE ROOM

    //master_relay.writePin(RELAY_OUT0_PIN, RELAY_ON);
    //master_relay.writePin(RELAY_OUT1_PIN, RELAY_ON);
    master_relay.writePin(RELAY_OUT2_PIN, RELAY_OFF);
    master_relay.writePin(RELAY_OUT3_PIN, RELAY_ON);
    master_relay.writePin(RELAY_OUT4_PIN, RELAY_ON);
    master_relay.writePin(RELAY_OUT5_PIN, RELAY_ON);
    master_relay.writePin(RELAY_OUT6_PIN, RELAY_ON);
    master_relay.writePin(RELAY_OUT7_PIN, RELAY_ON);

  }
  else if (coil2 == 1) {  //   OPREATING ROOM
    //master_relay.writePin(RELAY_OUT0_PIN, RELAY_ON);
    //master_relay.writePin(RELAY_OUT1_PIN, RELAY_ON);
    master_relay.writePin(RELAY_OUT2_PIN, RELAY_ON);
    master_relay.writePin(RELAY_OUT3_PIN, RELAY_ON);
    master_relay.writePin(RELAY_OUT4_PIN, RELAY_OFF);
    master_relay.writePin(RELAY_OUT5_PIN, RELAY_ON);
    master_relay.writePin(RELAY_OUT6_PIN, RELAY_ON);
    master_relay.writePin(RELAY_OUT7_PIN, RELAY_ON);
  }

  else {
    //master_relay.writePin(RELAY_OUT0_PIN, RELAY_OFF);
    //master_relay.writePin(RELAY_OUT1_PIN, RELAY_OFF);
    master_relay.writePin(RELAY_OUT2_PIN, RELAY_OFF);
    master_relay.writePin(RELAY_OUT3_PIN, RELAY_OFF);
    master_relay.writePin(RELAY_OUT4_PIN, RELAY_OFF);
    master_relay.writePin(RELAY_OUT5_PIN, RELAY_OFF);
    master_relay.writePin(RELAY_OUT6_PIN, RELAY_OFF);
    master_relay.writePin(RELAY_OUT7_PIN, RELAY_OFF);

  }

  if ((coil3 == 1) && (coil1 == 1 || coil2 == 1))  {
    master_relay.writePin(RELAY_OUT0_PIN, RELAY_ON);
    master_relay.writePin(RELAY_OUT1_PIN, RELAY_ON);
  }
  else {
    master_relay.writePin(RELAY_OUT0_PIN, RELAY_OFF);
    master_relay.writePin(RELAY_OUT1_PIN, RELAY_OFF);
  }

  if (millis() > ts + 500) {
    ts = millis();
    if ( outputValue1 > 125 )  {
      outputValue1 = outputValue1 + 1;
    }
    if ( outputValue2 > 125 )  {
      outputValue1 = outputValue1 + 1;
    }

    mb.Ireg(SENSOR_IREG1, (t));
    mb.Ireg(SENSOR_IREG2, (h));
    mb.Ireg(SENSOR_IREG3, (outputValue1));
    mb.Ireg(SENSOR_IREG4, (outputValue2));
  }

  //Serial.print(mb.Coil(LAMP1_COIL));
  //Serial.print(mb.Coil(LAMP2_COIL));
  //Serial.print(mb.Coil(LAMP3_COIL));
  //Serial.println(mb.Coil(LAMP4_COIL));

  delay(500);

}




//----------------------------------------
