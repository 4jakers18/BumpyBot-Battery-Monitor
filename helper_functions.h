// helper_functions.h
#ifndef HELPER_FUNCTIONS_H
#define HELPER_FUNCTIONS_H
#include <ros.h>
#include <sensor_msgs/BatteryState.h>
#include "Arduino.h"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>



extern bool rosInit;
extern bool lowbattery;
extern bool isCharging;
extern float previousVoltage;
extern int chargingStopCounter;
extern float Voltage_Measured;
extern float Cap_Percent;
extern float Temp_C;
extern float Temp_F;
extern const int VoltageInput;
extern const int TempInput;
extern const int maxAdcValue;
extern const float referenceVoltage;
extern const float R1;
extern const float R2;
extern const unsigned int sampleInterval;
extern const unsigned int averagePeriod;
extern const int THERMISTORNOMINAL;
extern const int TEMPERATURENOMINAL;
extern const int BCOEFFICIENT;
extern const int SERIESRESISTOR;
extern const float CHARGING_VOLTAGE_DIFFERENCE;
extern const int CHARGING_STOP_THRESHOLD;


void ROSmsg_init_write();
void getAveragedReadings(float &avgVoltage_Divided, float &avgTempRes);
void detectChargingStatus();
float steinhart(float &avgTempRes);
float Battery_Capacity(float Voltage_Measured);
void draw(void);

#endif
