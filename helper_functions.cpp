// helper_functions.cpp
#include "Arduino.h"
#include "helper_functions.h"
ros::NodeHandle  nh;
sensor_msgs::BatteryState msg;
ros::Publisher battery("battery", &msg);
void ROSmsg_init_write(){
  if(Serial && !rosInit){    //initialize ROS if it's not already initialized
    nh.initNode();
    nh.advertise(battery);
    rosInit = true;
    }
    
  if(rosInit){
    if(lowbattery == 0){
      msg.power_supply_health = 1; //Battery Health Good (normal)
    }
    else{
      msg.power_supply_health = 3; //Low Battery (docs say dead battery)
    }
    
    if(isCharging == true){
      msg.power_supply_status = 1; //Charging
      }
      
    else{
      msg.power_supply_status = 2; //Discharging
    }

    if(Cap_Percent == 100){
      msg.power_supply_status = 4; //Battery Capacity Full
    }


    msg.power_supply_technology = 2; //LiON Battery Type
    msg.voltage=Voltage_Measured; //Voltage
    msg.design_capacity = 20000; //100% capacity in mAh
    msg.percentage = Cap_Percent; 
    msg.capacity = msg.design_capacity*(Cap_Percent/100); //estimate actual mAh capacity

    msg.temperature = Temp_C; // Temperature (Use Temp_C or Temp_F)
    
    battery.publish(&msg);
    nh.spinOnce();
  }
}

void getAveragedReadings(float &avgVoltage_Divided, float &avgTempRes) {
  unsigned long startTime = millis();
  float sum1 = 0, sum2 = 0;
  int sampleCount = 0;

  while (millis() - startTime < averagePeriod) {  // Keep sampling for 10 seconds
    int VoltageADC_Raw = analogRead(VoltageInput);
    float Voltage_Divided = (VoltageADC_Raw / (float)maxAdcValue) * referenceVoltage;
    sum1 += Voltage_Divided;

    int TempADC_Raw = analogRead(TempInput);
    float TempVoltage = (TempADC_Raw / (float)maxAdcValue) * referenceVoltage;
    sum2 += TempVoltage;

    sampleCount++;
    delay(sampleInterval);  //Wait for the next sample
  }

  avgVoltage_Divided = sum1 / sampleCount; //Send to Undivided function to get average Battery Voltage
  avgTempRes = (SERIESRESISTOR)/((referenceVoltage / (sum2 / sampleCount))-1); //Calculates Resistance of Temp Probe from average voltage, Sent directly to Steinhart function
}

void detectChargingStatus() {
    if (previousVoltage <= 20.0) {
        //Initialize previousVoltage and return.
        previousVoltage = Voltage_Measured;
        return;
    }

    float voltageDifference = Voltage_Measured - previousVoltage;
    if (voltageDifference >= CHARGING_VOLTAGE_DIFFERENCE) {
        isCharging = true;
        chargingStopCounter = 0;
    } else if (voltageDifference < -0.035) {
        chargingStopCounter++;
        if (chargingStopCounter >= CHARGING_STOP_THRESHOLD) {
            isCharging = false;
            chargingStopCounter = 0;
        }
    } else {
        chargingStopCounter = 0;
    }

    //Update the previous voltage for the next cycle
    previousVoltage = Voltage_Measured;
}

float steinhart(float &avgTempRes){ //Steinhart Thermistor Calculation
  float Temp_C;
  Temp_C = (log(avgTempRes / THERMISTORNOMINAL)/BCOEFFICIENT); //  [ln(R/Ro)] ÷ Beta
  Temp_C += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  Temp_C = 1.0 / Temp_C;                 // Invert
  Temp_C -= 273.15;                         //Convert absolute temp to C
  return Temp_C;
}

//Simple battery Cap_Percent (without discharge curve)
float Battery_Capacity (float Voltage_Measured){
  //float Voltage_float = 100*Voltage_Measured;
  //return map(Voltage_float, 2300, 2940, 0, 100);
  float Cap_Percent = map(Voltage_Measured, 23.00, 29.40, 0, 100.0);
  
  if(Cap_Percent >= 96){
    Cap_Percent = 100;
  }
  else if(Cap_Percent < 0){ //nullify negative capacity value
    Cap_Percent = 0;
  }
  return Cap_Percent;
}
