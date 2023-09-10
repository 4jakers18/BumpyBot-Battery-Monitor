/*********************************************************************
 * 
 * Written by Jacob Tomczeszyn
 * Designed for BumpyBot
 * The Human Centered Robotics Laboratory
 * The University of Texas at Austin
 * 
 * 
 Designed for Adafruit QT Py M0, Reads ADC values at A2 and A3 expecting divided voltage.
 Displays undivided voltage and other derived values on Adafruit 1.12" I2C display.
 Acts as a ROS publisher utilizing rosserial, 
 for msg info, see http://docs.ros.org/en/api/sensor_msgs/html/msg/BatteryState.html

 UTILIZES A CUSTOM splash.h in Adafruit_SH110X library for project Logo (not neccesary)

 UTILIZES A MODIFIED ArduinoHardware.h in ros_lib library for 
 rosserial arduino compatability with the Adafruit QT Py M0: 

 in ros_lib/ArduinoHardware.h, 
 add the following lines between the first "#enddif" and the next "#if defined"

******************************
#if 1
class ArduinoHardware {
  public:
    ArduinoHardware()
    {
      baud_ = 250000;
    }  
    void setBaud(long baud){
      this->baud_= baud;
    }
    int getBaud(){return baud_;}
    void init(){
      Serial.begin(baud_);
      while(!Serial.available());
    }
    int read(){return Serial.read();};
    void write(uint8_t* data, int length){
      for(int i=0; i<length; i++)
        Serial.write(data[i]);
    }
    unsigned long time(){return millis();}
  protected:
    long baud_;
};
#else
******************************

*********************************************************************/

//ROSserial
#include <ros.h>
#include <sensor_msgs/BatteryState.h>
ros::NodeHandle  nh;
sensor_msgs::BatteryState msg;
ros::Publisher battery("battery", &msg);
bool rosInit = false;

//Graphics & Display
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 128 // OLED display height, in pixels
#define OLED_RESET -1     // can set an oled reset pin if desired
Adafruit_SH1107 display = Adafruit_SH1107(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET, 1000000, 100000);

//ADC & Averaging
#define VoltageInput A3
#define TempInput  A2
#define maxAdcValue  4095   // Maximum ADC value for 12-bit ADC
#define referenceVoltage  3.299 // Reference voltage
#define R1  47000 // R1 Approximate Value
#define R2  5901.82 //R2 Actual Value (only one need be tuned, only the ratio is important)
#define sampleInterval  100  // Sample every 100ms
#define averagePeriod  3000 // Average over 2 seconds
float Voltage_Divided = 0.0;
float Voltage_Measured = 0.0;
float Cap_Percent;
bool lowbattery = 0; //Low Battery Flag
bool noADC = 0; // Set true to use TEST values for display and rosserial


//Thermistor Calculation
#define THERMISTORNOMINAL 10000 // resistance at 25 degrees C   
#define TEMPERATURENOMINAL 25 // temp. for nominal resistance (almost always 25 C)
#define BCOEFFICIENT 3892 // The beta coefficient of the thermistor (usually 3000-4000)
#define SERIESRESISTOR 10000 // the value of other thermistor resistor
float Temp_C = 0.0;
float Temp_F = 0.0;

//Charging State Detection
#define CHARGING_VOLTAGE_DIFFERENCE  0.5
#define CHARGING_STOP_THRESHOLD  2
float previousVoltage = 0.00;
bool isCharging = false;
int chargingStopCounter = 0; 



void setup()   {
  analogReference(AR_EXTERNAL);  //Set A1 as Reference (connected to the 3.3V reg.)
  analogReadResolution(12); //enable 12-bit ADC on SAMD21 board
  
  pinMode(VoltageInput, INPUT_PULLUP); //"_PULLUP" neccesary for M0 boards
  pinMode(TempInput, INPUT_PULLUP);


/*Display Setup*/
  delay(250); //Wait for the OLED to power up
  display.setRotation(1);
  display.begin(0x3D, true); //I2C Address 0x3D is default
  display.display(); //Show image buffer (Bumpybot Logo) on the display hardware.
/*End Display Setup*/
  delay(2000); //Display glorius project logo for 2 seconds
}

void loop() {
  if(noADC == 1){
    Voltage_Measured = 27.54;
    Temp_C = 31.5;
    }
    
  else{
  float avgVoltage_Divided, avgTempRes;
  getAveragedReadings(avgVoltage_Divided, avgTempRes);
  Voltage_Measured = (avgVoltage_Divided / (R2/(R1+R2)));
  detectChargingStatus();
  Temp_C = steinhart(avgTempRes);
  }
  
  Temp_F = 32 + (Temp_C * 1.8); //Fahrenheit Conversion (optional, use either Temp_F or Temp_C in draw() if you wish);
  
  Cap_Percent = Battery_Capacity(Voltage_Measured);
  
  if(Cap_Percent >= 0 && Cap_Percent <= 5){ // Low Battery Warning flag
    lowbattery = 1;
    }
  else{
    lowbattery = 0;
    }
  draw();
  ROSmsg_init_write();
}

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

  // Keep sampling for 10 seconds
  while (millis() - startTime < averagePeriod) {
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

//Function to display values on OLED
void draw(void) {
  if(noADC == 1){
    delay(averagePeriod); }
  int offset;
  int capacity_test = 75;
  display.clearDisplay();
  display.setRotation(2);
  display.setTextSize(2);      // Normal 1:1 pixel scale
  display.setTextColor(SH110X_WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
  display.invertDisplay(false);
  
//Low Battery Display Warning

   
  if(isCharging == true){
  display.println("~Charging~");
  display.println(" "); 
  }
  else if(lowbattery == true){
  display.invertDisplay(true);
  display.println("  LOW");
  display.println("BATTERY"); 
  } 

  else{
  display.println("BumpyBot");
  display.println(" "); 
  }

  display.print(" ");
  if(Voltage_Measured < 1){
    display.print("???");
  }
  else{
  display.print(Voltage_Measured,2);
  display.println("V");
  }
  display.println(" ");
  
  if(isCharging == true){
  display.print("  ");
  display.write(0x18); //up arrow symbol on capacity percent
  display.print(" ");
  Cap_Percent -= 3;
  }
  
  else if(lowbattery == true){
  display.print("  ");
  display.write(0x21); // '!' on capacity percent
  display.print(" "); 
  }
  
  else{
  display.print(" "); 
  }
    if(Voltage_Measured < 1){
    display.println("???");
  }
  else{
  display.print(Cap_Percent,0);
  display.println("%");
  }
  display.println(" "); 
  display.print(" ");
  display.print(Temp_F, 1);
  display.println("F");
//replace the above two lines with the following two lines to use Celcius on the display
 // display.print(Temp_C, 1); 
 // display.println("C");
 
//Cap_Percent offset
if(Cap_Percent >= 0){
offset = 100 - Cap_Percent;
}
else{
  offset = 95;
}
//Battery Visual
//vertical
display.fillRoundRect(101,  18,  19,  4,  3,  SH110X_WHITE); 
display.drawRoundRect(93,  20,  35,  108,  10,  SH110X_WHITE);
display.drawRoundRect(94,  21,  33,  106,  8,  SH110X_WHITE);
display.fillRoundRect(98,  24+offset,  25,  100-offset,  5,  SH110X_WHITE);

//horizontal WIP
//display.fillRoundRect(18, 101,  4, 19,  3,  SH110X_WHITE); 
//display.drawRoundRect(20, 93,  35,  108,  10,  SH110X_WHITE);
//display.drawRoundRect(94,  21,  33,  106,  8,  SH110X_WHITE);
//display.fillRoundRect(98,  24+offset,  25,  100-offset,  5,  SH110X_WHITE);
  


  display.display();
  delay(100);
}
