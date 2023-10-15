/********************************************************************
 * 
 * Written by Jacob Tomczeszyn
 * Designed for BumpyBot
 * The Human Centered Robotics Laboratory
 * The University of Texas at Austin
 * 
 Designed for Adafruit QT Py M0, Reads ADC values at A2 and A3 expecting divided voltage.
 Displays undivided voltage and other derived values on Adafruit 1.12" I2C display.
 Acts as a ROS publisher utilizing rosserial, 

 See README.md for more info.
*******************************************************************/

#include "helper_functions.h"
//ROSserial


bool rosInit = false;

//Graphics & Display

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 128 // OLED display height, in pixels
#define OLED_RESET -1     // can set an oled reset pin if desired
Adafruit_SH1107 display = Adafruit_SH1107(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET, 1000000, 100000);

//ADC & Averaging
const int VoltageInput = A3;
const int TempInput = A2;
const int maxAdcValue = 4095;  // Maximum ADC value for 12-bit ADC
const float referenceVoltage = 3.299; // Reference voltage
const float R1 = 47000; // R1 Approximate Value
const float R2 = 5901.82; // R2 Actual Value (only one need be tuned, only the ratio is important)
const unsigned int sampleInterval = 100;  // Sample every 100ms
const unsigned int averagePeriod = 3000; // Average over 3 seconds

float Voltage_Divided = 0.0;
float Voltage_Measured = 0.0;
float Cap_Percent;
bool lowbattery = 0; //Low Battery Flag

bool noADC = 0; // Set true to use TEST values for display and rosserial


//Thermistor Calculation
const int THERMISTORNOMINAL = 10000; // resistance at 25 degrees C
const int TEMPERATURENOMINAL = 25; // temp. for nominal resistance (almost always 25 C)
const int BCOEFFICIENT = 3892; // The beta coefficient of the thermistor (usually 3000-4000)
const int SERIESRESISTOR = 10000; // the value of other thermistor resistor



float Temp_C = 0.0;
float Temp_F = 0.0;

//Charging State Detection
const float CHARGING_VOLTAGE_DIFFERENCE = 0.5;
const int CHARGING_STOP_THRESHOLD = 2;
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
