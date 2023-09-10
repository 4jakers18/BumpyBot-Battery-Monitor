# Bumpybot-Battery-Monitor
Battery Monitoring Device designed for BumpyBot utilizing ROS-Serial and a I2C TFT Display

Designed by Jacob Tomczeszyn for The Human-Centered Robotics Laboratory at The University of Texas at Austin
  
 Designed for Adafruit QT Py M0, Reads ADC values at A2 and A3 expecting divided voltage.
 Displays undivided voltage and other derived values on Adafruit 1.12" I2C display.
 
 Acts as a ROS publisher utilizing rosserial over a USB connection the Bumpybot Computer.
 
For `BatteryState` ROS message documentation see [here](http://docs.ros.org/en/api/sensor_msgs/html/msg/BatteryState.html).


## Hardware 

(To Be Added)

## Setup & Dependencies

- [Follow instructions on setting up Arduino IDE for Adafruit SAMD21 boards](https://learn.adafruit.com/adafruit-qt-py/arduino-ide-setup)
    - It is recommended to use the arduino-cli that's already installed and setup on the Bumpybot computer.

- Either use the precompiled [ros_lib library in this repository](libraries/ros_lib) or follow [instructions on compiling yourself](http://wiki.ros.org/rosserial_arduino).
- [Adafruit GFX Arduino Library](https://github.com/adafruit/Adafruit-GFX-Library)
- [Adafruit SH110X Display Driver Library](https://github.com/adafruit/Adafruit_SH110x)

The ROSserial and Adafruit libraries are also included in this [repository](libraries). 

### If you wish to install/compile them from scratch, Please see the following notes:

#### ros_lib/ArduinoHardware.h
If you self-compile ros_lib, you must modify ros_lib/ArduinoHardware.h to enable ROSserial compatibility with the Adafruit QT Py M0.
Add the following lines between the first `#enddif` and the next `#if defined` near the top of the file.
The precompiled and modified [ros_lib](libraries/ros_lib) is already included in this repository, you can just use it instead of compiling it yourself.

```
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
```
#### Adafruit_SH110X/spash.h 
I've created a custom startup image of the Bumpybot Logo for the display.
This custom file is not necessary but is included in this repository.

