// Distributed with a free-will license.
// Use it any way you want, profit or free, provided it fits in the licenses of its associated works.
// MAX44009
// This code is designed to work with the MAX44009_I2CS I2C Mini Module available from ControlEverything.com.
// https://www.controleverything.com/products

// Your sketch must #include this library, and the Wire library
// (Wire is a standard library included with Arduino):

//Library for the TSL2561 sensor
#include <SparkFunTSL2561.h>
//Library to send information bya I2C
#include <Wire.h>
//Library for the BMP_180 sensor
#include <BMP180.h>

// Store an instance of the BMP180 sensor.
BMP180 barometer;
// We are going to use the on board LED for an indicator.
int indicatorLed = 13; 
float seaLevelPressure = 101325;

// Create an SFE_TSL2561 object, here called "light":
SFE_TSL2561 light;

// Global variables:
boolean gain;     // Gain setting, 0 = X1, 1 = X16;
unsigned int ms;  // Integration ("shutter") time in milliseconds

// MAX44009 I2C address is 0x4A(74)
#define Addr 0x4A

void setup()
{
  //Comunication with MAX44009 sensor
  // Initialise I2C communication as MASTER
  Wire.begin();
  // Initialise serial communication, set baud rate = 9600
  Serial.begin(9600);

  // Start I2C Transmission
  Serial.println("Comunicacion con MAX44009");
  Wire.beginTransmission(Addr);
  // Select configuration register
  Wire.write(0x02);
  // Continuous mode, Integration time = 800 ms
  Wire.write(0x40);
  // Stop I2C transmission
  Wire.endTransmission();
  delay(300);

  //Comunication with TSL2561 sensor
  Serial.println("TSL2561 example sketch");

  // Initialize the SFE_TSL2561 library

  // You can pass nothing to light.begin() for the default I2C address (0x39),
  // or use one of the following presets if you have changed
  // the ADDR jumper on the board:
  
  // TSL2561_ADDR_0 address with '0' shorted on board (0x29)
  // TSL2561_ADDR   default address (0x39)
  // TSL2561_ADDR_1 address with '1' shorted on board (0x49)

  // For more information see the hookup guide at: https://learn.sparkfun.com/tutorials/getting-started-with-the-tsl2561-luminosity-sensor

  light.begin();

  // Get factory ID from sensor:
  // (Just for fun, you don't need to do this to operate the sensor)

  unsigned char ID;
  
  if (light.getID(ID))
  {
    Serial.print("Got factory ID: 0X");
    Serial.print(ID,HEX);
    Serial.println(", should be 0X5X");
  }
  // Most library commands will return true if communications was successful,
  // and false if there was a problem. You can ignore this returned value,
  // or check whether a command worked correctly and retrieve an error code:
  else
  {
    byte error = light.getError();
    printError(error);
  }

  // The light sensor has a default integration time of 402ms,
  // and a default gain of low (1X).
  
  // If you would like to change either of these, you can
  // do so using the setTiming() command.
  
  // If gain = false (0), device is set to low gain (1X)
  // If gain = high (1), device is set to high gain (16X)

  gain = 0;

  // If time = 0, integration will be 13.7ms
  // If time = 1, integration will be 101ms
  // If time = 2, integration will be 402ms
  // If time = 3, use manual start / stop to perform your own integration

  unsigned char time = 2;

  // setTiming() will set the third parameter (ms) to the
  // requested integration time in ms (this will be useful later):
  
  Serial.println("Set timing...");
  light.setTiming(gain,time,ms);

  // To start taking measurements, power up the sensor:
  
  Serial.println("Powerup...");
  light.setPowerUp();
  
  // The sensor will now gather light during the integration time.
  // After the specified time, you can retrieve the result from the sensor.
  // Once a measurement occurs, another integration period will start.

  //Comunication with BMP180 sensor
  // We start the I2C on the Arduino for communication with the BMP180 sensor.
  Wire.begin();
  // Set up the Indicator LED.
  pinMode(indicatorLed, OUTPUT);
  // We create an instance of our BMP180 sensor.
  barometer = BMP180();
  // We check to see if we can connect to the sensor.
  if(barometer.EnsureConnected())
  {
    Serial.println("Connected to BMP180."); // Output we are connected to the computer.
    digitalWrite(indicatorLed, HIGH); // Set our LED.
    
     // When we have connected, we reset the device to ensure a clean start.
    barometer.SoftReset();
    // Now we initialize the sensor and pull the calibration data.
    barometer.Initialize();
  }
  else
  { 
    Serial.println("No sensor found.");
    digitalWrite(indicatorLed, LOW); // Set our LED.
  }
  
}
//MAX44009 measurement
void MAX44009_Measurement(int type){
  unsigned int data[2];

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select data register
  Wire.write(0x03);
  // Stop I2C transmission
  Wire.endTransmission();

  // Request 2 bytes of data
  Wire.requestFrom(Addr, 2);

  // Read 2 bytes of data
  // luminance msb, luminance lsb
  if (Wire.available() == 2)
  {
    data[0] = Wire.read();
    data[1] = Wire.read();
  }

  // Convert the data to lux
  int exponent = (data[0] & 0xF0) >> 4;
  int mantissa = ((data[0] & 0x0F) << 4) | (data[1] & 0x0F);
  float luminance = pow(2, exponent) * mantissa * 0.045;

  if(type==0){
    // Output data to serial monitor
    Serial.print("Ambient Light luminance :");
    Serial.print(luminance);
    Serial.println(" lux");
    Serial.println();
  }
  else{
    Serial.print(luminance);
    Serial.println();
  }
  //delay(300);
}

//Print error with I2C comunication
void printError(byte error)
  // If there's an I2C error, this function will
  // print out an explanation.
{
  Serial.print("I2C error: ");
  Serial.print(error,DEC);
  Serial.print(", ");
  
  switch(error)
  {
    case 0:
      Serial.println("success");
      break;
    case 1:
      Serial.println("data too long for transmit buffer");
      break;
    case 2:
      Serial.println("received NACK on address (disconnected?)");
      break;
    case 3:
      Serial.println("received NACK on data");
      break;
    case 4:
      Serial.println("other error");
      break;
    default:
      Serial.println("unknown error");
  }
}

//TSL2561 sensor messurement
void TSL2561_Measurement(int type){
  // Wait between measurements before retrieving the result
  // (You can also configure the sensor to issue an interrupt
  // when measurements are complete)
  
  // This sketch uses the TSL2561's built-in integration timer.
  // You can also perform your own manual integration timing
  // by setting "time" to 3 (manual) in setTiming(),
  // then performing a manualStart() and a manualStop() as in the below
  // commented statements:
  
  // ms = 1000;
  // light.manualStart();
  delay(ms);
  // light.manualStop();
  
  // Once integration is complete, we'll retrieve the data.
  
  // There are two light sensors on the device, one for visible light
  // and one for infrared. Both sensors are needed for lux calculations.
  
  // Retrieve the data from the device:

  unsigned int data0, data1;
  
  if (light.getData(data0,data1))
  {
    // getData() returned true, communication was successful
   
    // To calculate lux, pass all your settings and readings
    // to the getLux() function.
    
    // The getLux() function will return 1 if the calculation
    // was successful, or 0 if one or both of the sensors was
    // saturated (too much light). If this happens, you can
    // reduce the integration time and/or gain.
    // For more information see the hookup guide at: https://learn.sparkfun.com/tutorials/getting-started-with-the-tsl2561-luminosity-sensor
  
    double lux;    // Resulting lux value
    boolean good;  // True if neither sensor is saturated
    
    // Perform lux calculation:

    good = light.getLux(gain,ms,data0,data1,lux);
    
    // Print out the results:
    if(type==0){
       Serial.print("data0: ");
      Serial.print(data0);
      Serial.print(" data1: ");
      Serial.print(data1);
      Serial.print(" lux: ");
      Serial.print(lux);
      if(good) Serial.println(" (good)"); else Serial.println(" (BAD)");
      Serial.println();
    }  
    else{
      Serial.print(lux);
      Serial.println();
    }
  }
  else{
    // getData() returned false because of an I2C error, inform the user.
    byte error = light.getError();
    printError(error);
  }
}

//Test how mnay I2C sensors are connected
void Test_I2C(){
  byte error, address;
  int nDevices;
 
  Serial.println("Scanning...");
 
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
 
      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
 
  delay(5000);           // wait 5 seconds for next scan
}

//Sensor BMP180 measurement
void BMP_180_Measurement(int type){
  if(barometer.IsConnected)
  {
    // Retrive the current pressure in Pascals.
    long currentPressure = barometer.GetPressure();
    // Retrive the current altitude (in meters). Current Sea Level Pressure is required for this.
    float altitude = barometer.GetAltitude(seaLevelPressure);
    // Retrive the current temperature in degrees celcius.
    float currentTemperature = barometer.GetTemperature();
    
    //Retrive the current luminosity
    //int Lum = analogRead(analogPin_0);
    //Luminosidad = Lum*4.95/1023.0;

    if(type==0){
      // Print out the Pressure.
      Serial.print("Pressure: ");
      Serial.print(currentPressure);
      Serial.print(" Pa");
  
       // Print out the Luminosity.
      //Serial.print("\tLuminosity: ");
      //Serial.print(Luminosidad,4);
      //Serial.print(" W/m²");
          
      // Print out the Altitude.
      Serial.print("\tAltitude: ");
      Serial.print(altitude);
      Serial.print(" m");
          
      // Print out the Temperature
      Serial.print("\tTemperature: ");
      Serial.print(currentTemperature);
      Serial.write(176);
      Serial.print("C");
      
      Serial.println(); // Start a new line.
      //delay(1000); // Show new results every second.
    }
    else{
      Serial.print(currentPressure);
      Serial.println();
    }
  }
}

//Print informacion
void loop()
{
  //Indicator for the type of information that will be printed
  int type=1;
  if(type>0){
  Serial.println("-1000");
  }
  BMP_180_Measurement(type);
  //MAX44009_Measurement(type);
  TSL2561_Measurement(type);
  //Test_I2C();
  delay(1000);
} 
