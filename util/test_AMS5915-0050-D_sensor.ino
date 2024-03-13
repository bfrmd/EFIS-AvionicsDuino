// Software utility to check the output from the AMS5915-0050-D sensor for any differential pressure between 0 and 50 mbar. 
// Results in counts and Pa are displayed on the IDE serial monitor. See AMS5915 datasheet for further information.

#include "Wire.h"
float diffPressure;
float previousFilteredDiffPressure;
float Temp_C;
uint8_t bufferI2C[4];               // buffer for I2C data
size_t numBytes;                    // number of bytes received from I2C
const size_t _maxAttempts = 10;     // maximum number of attempts to talk to sensor
int statusCapteur;                  // track success of reading from sensor
uint16_t _pressureCounts;           // pressure digital output, counts
uint16_t _temperatureCounts;        // temperature digital output, counts
int _pMin=0, _pMax=50;              // min and max pressure, millibar
uint16_t minOutputCount = 1638;     // at specified minimum pressure (i.e.  0 millibar ) see AMS5915-0050-D datasheet
uint16_t maxOutputCount = 14745;    // at specified maximum pressure (i.e. 50 millibars) see AMS5915-0050-D datasheet
const float _mBar2Pa = 100.0f;      // conversion millibar to PA
uint32_t  accumulator=0;
uint16_t counter = 0;

void setup() 
{
  Serial.begin(9600);
  Wire1.begin();                             // Adjust Wire (here and below) if necessary: Wire, Wire1, Wire2...
  Wire1.setClock(400000);                    // i2c bus frequency
 for (size_t i=0; i < _maxAttempts; i++)     // checking to see if we can talk with the sensor
  {
    statusCapteur = readSensorCounts(&_pressureCounts,&_temperatureCounts);
    if (statusCapteur > 0) {break;}
    delay(10);
  }

if (statusCapteur < 0) 
  {
    
    Serial.print("An error occured when communicating with the AMS5915-0050-D sensor");
    while(1){}
  }
}

void loop() 
{
  delay(100);
  readSensorUnits();
  diffPressure = IIRFilteredValue (previousFilteredDiffPressure, diffPressure , 0.005); // IIR filter is applied 
  previousFilteredDiffPressure = diffPressure;  
  Serial.print ("Differential presure : "); 
  Serial.print (diffPressure); 
  Serial.print (" Pa");
  Serial.print ("        Temperature : "); 
  Serial.print (Temp_C); 
  Serial.print (" °C");
}

int readSensorUnits()  // reads data from the sensor and returns values in units of Pascals and °C
{
  statusCapteur = readSensorCounts(&_pressureCounts,&_temperatureCounts);                                                                                      // get pressure and temperature counts off transducer
  diffPressure = (((float)(_pressureCounts - minOutputCount))/(((float)(maxOutputCount - minOutputCount))/((float)(_pMax - _pMin)))+(float)_pMin)*_mBar2Pa;    // convert counts to pressure, PA
  Temp_C = (float)((_temperatureCounts*200))/2048.0f-50.0f;                                                                                                    // convert counts to temperature, C
  return statusCapteur;
}

int readSensorCounts(uint16_t * pressureCounts,uint16_t * temperatureCounts)                          // reads pressure and temperature and returns values in counts
{ 
  numBytes = Wire1.requestFrom(0x28,sizeof(bufferI2C));                                               // read from sensor. Adjust I2C address if necessary
  if (numBytes == sizeof(bufferI2C))                                                                  // put the data in buffer
  {
    bufferI2C[0] = Wire1.read(); 
    bufferI2C[1] = Wire1.read();
    bufferI2C[2] = Wire1.read();
    bufferI2C[3] = Wire1.read();
    *pressureCounts = (((uint16_t) (bufferI2C[0]&0x3F)) <<8) + (((uint16_t) bufferI2C[1]));            // assemble into a uint16_t
    *temperatureCounts = (((uint16_t) (bufferI2C[2])) <<3) + (((uint16_t) bufferI2C[3]&0xE0)>>5);
    accumulator = accumulator + *pressureCounts;
    counter ++; 
    
    Serial.print ("        Pressure Counts : "); 
    Serial.print(*pressureCounts);  
    Serial.print ("        Mean Pressure Counts : "); 
    Serial.print(accumulator/counter);                      // when diffPressure is zero, should be as close as possible to minOutputCount (1638) after a few seconds
    Serial.print ("        Temp Counts : "); 
    Serial.println(*temperatureCounts);
    statusCapteur = 1;
  } 
  else 
  {
    statusCapteur = -1;
  }
  return statusCapteur;
}


//  Infinite Impulse Response Filter function (IIR filter)
float IIRFilteredValue (float previousFilteredValue, float currentValue , float IIRFilterCoeff)
{
  return previousFilteredValue  * (1-IIRFilterCoeff) + currentValue * IIRFilterCoeff ;
}
