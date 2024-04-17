#include "AMS5915_simplified.h"

AMS5915_simplified::AMS5915_simplified(TwoWire &bus, uint8_t address, Transducer type) // constructor, I2C bus, sensor address, and transducer type
{
  _bus = &bus;           // I2C bus
  _address = address;    // I2C address
  _type = type;          // transducer type
}

int AMS5915_simplified::begin() // starts the I2C communication and sets the pressure and temperature ranges using getTransducer
{
  _bus->begin();                              // starting the I2C bus
  _bus->setClock(_i2cRate);                   // setting the I2C clock
  getTransducer();                            // setting the min and max pressure based on the chip
  for (size_t i = 0; i < _maxAttempts; i++)   // checking to see if we can talk with the sensor
  {
    _status = readBytes(&_pressureCounts, &_temperatureCounts);
    if (_status > 0) {
      break;
    }
    delay(10);
  }
  return _status;
}

int AMS5915_simplified::readSensor(char letter)  // reads data from the sensor
{
  uint16_t digOutPmin;
  switch (letter)
  {
    case 'A' :  digOutPmin = 1638; break;
    case 'D' : digOutPmin = 1569; break; // le capteur utilisé présente une erreur de zéro, la valeur pour une pression différentielle nulle devrait être 1638, hors elle est à 1569
  }

  _status = readBytes(&_pressureCounts, &_temperatureCounts);                                                                                                 // get pressure and temperature counts off transducer
  _data.Pressure_Pa = (((float)(_pressureCounts - digOutPmin)) / (((float)(_digOutPmax - digOutPmin)) / ((float)(_pMax - _pMin))) + (float)_pMin) * _mBar2Pa; // convert counts to pressure, PA
  _data.Temp_C = (float)((_temperatureCounts * 200)) / 2048.0f - 50.0f;                                                                                       // convert counts to temperature, C
  return _status;
}

float AMS5915_simplified::getPressure_Pa() // returns the pressure value, PA
{
  return _data.Pressure_Pa;
}

float AMS5915_simplified::getTemperature_C()  // returns the temperature value, C
{
  return _data.Temp_C;
}

void AMS5915_simplified::getTransducer()  // sets the pressure range based on the chip
{
  switch (_type) // setting the min and max pressures based on which transducer it is
  {
    case AMS5915_0050_D:
      _pMin = AMS5915_0050_D_P_MIN;
      _pMax = AMS5915_0050_D_P_MAX;
      break;

    case AMS5915_1500_A:
      _pMin = AMS5915_1500_A_P_MIN;
      _pMax = AMS5915_1500_A_P_MAX;
      break;
  }
}

int AMS5915_simplified::readBytes(uint16_t* pressureCounts, uint16_t* temperatureCounts) // reads pressure and temperature and returns values in counts
{
  _numBytes = _bus->requestFrom(_address, sizeof(_buffer)); // read from sensor
  if (_numBytes == sizeof(_buffer)) // put the data in buffer
  {
    _buffer[0] = _bus->read();
    _buffer[1] = _bus->read();
    _buffer[2] = _bus->read();
    _buffer[3] = _bus->read();
    *pressureCounts = (((uint16_t) (_buffer[0] & 0x3F)) << 8) + (((uint16_t) _buffer[1])); // assemble into a uint16_t
    *temperatureCounts = (((uint16_t) (_buffer[2])) << 3) + (((uint16_t) _buffer[3] & 0xE0) >> 5);
    _status = 1;
  }
  else
  {
    _status = -1;
  }
  return _status;
}
