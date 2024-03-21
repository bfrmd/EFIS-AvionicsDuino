// Cette classe est copiée-collée à partir de la bibliothèque AMS5915 de Brian R Taylor (https://github.com/bolderflight/AMS5915-arduino)
// en l'adaptant pour un usage exclusif des capteurs AMS5915/1500-A et AMS5915/0050-D

#ifndef AMS5915_simplified_h
#define AMS5915_simplified_h

#include "Wire.h"

class AMS5915_simplified
{
  public:
    enum Transducer
    {
      AMS5915_0050_D,
      AMS5915_1500_A
    };

    AMS5915_simplified(TwoWire &bus, uint8_t address, Transducer type);
    int begin();
    int readSensor(char letter);
    float getPressure_Pa();
    float getTemperature_C();

  private:

    struct Data // struct to hold sensor data
    {
      float Pressure_Pa;
      float Temp_C;
    };
    Data _data;

    TwoWire *_bus;                    // I2C bus
    uint8_t _address;                 // sensor address
    Transducer _type;                 // transducer type
    uint8_t _buffer[4];               // buffer for I2C data
    size_t _numBytes;                 // number of bytes received from I2C
    const size_t _maxAttempts = 10;   // maximum number of attempts to talk to sensor
    int _status;                      // track success of reading from sensor
    uint16_t _pressureCounts;         // pressure digital output, counts
    uint16_t _temperatureCounts;      // temperature digital output, counts
    int _pMin; int _pMax;             // min and max pressure, millibar
    const uint32_t _i2cRate = 400000; // i2c bus frequency
    const float _mBar2Pa = 100.0f;    // conversion millibar to PA
    //const int _digOutPmin;            // digital output at minimum pressure : en fait, ça dépend du capteur...!! Donc cette ligne n'est pas utilisée
    const int _digOutPmax = 14745;    // digital output at maximum pressure

    // min and max pressures, millibar
    const int AMS5915_0050_D_P_MIN = 0;
    const int AMS5915_0050_D_P_MAX = 50;
    const int AMS5915_1500_A_P_MIN = 0;
    const int AMS5915_1500_A_P_MAX = 1500;

    void getTransducer();
    int readBytes(uint16_t* pressureCounts, uint16_t* temperatureCounts);
};

#endif
