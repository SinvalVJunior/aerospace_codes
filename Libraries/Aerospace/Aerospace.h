

#ifndef Aerospace_h
#define Aerospace_h

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#include <pins_arduino.h>
#endif

//------------------Barometer------------
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SPI.h>

/**************************************************************************/
/*! 
    @brief Register addresses
*/
/**************************************************************************/
enum {
  BME280_REGISTER_DIG_T1              = 0x88,
  BME280_REGISTER_DIG_T2              = 0x8A,
  BME280_REGISTER_DIG_T3              = 0x8C,

  BME280_REGISTER_DIG_P1              = 0x8E,
  BME280_REGISTER_DIG_P2              = 0x90,
  BME280_REGISTER_DIG_P3              = 0x92,
  BME280_REGISTER_DIG_P4              = 0x94,
  BME280_REGISTER_DIG_P5              = 0x96,
  BME280_REGISTER_DIG_P6              = 0x98,
  BME280_REGISTER_DIG_P7              = 0x9A,
  BME280_REGISTER_DIG_P8              = 0x9C,
  BME280_REGISTER_DIG_P9              = 0x9E,

  BME280_REGISTER_DIG_H1              = 0xA1,
  BME280_REGISTER_DIG_H2              = 0xE1,
  BME280_REGISTER_DIG_H3              = 0xE3,
  BME280_REGISTER_DIG_H4              = 0xE4,
  BME280_REGISTER_DIG_H5              = 0xE5,
  BME280_REGISTER_DIG_H6              = 0xE7,

  BME280_REGISTER_CHIPID             = 0xD0,
  BME280_REGISTER_SOFTRESET          = 0xE0,

  BME280_REGISTER_CONTROLHUMID       = 0xF2,
  BME280_REGISTER_STATUS             = 0XF3,
  BME280_REGISTER_CONTROL            = 0xF4,
  BME280_REGISTER_CONFIG             = 0xF5
};

/**************************************************************************/
/*! 
    @brief  calibration data
*/
/**************************************************************************/
typedef struct
{
  uint16_t dig_T1; ///< temperature compensation value
  int16_t  dig_T2; ///< temperature compensation value
  int16_t  dig_T3; ///< temperature compensation value

  uint16_t dig_P1; ///< pressure compensation value
  int16_t  dig_P2; ///< pressure compensation value
  int16_t  dig_P3; ///< pressure compensation value
  int16_t  dig_P4; ///< pressure compensation value
  int16_t  dig_P5; ///< pressure compensation value
  int16_t  dig_P6; ///< pressure compensation value
  int16_t  dig_P7; ///< pressure compensation value
  int16_t  dig_P8; ///< pressure compensation value
  int16_t  dig_P9; ///< pressure compensation value

  uint8_t  dig_H1; ///< humidity compensation value
  int16_t  dig_H2; ///< humidity compensation value
  uint8_t  dig_H3; ///< humidity compensation value
  int16_t  dig_H4; ///< humidity compensation value
  int16_t  dig_H5; ///< humidity compensation value
  int8_t   dig_H6; ///< humidity compensation value
} bme280_calib_data;

//DHT
#define DEBUG_PRINTER Serial

// Setup debug printing macros.
#ifdef DHT_DEBUG
  #define DEBUG_PRINT(...) { DEBUG_PRINTER.print(__VA_ARGS__); }
  #define DEBUG_PRINTLN(...) { DEBUG_PRINTER.println(__VA_ARGS__); }
#else
  #define DEBUG_PRINT(...) {}
  #define DEBUG_PRINTLN(...) {}
#endif

// Define types of sensors.
#define DHT11 11


//------------------GPS----------------

#define _GPS_VERSION 13 // software version of this library
#define _GPS_MPH_PER_KNOT 1.15077945
#define _GPS_MPS_PER_KNOT 0.51444444
#define _GPS_KMPH_PER_KNOT 1.852
#define _GPS_MILES_PER_METER 0.00062137112
#define _GPS_KM_PER_METER 0.001
// #define _GPS_NO_STATS


class Aerospace
{
  public:
    //---------------------------Acelerometro------------------------
    Aerospace();
    void accelero_begin();
    int accelero_getXAccel();
    int accelero_getYAccel();
    int accelero_getZAccel();
    void accelero_setOffSets(int xOffSet, int yOffSet, int zOffSet);
    void accelero_calibrate();                             // only to be executed when Z-axis is oriented to the ground
    int accelero_getOrientation();

    // DHT : EM ANDAMENTO
    //VERIFICAR : se estamos usando apenas essas funcoes e os parâmetros delas 

	float DHT_readTemperature(bool S=false, bool force=false);
	float DHT_readHumidity(bool force=false);
	boolean DHT_read(bool force=false);
	//----------------------GPS-----------------
	enum {
	    GPS_INVALID_AGE = 0xFFFFFFFF,      GPS_INVALID_ANGLE = 999999999, 
	    GPS_INVALID_ALTITUDE = 999999999,  GPS_INVALID_DATE = 0,
	    GPS_INVALID_TIME = 0xFFFFFFFF,		 GPS_INVALID_SPEED = 999999999, 
	    GPS_INVALID_FIX_TIME = 0xFFFFFFFF, GPS_INVALID_SATELLITES = 0xFF,
	    GPS_INVALID_HDOP = 0xFFFFFFFF
	  };
	static const float GPS_INVALID_F_ANGLE, GPS_INVALID_F_ALTITUDE, GPS_INVALID_F_SPEED;
	
	bool GPS_encode(char c);
	void GPS_get_position(long *latitude, long *longitude, unsigned long *fix_age = 0);
	void GPS_get_datetime(unsigned long *date, unsigned long *time, unsigned long *age = 0);
	void GPS_crack_datetime(int *year, byte *month, byte *day,byte *hour, byte *minute, byte *second, byte *hundredths = 0, unsigned long *fix_age = 0);
  float GPS_f_altitude();
  float GPS_f_speed_kmph();
  float GPS_f_speed_knots();
  float GPS_f_speed_mps();

  //-----------------Barometer--------------------
  bool BME_begin(void);
  bool BME_init();
  float BME_readTemperature(void);
  float BME_readPressure(void);
  float BME_readHumidity(void);
        
  float BME_readAltitude(float seaLevel);
  float BME_seaLevelForAltitude(float altitude, float pressure);

  //sampling rates
  enum sensor_sampling {
    SAMPLING_NONE = 0b000,
    SAMPLING_X1   = 0b001,
    SAMPLING_X2   = 0b010,
    SAMPLING_X4   = 0b011,
    SAMPLING_X8   = 0b100,
    SAMPLING_X16  = 0b101
  };

  //power modes
  enum sensor_mode {
    MODE_SLEEP  = 0b00,
    MODE_FORCED = 0b01,
    MODE_NORMAL = 0b11
  };

  //Filter values
  enum sensor_filter {
    FILTER_OFF = 0b000,
    FILTER_X2  = 0b001,
    FILTER_X4  = 0b010,
    FILTER_X8  = 0b011,
    FILTER_X16 = 0b100
  };

  enum standby_duration {
    STANDBY_MS_0_5  = 0b000,
    STANDBY_MS_10   = 0b110,
    STANDBY_MS_20   = 0b111,
    STANDBY_MS_62_5 = 0b001,
    STANDBY_MS_125  = 0b010,
    STANDBY_MS_250  = 0b011,
    STANDBY_MS_500  = 0b100,
    STANDBY_MS_1000 = 0b101
  };

  void BME_setSampling(sensor_mode mode              = MODE_NORMAL,
		sensor_sampling tempSampling  = SAMPLING_X16,
		sensor_sampling pressSampling = SAMPLING_X16,
		sensor_sampling humSampling   = SAMPLING_X16,
		sensor_filter filter          = FILTER_OFF,
		standby_duration duration     = STANDBY_MS_0_5
		);



  private:
//------------Acelerometro-------------
    int _sleepPin;
    int _selfTestPin;
    int _zeroGPin;
    int _gSelectPin;
    int _xPin;
    int _yPin;
    int _zPin;
    int _offSets[3];
    double _refVoltage;
    int _average;
    boolean _sleep;
    boolean _sensi;

    //DHT : EM ANDAMENTO
    //VERIFICAR : se estamos utilizando apenas essas variaveis
  	uint8_t data[5];
  	uint8_t _pin, _type;
  #ifdef __AVR
    // Use direct GPIO access on an 8-bit AVR so keep track of the port and bitmask
    // for the digital pin connected to the DHT.  Other platforms will use digitalRead.
    uint8_t _bit, _port;
  #endif
  uint32_t _lastreadtime, _maxcycles;
  bool _lastresult;

  uint32_t DHT_expectPulse(bool level);
  //--------------------------GPS-------------------
   int GPS_gpsstrcmp(const char *str1, const char *str2);
   unsigned long GPS_parse_decimal();
   unsigned long GPS_parse_degrees();
   long GPS_gpsatol(const char *str);
   bool GPS_gpsisdigit(char c) { return c >= '0' && c <= '9'; }
   enum {_GPS_SENTENCE_GPGGA, _GPS_SENTENCE_GPRMC, _GPS_SENTENCE_GNGNS, _GPS_SENTENCE_GNGSA,
      _GPS_SENTENCE_GPGSV, _GPS_SENTENCE_GLGSV,  _GPS_SENTENCE_OTHER};
   // properties
  unsigned long _time, _new_time;
  unsigned long _date, _new_date;
  long _latitude, _new_latitude;
  long _longitude, _new_longitude;
  long _altitude, _new_altitude;
  unsigned long  _speed, _new_speed;
  unsigned long  _course, _new_course;
  unsigned long  _hdop, _new_hdop;
  unsigned short _numsats, _new_numsats;

  unsigned long _last_time_fix, _new_time_fix;
  unsigned long _last_position_fix, _new_position_fix;

  // parsing state variables
  byte _parity;
  bool _is_checksum_term;
  char _term[15];
  byte _sentence_type;
  byte _term_number;
  byte _term_offset;
  bool _gps_data_good;
  struct TrackedSattelites {
      uint8_t prn;      //"pseudo-random noise" sequences, or Gold codes. GPS sats are listed here http://en.wikipedia.org/wiki/List_of_GPS_satellites
      uint8_t strength; //in dB
  };

  char _constellations[5];
  uint8_t _sat_used_count;

  //format:
  //bit 0-7: sat ID
  //bit 8-14: snr (dB), max 99dB
  //bit 15: used in solution (when tracking)
  uint32_t tracked_sat_rec[24]; //TODO: externalize array size
  int _tracked_satellites_index;
  uint8_t _sat_index;

#ifndef _GPS_NO_STATS
  // statistics
  unsigned long _encoded_characters;
  unsigned short _good_sentences;
  unsigned short _failed_checksum;
  unsigned short _passed_checksum;
#endif
  int GPS_from_hex(char a);
  bool GPS_term_complete();

  //------------------Barometer-----------------
  void BME_readCoefficients(void);
  bool BME_isReadingCalibration(void);
  uint8_t BME_spixfer(uint8_t x);

  void      BME_write8(byte reg, byte value);
  uint8_t   BME_read8(byte reg);
  uint16_t  BME_read16_LE(byte reg); // little endian

  uint8_t   _i2caddr;
  TwoWire *_wire;
  int8_t _cs, _sck, _mosi, _miso;

  bme280_calib_data _bme280_calib;

  // The ctrl_meas register
  struct ctrl_meas {
    // temperature oversampling
    // 000 = skipped
    // 001 = x1
    // 010 = x2
    // 011 = x4
    // 100 = x8
    // 101 and above = x16
    unsigned int osrs_t : 3;

    // pressure oversampling
    // 000 = skipped
    // 001 = x1
    // 010 = x2
    // 011 = x4
    // 100 = x8
    // 101 and above = x16
    unsigned int osrs_p : 3;

    // device mode
    // 00       = sleep
    // 01 or 10 = forced
    // 11       = normal
    unsigned int mode : 2;

    unsigned int get() {
      return (osrs_t << 5) | (osrs_p << 2) | mode;
    }
  };
  ctrl_meas _measReg;

  // The ctrl_hum register
  struct ctrl_hum {
    // unused - don't set
    unsigned int none : 5;

    // pressure oversampling
    // 000 = skipped
    // 001 = x1
    // 010 = x2
    // 011 = x4
    // 100 = x8
    // 101 and above = x16
    unsigned int osrs_h : 3;

    unsigned int get() {
      return (osrs_h);
    }
  };
  ctrl_hum _humReg;

  // The config register
  struct config {
    // inactive duration (standby time) in normal mode
    // 000 = 0.5 ms
    // 001 = 62.5 ms
    // 010 = 125 ms
    // 011 = 250 ms
    // 100 = 500 ms
    // 101 = 1000 ms
    // 110 = 10 ms
    // 111 = 20 ms
    unsigned int t_sb : 3;

    // filter settings
    // 000 = filter off
    // 001 = 2x filter
    // 010 = 4x filter
    // 011 = 8x filter
    // 100 and above = 16x filter
    unsigned int filter : 3;

    // unused - don't set
    unsigned int none : 1;
    unsigned int spi3w_en : 1;

    unsigned int get() {
      return (t_sb << 5) | (filter << 2) | spi3w_en;
    }
  };
  config _configReg;
};


class DHT_InterruptLock {
  public:
   DHT_InterruptLock() {
    noInterrupts();
   }
   ~DHT_InterruptLock() {
    interrupts();
   }

};

#endif
