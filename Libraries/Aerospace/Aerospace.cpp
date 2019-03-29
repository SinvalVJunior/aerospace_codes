#include "Aerospace.h"
//------------------------GPS------------------
#define COMBINE(sentence_type, term_number) (((unsigned)(sentence_type) << 5) | term_number)
#define _GPRMC_TERM   "GPRMC"
#define _GPGGA_TERM   "GPGGA"
#define _GPGSA_TERM   "GPGSA"
#define _GNRMC_TERM   "GNRMC"
#define _GNGNS_TERM   "GNGNS"
#define _GNGSA_TERM   "GNGSA"
#define _GPGSV_TERM   "GPGSV"
#define _GLGSV_TERM   "GLGSV"

Aerospace::Aerospace()
//--------------GPS---------------------------------- 
  :  _time(GPS_INVALID_TIME)
  ,  _date(GPS_INVALID_DATE)
  ,  _latitude(GPS_INVALID_ANGLE)
  ,  _longitude(GPS_INVALID_ANGLE)
  ,  _altitude(GPS_INVALID_ALTITUDE)
  ,  _speed(GPS_INVALID_SPEED)
  ,  _course(GPS_INVALID_ANGLE)
  ,  _hdop(GPS_INVALID_HDOP)
  ,  _numsats(GPS_INVALID_SATELLITES)
  ,  _last_time_fix(GPS_INVALID_FIX_TIME)
  ,  _last_position_fix(GPS_INVALID_FIX_TIME)
  ,  _parity(0)
  ,  _is_checksum_term(false)
  ,  _sentence_type(_GPS_SENTENCE_OTHER)
  ,  _term_number(0)
  ,  _term_offset(0)
  ,  _gps_data_good(false)
#ifndef _GPS_NO_STATS
  ,  _encoded_characters(0)
  ,  _good_sentences(0)
  ,  _failed_checksum(0)
#endif
{
  //Acelerometro
    _sleepPin=5;
    _selfTestPin=7;
    _zeroGPin=6;
    _gSelectPin=8;
    _xPin=A3;
    _yPin=A6;
    _zPin=A5;
    _offSets[0]=0;
    _offSets[1]=0;
    _offSets[2]=0;
    _refVoltage=0;
    _average=0;
    _sleep=false;
    _sensi=false;
    //DHT
    _pin = 9;
    _type = DHT11;
  #ifdef __AVR
    _bit = digitalPinToBitMask(9);//9 É O PINO UTILIZADO
    _port = digitalPinToPort(9);//9 É O PINO UTILIZADO
  #endif
  _maxcycles = microsecondsToClockCycles(1000);
  //--------------------GPS---------------

  _term[0] = '\0';
}

//ACCELERO

void Aerospace::accelero_begin()
{
  pinMode(_sleepPin, OUTPUT);
  pinMode(_selfTestPin, OUTPUT);
  pinMode(_zeroGPin, INPUT);
  pinMode(_gSelectPin, OUTPUT);
  pinMode(_xPin, INPUT);
  pinMode(_yPin, INPUT);
  pinMode(_zPin, INPUT);
  digitalWrite(_sleepPin,HIGH);
  digitalWrite(_selfTestPin,LOW);
  _sleep = false;
  accelero_setOffSets(0,0,0);
  _average=10;
  _sensi = LOW;
  _refVoltage = 5;
  analogReference(EXTERNAL);
  digitalWrite(_gSelectPin,!_sensi);
}


void Aerospace::accelero_setOffSets(int xOffSet, int yOffSet, int zOffSet)
{  
    _offSets[0]= map(xOffSet,0,3300,0,1024);
    _offSets[1]= map(yOffSet,0,3300,0,1024);
    _offSets[2]= map(zOffSet,0,3300,0,1024);

}

int Aerospace::accelero_getXAccel()
{
  int sum = 0;
  for (int i = 0;i<_average;i++)
  {
    sum = sum + map(analogRead(_xPin)+_offSets[0]+2,0,1024,-825,800);
  }
  return sum/_average;
}

int Aerospace::accelero_getYAccel()
{
  int sum = 0;
  for (int i = 0;i<_average;i++)
  {
    sum = sum + map(analogRead(_yPin)+_offSets[1]+2,0,1024,-825,800);
  }
  return sum/_average;
}

int Aerospace::accelero_getZAccel()
{
  int sum = 0;
  for (int i = 0;i<_average;i++)
  {
    sum = sum + map(analogRead(_zPin)+_offSets[2],0,1024,-825,800);
  }
  return sum/_average;
}

int Aerospace::accelero_getOrientation()
{
  int gemiddelde = 10;
  int x = 0;
  int y = 0;
  int z = 0;
  int xAbs = 0;
  int yAbs = 0;
  int zAbs = 0;
  for(int i = 0; i<gemiddelde ; i++)              //We take in this case 10 measurements to average the error a little bit
  {
    x = x+accelero_getXAccel();
    y = y+accelero_getYAccel();
    z = z+accelero_getZAccel();
  }
  x= x/gemiddelde;
  y = y/gemiddelde;
  z = z/gemiddelde;
  xAbs = abs(100-abs(x));
  yAbs = abs(100-abs(y));
  zAbs = abs(100-abs(z));
  if (xAbs<yAbs&&xAbs<zAbs)
  {
    if (x>0)
    {
      return 1;
    }
    return -1;
  }
  if (yAbs<xAbs&&yAbs<zAbs)
  {
    if (y>0)
    {
      return 2;
    }
    return -2;
  }
  if (zAbs<xAbs&&zAbs<yAbs)
  {
    if (z>0)
    {
      return 3;
    }
    return -3;
  }
  return 0;
}

void Aerospace::accelero_calibrate()
{
  Serial.println(accelero_getOrientation());
  Serial.print("\nCalibrating MMA7361011");
  double var = 5000;
  double sumX = 0;
  double sumY = 0;
  double sumZ = 0;
  for (int i = 0;i<var;i++)
  {
    sumX = sumX + map(analogRead(_xPin)+_offSets[0]+2,0,1024,0,3300);
    sumY = sumY + map(analogRead(_yPin)+_offSets[1]+2,0,1024,0,3300);
    sumZ = sumZ + map(analogRead(_zPin)+_offSets[2],0,1024,0,3300);
    if (i%100 == 0)
    {
      Serial.print(".");
    }
  }
  if (_sensi == false)
  {
    accelero_setOffSets(1672 - sumX / var,1671 - sumY / var, + 1876 - sumZ / var);
  }
  else
  {
    accelero_setOffSets(1650 - sumX / var,1650 - sumY / var, + 2450 - sumZ / var);
  }
  if (abs(accelero_getOrientation())!=3)
  {
    Serial.print("\nunable to calibrate");
    accelero_setOffSets(0,0,0);
  }
  else
  {
    Serial.print("\nDONE");
  }
}





  //DHT : AINDA EM ANDAMENTO
  //VERIFICAR: funcao read(),cases dht e define .h,define debug_println .h,construtor do dht e .cpp da library 





//Aerospace::dht_readTemperature
float Aerospace::DHT_readTemperature(bool S, bool force) {
  float f = NAN;

  if (DHT_read(force)) {
      f = data[2];
  }
  return f;
}


//Aerospace::dht_readHumidity
float Aerospace::DHT_readHumidity(bool force) {
  float f = NAN;
  if (DHT_read()) {
      f = data[0];
    }
  
  return f;
}


//Aerospace::dht_read
boolean Aerospace::DHT_read(bool force) { 
  // Check if sensor was read less than two seconds ago and return early
  // to use last reading.
  uint32_t currenttime = millis();
  if (!force && ((currenttime - _lastreadtime) < 2000)) {
    return _lastresult; // return last correct measurement
  }
  _lastreadtime = currenttime;

  // Reset 40 bits of received data to zero.
  data[0] = data[1] = data[2] = data[3] = data[4] = 0;

  // Send start signal.  See DHT datasheet for full signal diagram:
  //   http://www.adafruit.com/datasheets/Digital%20humidity%20and%20temperature%20sensor%20AM2302.pdf

  // Go into high impedence state to let pull-up raise data line level and
  // start the reading process.
  digitalWrite(_pin, HIGH);
  delay(250);

  // First set data line low for 20 milliseconds.
  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, LOW);
  delay(20);

  uint32_t cycles[80];
  {
    // Turn off interrupts temporarily because the next sections are timing critical
    // and we don't want any interruptions.
    DHT_InterruptLock lock;

    // End the start signal by setting data line high for 40 microseconds.
    digitalWrite(_pin, HIGH);
    delayMicroseconds(40);

    // Now start reading the data line to get the value from the DHT sensor.
    pinMode(_pin, INPUT_PULLUP);
    delayMicroseconds(10);  // Delay a bit to let sensor pull data line low.

    // First expect a low signal for ~80 microseconds followed by a high signal
    // for ~80 microseconds again.
    if (DHT_expectPulse(LOW) == 0) {
      DEBUG_PRINTLN(F("Timeout waiting for start signal low pulse."));
      _lastresult = false;
      return _lastresult;
    }
    if (DHT_expectPulse(HIGH) == 0) {
      DEBUG_PRINTLN(F("Timeout waiting for start signal high pulse."));
      _lastresult = false;
      return _lastresult;
    }

    // Now read the 40 bits sent by the sensor.  Each bit is sent as a 50
    // microsecond low pulse followed by a variable length high pulse.  If the
    // high pulse is ~28 microseconds then it's a 0 and if it's ~70 microseconds
    // then it's a 1.  We measure the cycle count of the initial 50us low pulse
    // and use that to compare to the cycle count of the high pulse to determine
    // if the bit is a 0 (high state cycle count < low state cycle count), or a
    // 1 (high state cycle count > low state cycle count). Note that for speed all
    // the pulses are read into a array and then examined in a later step.
    for (int i=0; i<80; i+=2) {
      cycles[i]   = DHT_expectPulse(LOW);
      cycles[i+1] = DHT_expectPulse(HIGH);
    }
  } // Timing critical code is now complete.

  // Inspect pulses and determine which ones are 0 (high state cycle count < low
  // state cycle count), or 1 (high state cycle count > low state cycle count).
  for (int i=0; i<40; ++i) {
    uint32_t lowCycles  = cycles[2*i];
    uint32_t highCycles = cycles[2*i+1];
    if ((lowCycles == 0) || (highCycles == 0)) {
      DEBUG_PRINTLN(F("Timeout waiting for pulse."));
      _lastresult = false;
      return _lastresult;
    }
    data[i/8] <<= 1;//x<<=y is x=x*2^y
    // Now compare the low and high cycle times to see if the bit is a 0 or 1.
    if (highCycles > lowCycles) {
      // High cycles are greater than 50us low cycle count, must be a 1.
      data[i/8] |= 1;
    }
    // Else high cycles are less than (or equal to, a weird case) the 50us low
    // cycle count so this must be a zero.  Nothing needs to be changed in the
    // stored data.
  }

  DEBUG_PRINTLN(F("Received:"));
  DEBUG_PRINT(data[0], HEX); DEBUG_PRINT(F(", "));
  DEBUG_PRINT(data[1], HEX); DEBUG_PRINT(F(", "));
  DEBUG_PRINT(data[2], HEX); DEBUG_PRINT(F(", "));
  DEBUG_PRINT(data[3], HEX); DEBUG_PRINT(F(", "));
  DEBUG_PRINT(data[4], HEX); DEBUG_PRINT(F(" =? "));
  DEBUG_PRINTLN((data[0] + data[1] + data[2] + data[3]) & 0xFF, HEX);

  // Check we read 40 bits and that the checksum matches.
  if (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
    _lastresult = true;
    return _lastresult;
  }
  else {
    DEBUG_PRINTLN(F("Checksum failure!"));
    _lastresult = false;
    return _lastresult;
  }
}


//Aerospace::dht_expectPulse
uint32_t Aerospace::DHT_expectPulse(bool level) {
  uint32_t count = 0;
  // On AVR platforms use direct GPIO port access as it's much faster and better
  // for catching pulses that are 10's of microseconds in length:
  #ifdef __AVR
    uint8_t portState = level ? _bit : 0;
    while ((*portInputRegister(_port) & _bit) == portState) {
      if (count++ >= _maxcycles) {
        return 0; // Exceeded timeout, fail.
      }
    }
  // Otherwise fall back to using digitalRead (this seems to be necessary on ESP8266
  // right now, perhaps bugs in direct port access functions?).
  #else
    while (digitalRead(_pin) == level) {
      if (count++ >= _maxcycles) {
        return 0; // Exceeded timeout, fail.
      }
    }
  #endif

  return count;
}

//--------------------------GPS------------------------------------
bool Aerospace::GPS_encode(char c)
{
  bool valid_sentence = false;

#ifndef _GPS_NO_STATS
  ++_encoded_characters;
#endif
  switch(c)
  {
  case ',': // term terminators
    _parity ^= c;
  case '\r':
  case '\n':
  case '*':
    if (_term_offset < sizeof(_term))
    {
      _term[_term_offset] = 0;
      valid_sentence = GPS_term_complete();
    }
    ++_term_number;
    _term_offset = 0;
    _is_checksum_term = c == '*';
    return valid_sentence;

  case '$': // sentence begin
    _term_number = _term_offset = 0;
    _parity = 0;
    _sentence_type = _GPS_SENTENCE_OTHER;
    _is_checksum_term = false;
    _gps_data_good = false;
    return valid_sentence;
  }

  // ordinary characters
  if (_term_offset < sizeof(_term) - 1)
    _term[_term_offset++] = c;
  if (!_is_checksum_term)
    _parity ^= c;

  return valid_sentence;
}
bool Aerospace::GPS_term_complete()
{
  if (_is_checksum_term)
  {
    byte checksum = 16 * GPS_from_hex(_term[0]) + GPS_from_hex(_term[1]);
    if (checksum == _parity)
    {
      if(_sentence_type == _GPS_SENTENCE_GPRMC)   //set the time and date even if not tracking
      {
          _time      = _new_time;
          _date      = _new_date;
      }
      if (_gps_data_good)
      {
#ifndef _GPS_NO_STATS
        ++_good_sentences;
#endif
        _last_time_fix = _new_time_fix;
        _last_position_fix = _new_position_fix;

        switch(_sentence_type)
        {
        case _GPS_SENTENCE_GPRMC:
          _time      = _new_time;
          _date      = _new_date;
          _latitude  = _new_latitude;
          _longitude = _new_longitude;
          _speed     = _new_speed;
          _course    = _new_course;
          break;
        case _GPS_SENTENCE_GPGGA:
          _altitude  = _new_altitude;
          _time      = _new_time;
          _latitude  = _new_latitude;
          _longitude = _new_longitude;
          _numsats   = _new_numsats;
          _hdop      = _new_hdop;
          break;
        }

        return true;
      }
    }

#ifndef _GPS_NO_STATS
    else
      ++_failed_checksum;
#endif
    return false;
  }

  // the first term determines the sentence type
  if (_term_number == 0)
  {
    if (!GPS_gpsstrcmp(_term, _GPRMC_TERM) || !GPS_gpsstrcmp(_term, _GNRMC_TERM))
      _sentence_type = _GPS_SENTENCE_GPRMC;
    else if (!GPS_gpsstrcmp(_term, _GPGGA_TERM))
      _sentence_type = _GPS_SENTENCE_GPGGA;
    else if (!GPS_gpsstrcmp(_term, _GNGNS_TERM))
      _sentence_type = _GPS_SENTENCE_GNGNS;
    else if (!GPS_gpsstrcmp(_term, _GNGSA_TERM) || !GPS_gpsstrcmp(_term, _GPGSA_TERM))
      _sentence_type = _GPS_SENTENCE_GNGSA;
    else if (!GPS_gpsstrcmp(_term, _GPGSV_TERM))
      _sentence_type = _GPS_SENTENCE_GPGSV;
    else if (!GPS_gpsstrcmp(_term, _GLGSV_TERM))
      _sentence_type = _GPS_SENTENCE_GLGSV;
    else
      _sentence_type = _GPS_SENTENCE_OTHER;
    return false;
  }

  if (_sentence_type != _GPS_SENTENCE_OTHER && _term[0])
    switch(COMBINE(_sentence_type, _term_number))
  {
    case COMBINE(_GPS_SENTENCE_GPRMC, 1): // Time in both sentences
    case COMBINE(_GPS_SENTENCE_GPGGA, 1):
    case COMBINE(_GPS_SENTENCE_GNGNS, 1):
      _new_time = GPS_parse_decimal();
      _new_time_fix = millis();
      break;
    case COMBINE(_GPS_SENTENCE_GPRMC, 2): // GPRMC validity
      _gps_data_good = _term[0] == 'A';
      break;
    case COMBINE(_GPS_SENTENCE_GPRMC, 3): // Latitude
    case COMBINE(_GPS_SENTENCE_GPGGA, 2):
    case COMBINE(_GPS_SENTENCE_GNGNS, 2):
      _new_latitude = GPS_parse_degrees();
      _new_position_fix = millis();
      break;
    case COMBINE(_GPS_SENTENCE_GPRMC, 4): // N/S
    case COMBINE(_GPS_SENTENCE_GPGGA, 3):
    case COMBINE(_GPS_SENTENCE_GNGNS, 3):
      if (_term[0] == 'S')
        _new_latitude = -_new_latitude;
      break;
    case COMBINE(_GPS_SENTENCE_GPRMC, 5): // Longitude
    case COMBINE(_GPS_SENTENCE_GPGGA, 4):
    case COMBINE(_GPS_SENTENCE_GNGNS, 4):
      _new_longitude = GPS_parse_degrees();
      break;
    case COMBINE(_GPS_SENTENCE_GPRMC, 6): // E/W
    case COMBINE(_GPS_SENTENCE_GPGGA, 5):
    case COMBINE(_GPS_SENTENCE_GNGNS, 5):
      if (_term[0] == 'W')
        _new_longitude = -_new_longitude;
      break;
    case COMBINE(_GPS_SENTENCE_GNGNS, 6):
      strncpy(_constellations, _term, 5);
      break;
    case COMBINE(_GPS_SENTENCE_GPRMC, 7): // Speed (GPRMC)
      _new_speed = GPS_parse_decimal();
      break;
    case COMBINE(_GPS_SENTENCE_GPRMC, 8): // Course (GPRMC)
      _new_course = GPS_parse_decimal();
      break;
    case COMBINE(_GPS_SENTENCE_GPRMC, 9): // Date (GPRMC)
      _new_date = GPS_gpsatol(_term);
      break;
    case COMBINE(_GPS_SENTENCE_GPGGA, 6): // Fix data (GPGGA)
      _gps_data_good = _term[0] > '0';
      break;
    case COMBINE(_GPS_SENTENCE_GPGGA, 7): // Satellites used (GPGGA): GPS only
    case COMBINE(_GPS_SENTENCE_GNGNS, 7): //  GNGNS counts-in all constellations
      _new_numsats = (unsigned char)atoi(_term);
      break;
    case COMBINE(_GPS_SENTENCE_GPGGA, 8): // HDOP
      _new_hdop = GPS_parse_decimal();
      break;
    case COMBINE(_GPS_SENTENCE_GPGGA, 9): // Altitude (GPGGA)
      _new_altitude = GPS_parse_decimal();
      break;
    case COMBINE(_GPS_SENTENCE_GNGSA, 3): //satellites used in solution: 3 to 15
      //_sats_used[
      break;
    case COMBINE(_GPS_SENTENCE_GPGSV, 2):   //beginning of sequence
    case COMBINE(_GPS_SENTENCE_GLGSV, 2):   //beginning of sequence
    {
      uint8_t msgId = atoi(_term)-1;  //start from 0
      if(msgId == 0) {
        //http://geostar-navigation.com/file/geos3/geos_nmea_protocol_v3_0_eng.pdf
        if(_sentence_type == _GPS_SENTENCE_GPGSV) {
          //reset GPS & WAAS trackedSatellites
          for(uint8_t x=0;x<12;x++)
          {
            tracked_sat_rec[x] = 0;
          }
        } else {
          //reset GLONASS trackedSatellites: range starts with 23
          for(uint8_t x=12;x<24;x++)
          {
            tracked_sat_rec[x] = 0;
          }
        }
      }
      _sat_index = msgId*4;   //4 sattelites/line
      if(_sentence_type == _GPS_SENTENCE_GLGSV)
      {
        _sat_index = msgId*4 + 12;   //Glonass offset by 12
      }
      break;
  }
    case COMBINE(_GPS_SENTENCE_GPGSV, 4):   //satellite #
    case COMBINE(_GPS_SENTENCE_GPGSV, 8):
    case COMBINE(_GPS_SENTENCE_GPGSV, 12):
    case COMBINE(_GPS_SENTENCE_GPGSV, 16):
    case COMBINE(_GPS_SENTENCE_GLGSV, 4):
    case COMBINE(_GPS_SENTENCE_GLGSV, 8):
    case COMBINE(_GPS_SENTENCE_GLGSV, 12):
    case COMBINE(_GPS_SENTENCE_GLGSV, 16):
      _tracked_satellites_index = atoi(_term);
      break;
    case COMBINE(_GPS_SENTENCE_GPGSV, 7):   //strength
    case COMBINE(_GPS_SENTENCE_GPGSV, 11):
    case COMBINE(_GPS_SENTENCE_GPGSV, 15):
    case COMBINE(_GPS_SENTENCE_GPGSV, 19):
    case COMBINE(_GPS_SENTENCE_GLGSV, 7):   //strength
    case COMBINE(_GPS_SENTENCE_GLGSV, 11):
    case COMBINE(_GPS_SENTENCE_GLGSV, 15):
    case COMBINE(_GPS_SENTENCE_GLGSV, 19):
      uint8_t stren = (uint8_t)atoi(_term);
      if(stren == 0)  //remove the record, 0dB strength
      {
        tracked_sat_rec[_sat_index + (_term_number-7)/4] = 0;
      }
      else
      {
        tracked_sat_rec[_sat_index + (_term_number-7)/4] = _tracked_satellites_index<<8 | stren<<1;
      }
      break;
  }

  return false;
}
void Aerospace::GPS_get_position(long *latitude, long *longitude, unsigned long *fix_age)
{
  if (latitude) *latitude = _latitude;
  if (longitude) *longitude = _longitude;
  if (fix_age) *fix_age = _last_position_fix == GPS_INVALID_FIX_TIME ? 
   GPS_INVALID_AGE : millis() - _last_position_fix;
}
void Aerospace::GPS_get_datetime(unsigned long *date, unsigned long *time, unsigned long *age)
{
  if (date) *date = _date;
  if (time) *time = _time;
  if (age) *age = _last_time_fix == GPS_INVALID_FIX_TIME ? 
   GPS_INVALID_AGE : millis() - _last_time_fix;
}
float Aerospace::GPS_f_altitude()    
{
  return _altitude == GPS_INVALID_ALTITUDE ? GPS_INVALID_F_ALTITUDE : _altitude / 100.0;
}
float Aerospace::GPS_f_speed_kmph()  
{ 
  float sk = GPS_f_speed_knots();
  return sk == GPS_INVALID_F_SPEED ? GPS_INVALID_F_SPEED : _GPS_KMPH_PER_KNOT * sk; 
}
float Aerospace::GPS_f_speed_mps()   
{ 
  float sk = GPS_f_speed_knots();
  return sk == GPS_INVALID_F_SPEED ? GPS_INVALID_F_SPEED : _GPS_MPS_PER_KNOT * sk; 
}
void Aerospace::GPS_crack_datetime(int *year, byte *month, byte *day, 
  byte *hour, byte *minute, byte *second, byte *hundredths, unsigned long *age)
{
  unsigned long date, time;
  GPS_get_datetime(&date, &time, age);
  if (year) 
  {
    *year = date % 100;
    *year += *year > 80 ? 1900 : 2000;
  }
  if (month) *month = (date / 100) % 100;
  if (day) *day = date / 10000;
  if (hour) *hour = time / 1000000;
  if (minute) *minute = (time / 10000) % 100;
  if (second) *second = (time / 100) % 100;
  if (hundredths) *hundredths = time % 100;
}
int Aerospace::GPS_from_hex(char a) 
{
  if (a >= 'A' && a <= 'F')
    return a - 'A' + 10;
  else if (a >= 'a' && a <= 'f')
    return a - 'a' + 10;
  else
    return a - '0';
}
int Aerospace::GPS_gpsstrcmp(const char *str1, const char *str2)
{
  while (*str1 && *str1 == *str2)
    ++str1, ++str2;
  return *str1;
}
unsigned long Aerospace::GPS_parse_decimal()
{
  char *p = _term;
  bool isneg = *p == '-';
  if (isneg) ++p;
  unsigned long ret = 100UL * GPS_gpsatol(p);
  while (GPS_gpsisdigit(*p)) ++p;
  if (*p == '.')
  {
    if (GPS_gpsisdigit(p[1]))
    {
      ret += 10 * (p[1] - '0');
      if (GPS_gpsisdigit(p[2]))
        ret += p[2] - '0';
    }
  }
  return isneg ? -ret : ret;
}
unsigned long Aerospace::GPS_parse_degrees()
{
  char *p;
  unsigned long left_of_decimal = GPS_gpsatol(_term);
  unsigned long hundred1000ths_of_minute = (left_of_decimal % 100UL) * 100000UL;
  for (p=_term; GPS_gpsisdigit(*p); ++p);
  if (*p == '.')
  {
    unsigned long mult = 10000;
    while (GPS_gpsisdigit(*++p))
    {
      hundred1000ths_of_minute += mult * (*p - '0');
      mult /= 10;
    }
  }
  return (left_of_decimal / 100) * 1000000 + (hundred1000ths_of_minute + 3) / 6;
}
long Aerospace::GPS_gpsatol(const char *str)
{
  long ret = 0;
  while (GPS_gpsisdigit(*str))
    ret = 10 * ret + *str++ - '0';
  return ret;
}
float Aerospace::GPS_f_speed_knots() 
{
  return _speed == GPS_INVALID_SPEED ? GPS_INVALID_F_SPEED : _speed / 100.0;
}
const float Aerospace::GPS_INVALID_F_ANGLE = 1000.0;
const float Aerospace::GPS_INVALID_F_ALTITUDE = 1000000.0;
const float Aerospace::GPS_INVALID_F_SPEED = -1.0;

//---------------------Barometer---------------------

/**************************************************************************/
/*!
    @brief  Initialise sensor with given parameters / settings
    @returns true on success, false otherwise
*/
/**************************************************************************/
bool Aerospace::BME_begin(void)
{
  _i2caddr = BME280_ADDRESS;
	_wire = &Wire;
	return BME_init();
}

/**************************************************************************/
/*!
    @brief  Initialise sensor with given parameters / settings
    @returns true on success, false otherwise
*/
/**************************************************************************/
bool Aerospace::BME_init()
{
    // init I2C or SPI sensor interface
    if (_cs == -1) {
        // I2C
        _wire -> BME_begin();
    } else {
        digitalWrite(_cs, HIGH);
        pinMode(_cs, OUTPUT);
        if (_sck == -1) {
            // hardware SPI
            SPI.begin();
        } else {
            // software SPI
            pinMode(_sck, OUTPUT);
            pinMode(_mosi, OUTPUT);
            pinMode(_miso, INPUT);
        }
    }

    // check if sensor, i.e. the chip ID is correct
    if (BME_read8(BME280_REGISTER_CHIPID) != 0x60)
        return false;

    // reset the device using soft-reset
    // this makes sure the IIR is off, etc.
    BME_write8(BME280_REGISTER_SOFTRESET, 0xB6);

    // wait for chip to wake up.
    delay(300);

    // if chip is still reading calibration, delay
    while (BME_isReadingCalibration())
          delay(100);

    BME_readCoefficients(); // read trimming parameters, see DS 4.2.2

    //setSampling(); // use defaults

    delay(100);

    return true;
}

/**************************************************************************/
/*!
    @brief  Reads an 8 bit value over I2C or SPI
    @param reg the register address to read from
    @returns the data byte read from the device
*/
/**************************************************************************/
uint8_t Aerospace::BME_read8(byte reg) {
    uint8_t value;
    
    if (_cs == -1) {
        _wire -> beginTransmission((uint8_t)_i2caddr);
        _wire -> write((uint8_t)reg);
        _wire -> endTransmission();
        _wire -> requestFrom((uint8_t)_i2caddr, (byte)1);
        value = _wire -> read();
    } else {
        if (_sck == -1)
            SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
        digitalWrite(_cs, LOW);
        BME_spixfer(reg | 0x80); // read, bit 7 high
        value = BME_spixfer(0);
        digitalWrite(_cs, HIGH);
        if (_sck == -1)
            SPI.endTransaction(); // release the SPI bus
    }
    return value;
}

/**************************************************************************/
/*!
    @brief  Writes an 8 bit value over I2C or SPI
    @param reg the register address to write to
    @param value the value to write to the register
*/
/**************************************************************************/
void Aerospace::BME_write8(byte reg, byte value) {
    if (_cs == -1) {
        _wire -> beginTransmission((uint8_t)_i2caddr);
        _wire -> write((uint8_t)reg);
        _wire -> write((uint8_t)value);
        _wire -> endTransmission();
    } else {
        if (_sck == -1)
            SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
        digitalWrite(_cs, LOW);
        BME_spixfer(reg & ~0x80); // write, bit 7 low
        BME_spixfer(value);
        digitalWrite(_cs, HIGH);
    if (_sck == -1)
        SPI.endTransaction(); // release the SPI bus
    }
}

/**************************************************************************/
/*!
    @brief return true if chip is busy reading cal data
    @returns true if reading calibration, false otherwise
*/
/**************************************************************************/
bool Aerospace::BME_isReadingCalibration(void)
{
  uint8_t const rStatus = BME_read8(BME280_REGISTER_STATUS);

  return (rStatus & (1 << 0)) != 0;
}

/**************************************************************************/
/*!
    @brief  Reads the factory-set coefficients
*/
/**************************************************************************/
void Aerospace::BME_readCoefficients(void)
{
    _bme280_calib.dig_T1 = BME_read16_LE(BME280_REGISTER_DIG_T1);
    _bme280_calib.dig_T2 = BME_readS16_LE(BME280_REGISTER_DIG_T2);
    _bme280_calib.dig_T3 = BME_readS16_LE(BME280_REGISTER_DIG_T3);

    _bme280_calib.dig_P1 = BME_read16_LE(BME280_REGISTER_DIG_P1);
    _bme280_calib.dig_P2 = BME_readS16_LE(BME280_REGISTER_DIG_P2);
    _bme280_calib.dig_P3 = BME_readS16_LE(BME280_REGISTER_DIG_P3);
    _bme280_calib.dig_P4 = BME_readS16_LE(BME280_REGISTER_DIG_P4);
    _bme280_calib.dig_P5 = BME_readS16_LE(BME280_REGISTER_DIG_P5);
    _bme280_calib.dig_P6 = BME_readS16_LE(BME280_REGISTER_DIG_P6);
    _bme280_calib.dig_P7 = BME_readS16_LE(BME280_REGISTER_DIG_P7);
    _bme280_calib.dig_P8 = BME_readS16_LE(BME280_REGISTER_DIG_P8);
    _bme280_calib.dig_P9 = BME_readS16_LE(BME280_REGISTER_DIG_P9);

    _bme280_calib.dig_H1 = BME_read8(BME280_REGISTER_DIG_H1);
    _bme280_calib.dig_H2 = BME_readS16_LE(BME280_REGISTER_DIG_H2);
    _bme280_calib.dig_H3 = BME_read8(BME280_REGISTER_DIG_H3);
    _bme280_calib.dig_H4 = (BME_read8(BME280_REGISTER_DIG_H4) << 4) | (BME_read8(BME280_REGISTER_DIG_H4+1) & 0xF);
    _bme280_calib.dig_H5 = (BME_read8(BME280_REGISTER_DIG_H5+1) << 4) | (BME_read8(BME280_REGISTER_DIG_H5) >> 4);
    _bme280_calib.dig_H6 = (int8_t)BME_read8(BME280_REGISTER_DIG_H6);
}

/**************************************************************************/
/*!
    @brief  Encapsulate hardware and software SPI transfer into one function
    @param x the data byte to transfer
    @returns the data byte read from the device
*/
/**************************************************************************/
uint8_t Aerospace::BME_spixfer(uint8_t x) {
    // hardware SPI
    if (_sck == -1)
        return SPI.transfer(x);

    // software SPI
    uint8_t reply = 0;
    for (int i=7; i>=0; i--) {
        reply <<= 1;
        digitalWrite(_sck, LOW);
        digitalWrite(_mosi, x & (1<<i));
        digitalWrite(_sck, HIGH);
        if (digitalRead(_miso))
            reply |= 1;
        }
    return reply;
}

/**************************************************************************/
/*!
    @brief  setup sensor with given parameters / settings
    
    This is simply a overload to the normal begin()-function, so SPI users
    don't get confused about the library requiring an address.
    @param mode the power mode to use for the sensor
    @param tempSampling the temp samping rate to use
    @param pressSampling the pressure sampling rate to use
    @param humSampling the humidity sampling rate to use
    @param filter the filter mode to use
    @param duration the standby duration to use
*/
/**************************************************************************/
void Aerospace::BME_setSampling(sensor_mode       mode,
		 sensor_sampling   tempSampling,
		 sensor_sampling   pressSampling,
		 sensor_sampling   humSampling,
		 sensor_filter     filter,
		 standby_duration  duration) {
    _measReg.mode     = mode;
    _measReg.osrs_t   = tempSampling;
    _measReg.osrs_p   = pressSampling;
        
    
    _humReg.osrs_h    = humSampling;
    _configReg.filter = filter;
    _configReg.t_sb   = duration;

    
    // you must make sure to also set REGISTER_CONTROL after setting the
    // CONTROLHUMID register, otherwise the values won't be applied (see DS 5.4.3)
    BME_write8(BME280_REGISTER_CONTROLHUMID, _humReg.get());
    BME_write8(BME280_REGISTER_CONFIG, _configReg.get());
    BME_write8(BME280_REGISTER_CONTROL, _measReg.get());
}

/**************************************************************************/
/*!
    @brief  Reads a signed 16 bit little endian value over I2C or SPI
    @param reg the register address to read from
    @returns the 16 bit data value read from the device
*/
/**************************************************************************/
uint16_t Aerospace::BME_read16_LE(byte reg) {
  uint16_t temp = read16(reg);
  return (temp >> 8) | (temp << 8);
}
