// LOOP data packet length and structure.
// See http://www.davisnet.com/support/weather/download/VantageSerialProtocolDocs_v230.pdf
// The CRC is not included in here but calculated on the fly.
// WARNING: Some of the items below are not implemented!!!
struct __attribute__((packed)) LoopPacket
{
  char loo[3];                // "LOO" at packet start indicates Rev B packet type
  byte barTrend;              // Barometric trend
  byte packetType;            // Packet type, always zero
  uint16_t nextRecord;        // Location in archive memory where next packet will be written
  uint16_t barometer;         // Current barometer in Hg / 1000
  int16_t insideTemperature;  // Inside temperature in tenths of degrees
  byte insideHumidity;        // Inside relative humidity in percent
  int16_t outsideTemperature; // Outside temperature in tenths of degrees

  byte windSpeed;             // Wind speed in miles per hour
  byte tenMinAvgWindSpeed;    // Average wind speed over last ten minutes

  uint16_t windDirection;     // Wind direction from 1 to 360 degrees (0 = no wind data)
  byte extraTemperatures[7];  // Temps from seven extra stations in whole degrees F offset by 90
  byte soilTemperatures[4];   // Soil temps from four extra sensors.  Format as above.
  byte leafTemperatures[4];   // Leaf temps from four extra sensors.  Format as above.
  byte outsideHumidity;       // Outside relative humidity in %.
  byte extraHumidities[7];    // Relative humidity in % from seven extra stations.
  uint16_t rainRate;          // Rain rate as number of rain clicks per hour (e.g 256 = 2.56 in/hr)
  byte uV;                    // UV index
  uint16_t solarRadiation;    // Solar radiation in Watts/m^2
  uint16_t stormRain;         // Storm rain stored as hundredths of an inch
  uint16_t startDateOfStorm;  // Bits 15-12 is month, bits 11-7 is day, and bits 6-0 is year offset by 2000
  uint16_t dayRain;           // Rain today sent as number of rain clicks (0.2mm or 0.01in)
  uint16_t monthRain;         // Rain this month sent as number of rain clicks (0.2mm or 0.01in)
  uint16_t yearRain;          // Rain this year sent as number of rain clicks (0.2mm or 0.01in)
  uint16_t dayET;             // ET today sent as thousandths of an inch
  uint16_t monthET;           // ET this month sent as hundredths of an inch
  uint16_t yearET;            // ET this year sent as hundredths of an inch
  byte soilMoistures[4];      // Soil moisture in centibars from up to four soil sensors
  byte leafWetnesses[4];      // A scale number from 0-15. 0 means very dry, 15 very wet. Supports four leaf sensors.
  byte insideAlarms;          // Currently active inside alarms.
  byte rainAlarms;            // Currently active rain alarms.
  uint16_t outsideAlarms;     // Currently active outside alarms.
  byte extraTempHumAlarms[8]; // Currently active temperature / humidity alarms for up to eight stations.
  byte soilLeafAlarms[4];     // Currently active soil and leaf alarms for up to four sensors
  byte transmitterBatteryStatus;  // Transmitter battery status (0 or 1)
  uint16_t consoleBatteryVoltage; // Console voltage as  ((Data * 300)/512)/100.0
  byte forecastIcons;             // Forecast icons
  byte forecastRuleNumber;        // Forecast rule number
  uint16_t timeOfSunrise;         // Sunrise time stored as hour * 100 + min.
  uint16_t timeOfSunset;          // Sunset time stored as hour * 100 + min.
  char lfcr[2];                   // Carriage return / line feed
};

// Initializer for the LOOP packet.  Static const so it goes into flash.
//static const LoopPacket loopInit =
const LoopPacket loopInit =
{
  {'L', 'O', 'O'}, // Identifies this as a Rev B packet
  255,             // Signal barometric trend as not known
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  {122, 122, 122, 122, 122, 122, 122},
  {122, 122, 122, 122},
  {122, 122, 122, 122},
  0,
  {255, 255, 255, 255, 255, 255, 255},
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  {0, 0, 0, 0},
  {0, 0, 0, 0},
  0,
  0,
  0,
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0},
  0,
  0,
  0,
  0,
  0,
  0,
  {'\n', '\r'}  // End of packet line feed and carriage return
};


// --- archive record type for DMP and DMPAFT commands
struct __attribute__((packed)) ArchiveRec
{
  uint16_t dateStamp;
  uint16_t timeStamp;
  int16_t  outsideTemp;
  int16_t  highOutTemp;
  int16_t  lowOutTemp;
  uint16_t rainfall;
  uint16_t highRainRate;
  uint16_t barometer;
  uint16_t solarRad;
  uint16_t windSamples;
  int16_t  insideTemp;
  byte     insideHum;
  byte     outsideHum;
  byte     avgWindSpd;
  byte     highWindSpd;
  byte     dirHiWindSpd;
  byte     prevWindDir;
  byte     avgUVIndex;
  byte     eT;
  uint16_t highSolarRad;
  byte     highUVIdx;
  byte     forecastRule;
  byte     leafTemp0;
  byte     leafTemp1;
  byte     leafWet0;
  byte     leafWet1;
  byte     soilTemp0;
  byte     soilTemp1;
  byte     soilTemp2;
  byte     soilTemp3;
  byte     recType;
  byte     extraHum0;
  byte     extraHum1;
  byte     extraTemp0;
  byte     extraTemp1;
  byte     extraTemp2;
  byte     soilMoist0;
  byte     soilMoist1;
  byte     soilMoist2;
  byte     soilMoist3;
};
