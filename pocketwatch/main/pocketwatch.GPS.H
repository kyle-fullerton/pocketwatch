#ifndef POCKETWATCH_GPS_H
#define POCKETWATCH_GPS_H

#include <SoftwareSerial.h>

#define POCKETWATCH__GPS__MAXDATACHARS 180

#define POCKETWATCH__GPS__HASRMC 0x01
#define POCKETWATCH__GPS__HASGGA 0x02

#define POCKETWATCH__GPS__DEGTORAD 0.017453292519943296

namespace pocketwatch
{

typedef unsigned long time_t;

uint8_t charToInt(char c);
const char* advancePastComma(const char* p, uint8_t numCharsNeeded);

struct GPSData
{
  GPSData();

  uint8_t yearSince2000;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;

  double latitude;
  double longitude;

  float groundSpeedKnots;
  float trackAngle;
  float altitudeMeters;

  uint8_t validFlags;
};

class GPS
{
public:
  GPS(uint8_t txPin, uint8_t rxPin);

  void start(time_t startTime);
  void process(time_t currentTime);

  const GPSData& getGPSData() const { return gpsData[activeGPSData]; }

private:

  void resetReceivedData();
  bool receivedDataIsComplete() const;
  bool receivedDataIsValid() const;
  void processGPSData(GPSData& data);
  void processRMCData(GPSData& data);
  void processGGAData(GPSData& data);
  bool gpsDataIsComplete(const GPSData& data) const;
  void activateGPSData(uint8_t newActiveData);

  SoftwareSerial serialConn;
  time_t lastReadTime;

  char receivedData[POCKETWATCH__GPS__MAXDATACHARS];
  uint8_t receivedDataCount;

  uint8_t activeGPSData;
  GPSData gpsData[2];
};

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
GPSData::GPSData() : yearSince2000(0),
                     month(0),
                     day(0),
                     hour(0),
                     minute(0),
                     second(0),
                     latitude(0.0),
                     longitude(0.0),
                     groundSpeedKnots(0.0f),
                     trackAngle(0.0f),
                     altitudeMeters(0.0f),
                     validFlags(0)
{
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
GPS::GPS(uint8_t txPin, uint8_t rxPin) : serialConn(txPin, rxPin),
                                         lastReadTime(0),
                                         receivedDataCount(0),
                                         activeGPSData(0)
{
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void GPS::start(time_t startTime)
{
  lastReadTime = startTime;

  serialConn.begin(9600);

  // Turn on Recommended Minimum Sentence C data and GGA data
  serialConn.println("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28");

  // Turn off antenna attachment status message - Adafruit library turns this on
  // TODO - Remove this when it's no longer needed
  serialConn.println("$PGCMD,33,0*6D");

  // Send data at 1 Hz
  serialConn.println("$PMTK220,1000*1F");
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void GPS::process(time_t currentTime)
{
  lastReadTime = currentTime;

  bool charactersRead = false;

  while (serialConn.available())
  {
    charactersRead = true;
    receivedData[receivedDataCount] = serialConn.read();
    ++receivedDataCount;
    if (receivedDataIsComplete())
    {
      uint8_t inactiveGPSData = (activeGPSData == 0) ? 1 : 0;

      processGPSData(gpsData[inactiveGPSData]);

      if (gpsDataIsComplete(gpsData[inactiveGPSData]))
      {
        activateGPSData(inactiveGPSData);
      }

      resetReceivedData();
    }

    // Prevent overflowint the buffer
    if (receivedDataCount >= POCKETWATCH__GPS__MAXDATACHARS)
    {
      receivedDataCount = 0;
    }
  }

  return;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void GPS::resetReceivedData() 
{
  receivedDataCount = 0;
  for (uint8_t i = 0; i < POCKETWATCH__GPS__MAXDATACHARS; ++i)
  {
    receivedData[i] = '\0';
  }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
bool GPS::receivedDataIsComplete() const
{
  // Complete data should end in "*xx\r\n", where xx is the checksum
  return ((receivedDataCount >= 5) &&
          (receivedData[receivedDataCount - 1] == '\n') &&
          (receivedData[receivedDataCount - 5] == '*'));
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
bool GPS::receivedDataIsValid() const
{
  uint8_t endChar = receivedDataCount - 5; // Should end in "*xx\r\n" as above
  if ((receivedDataCount < 12) || // 7 chars for the beginning + 5 for the end
      (receivedData[endChar] != '*'))
  {
    return false;
  }

  // These are the only message types we process.
  if ((strncmp(receivedData, "$GPRMC,", 7) != 0) &&
      (strncmp(receivedData, "$GPGGA,", 7) != 0))
  {
    return false;
  }

  byte calcChecksum = 0;
  for (uint8_t i = 1; i < endChar; ++i)
  {
    calcChecksum ^= (byte)receivedData[i];
  }

  byte expectedChecksum = charToInt(receivedData[endChar + 1]) * 16 +
                          charToInt(receivedData[endChar + 2]);

  return (calcChecksum == expectedChecksum);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void GPS::processGPSData(GPSData& data)
{
  if ( ! receivedDataIsValid())
  {
    return;
  }
  
  if (receivedData[3] == 'R') // Must be RMC, or receivedDataIsValid() would have failed
  {
    processRMCData(data);
  }
  else // Must be GGA
  {
    processGGAData(data);
  }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void GPS::processRMCData(GPSData& data)
{
  // $GPRMC,HHMMSS.SSS,S,LLMM.MMM,D,LLLMM.MMM,D,SSS.S,TTT.T,DDMMYY,,,D*CC\r\n
  // Dollar sign        - Beginning of message
  // GPRMC              - General-purpose message containing NMEA Recommended Minimum Sentence C
  // Time               - Hour, minute, second in UTC
  // Status             - either 'A' for Active or 'V' for Void
  // Latitude           - 2 digits for degrees and the rest is in minutes
  // Direction          - either 'N' for North or 'S' for South
  // Longitude          - 3 digits for degrees and the rest is in minutes
  // Direction          - either 'E' for East or 'W' for West
  // Speed              - Ground speed in knots
  // Track              - Direction of movement along the ground in degrees
  // Date               - Day, month, year in UTC (last 2 digits of year only)
  // Magnetic Variation - not supported by my GPS, but in degrees
  // Direction          - either 'E' for East or 'W' for West
  // D                  - No idea
  // *                  - End of message
  // Checksum           - Exclusive-OR of all characters within message, excluding '$' and '*'
  // \r\n               - Newline at the end of the message

  // "Replace" the "\r\n" at the string with a string terminator, for safety
  // because the rest of this code is very safe
  receivedData[receivedDataCount - 2] = '\0';

  const char* rest = advancePastComma(receivedData, 6);
  if (rest != NULL)
  {
    data.hour = charToInt(rest[0]) * 10 +
                charToInt(rest[1]);
    data.minute = charToInt(rest[2]) * 10 +
                  charToInt(rest[3]);
    data.second = charToInt(rest[4]) * 10 +
                  charToInt(rest[5]);
  }
  else
  {
    data.hour = 0;
    data.minute = 0;
    data.second = 0;
  }

  rest = advancePastComma(rest, 1);
  bool isActive = false;
  if (rest != NULL)
  {
    isActive = (rest[0] == 'A');
  }
  // else leave it false

  rest = advancePastComma(rest, 4);
  if (rest != NULL)
  {
    if (isActive)
    {
      data.latitude = charToInt(rest[0]) * 10 +
                      charToInt(rest[1]) +
                      atof(rest + 2) / 60.0;
      data.latitude *= POCKETWATCH__GPS__DEGTORAD;
    }
  }
  else
  {
    data.latitude = 0.0;
  }

  rest = advancePastComma(rest, 1);
  if (rest != NULL)
  {
    if (rest[0] == 'S')
    {
      data.latitude *= -1.0;
    }
  }

  rest = advancePastComma(rest, 5);
  if (rest != NULL)
  {
    if (isActive)
    {
      data.longitude = charToInt(rest[0]) * 100 +
                       charToInt(rest[1]) * 10 +
                       charToInt(rest[2]) +
                       atof(rest + 3) / 60.0;
      data.longitude *= POCKETWATCH__GPS__DEGTORAD;
    }
  }
  else
  {
    data.longitude = 0.0;
  }

  rest = advancePastComma(rest, 1);
  if (rest != NULL)
  {
    if (rest[0] == 'W')
    {
      data.longitude *= -1.0;
    }
  }

  rest = advancePastComma(rest, 2);
  if (rest != NULL)
  {
    data.groundSpeedKnots = atof(rest);
  }
  else
  {
    data.groundSpeedKnots = 0.0f;
  }

  rest = advancePastComma(rest, 2);
  if (rest != NULL)
  {
    data.trackAngle = atof(rest);
    data.trackAngle *= POCKETWATCH__GPS__DEGTORAD;
  }
  else
  {
    data.trackAngle = 0.0f;
  }
  
  rest = advancePastComma(rest, 6);
  if (rest != NULL)
  {
    data.day = charToInt(rest[0]) * 10 +
               charToInt(rest[1]);
    data.month = charToInt(rest[2]) * 10 +
                 charToInt(rest[3]);
    data.yearSince2000 = charToInt(rest[4]) * 10 +
                         charToInt(rest[5]);
  }

  data.validFlags |= POCKETWATCH__GPS__HASRMC;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void GPS::processGGAData(GPSData& data)
{
  // $GPGGA,HHMMSS.SSS,LLMM.MMM,D,LLLMM.MMM,D,F,NN,DDDD,AAAAA,M,GGG,M,TTTT,RRRR,*CC\r\n
  // Dollar sign        - Beginning of message
  // GPGGA              - General-purpose message containing NMEA GPS Fix Data
  // Time               - Hour, minute, second in UTC
  // Latitude           - 2 digits for degrees and the rest is in minutes
  // Direction          - either 'N' for North or 'S' for South
  // Longitude          - 3 digits for degrees and the rest is in minutes
  // Direction          - either 'E' for East or 'W' for West
  // Fix Quality        - either '0' for Invalid, '1' for GPS fix, or '2' for DGPS fix
  // Num Satellites     - Number of satellites in view
  // Dilution           - Expected error in horizontal position
  // Altitude           - Altitude of antenna
  // Units              - 'M' for Meters of altitude of antenna
  // Geoid height       - Height of geoid above WGS84 ellipsoid
  // Units              - 'M' for Meters of geoid height
  // DGPS update time   - Time since last DGPS update
  // DGPS reference ID  - ID of the DGPS reference station that provided the last update
  // *                  - End of message
  // Checksum           - Exclusive-OR of all characters within message, excluding '$' and '*'
  // \r\n               - Newline at the end of the message

  // "Replace" the "\r\n" at the string with a string terminator, for safety
  // because the rest of this code is very safe
  receivedData[receivedDataCount - 2] = '\0';
  const char* rest = advancePastComma(receivedData, 6); // Header

  // Skip over a lot of things we don't care about:
  rest = advancePastComma(rest, 1); // Time
  rest = advancePastComma(rest, 1); // Latitude
  rest = advancePastComma(rest, 1); // Direction
  rest = advancePastComma(rest, 1); // Longitude
  rest = advancePastComma(rest, 1); // Direction
  rest = advancePastComma(rest, 1); // Fix Quality
  rest = advancePastComma(rest, 1); // Num Satellites
  rest = advancePastComma(rest, 1); // Dilution

  if (rest != NULL)
  {
    data.altitudeMeters = atof(rest);
  }
  else
  {
    data.altitudeMeters = 0.0f;
  }
  
  rest = advancePastComma(rest, 1);
  if (rest != NULL)
  {
    if (rest[0] != 'M')
    {
      data.altitudeMeters = -1.0f;
    }
  }

  // Just ignore the rest of the message
 
  data.validFlags |= POCKETWATCH__GPS__HASGGA;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
bool GPS::gpsDataIsComplete(const GPSData& data) const
{
  return ((data.validFlags & POCKETWATCH__GPS__HASRMC) &&
          (data.validFlags & POCKETWATCH__GPS__HASGGA));
}
  
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void GPS::activateGPSData(uint8_t newActiveData)
{
  // First invalidate the old one.
  gpsData[activeGPSData].validFlags = 0;

  // Now swap in the new one.
  activeGPSData = newActiveData;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
const char* advancePastComma(const char* p, uint8_t numCharsNeeded)
{
  if (p == NULL)
  {
    return NULL;
  }
  
  const char* retVal = strchr(p, ',');
  if (retVal != NULL)
  {
    // The first character of retVal is guaranteed to be ',' so advance it by one
    ++retVal;
    if (strlen(retVal) < numCharsNeeded)
    {
      // It's not long enough so it won't work
      retVal = NULL;
    }
  }
  return retVal;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
uint8_t charToInt(char c)
{
  if (c >= '0' && c <= '9')
  {
    return (uint8_t)(c - '0');
  }
  else if (c >= 'A' && c <= 'F')
  {
    return (uint8_t)(c - 'A' + 10);
  }
  else if (c >= 'a' && c <= 'f')
  {
    return (uint8_t)(c - 'a' + 10);
  }
  else
  {
    return 0;
  }
}
} // end namespace pocketwatch

#endif
