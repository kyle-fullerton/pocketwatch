


#include "pocketwatch.Blinker.H"
#include "pocketwatch.Compass.H"
#include "pocketwatch.Display.H"
#include "pocketwatch.GPS.H"
#include "pocketwatch.Selector.H"
#include "pocketwatch.Types.H"
#include "pocketwatch.WaypointKeeper.H"

#define POCKETWATCH__PINOUT__LED 13
#define POCKETWATCH__PINOUT__GPS__TX 3
#define POCKETWATCH__PINOUT__GPS__RX 2
#define POCKETWATCH__PINOUT__SELECTOR_INPUT A0
#define POCKETWATCH__PINOUT__BUTTON A1
#define MSEC 1
#define SEC (1000 * MSEC)

//#define DEBUG 1


pocketwatch::Blinker blinker;
pocketwatch::Compass compass;
pocketwatch::Display displayer;
pocketwatch::GPS gps(POCKETWATCH__PINOUT__GPS__TX, POCKETWATCH__PINOUT__GPS__RX);
pocketwatch::Selector selector;
pocketwatch::WaypointKeeper waypointKeeper;


void setup() {
  unsigned long currentTime = millis();
  
  blinker.start(POCKETWATCH__PINOUT__LED,
                currentTime,
                2 * SEC,
                2 * SEC);
  compass.start(currentTime,
                200 * MSEC);
  gps.start(currentTime);
  selector.start(POCKETWATCH__PINOUT__SELECTOR_INPUT,
                 currentTime,
                 50 * MSEC,
                 250 * MSEC,
                 4,
                 330.0/340.0);
  waypointKeeper.start(POCKETWATCH__PINOUT__BUTTON,
                       currentTime,
                       200 * MSEC,
                       3 * SEC, 
                       2 * SEC,
                       10 * SEC,
                       2,
                       4);

  pocketwatch::types::MotorConfig bigConfig;
  bigConfig.pinA1 = 8;
  bigConfig.pinA2 = 11;
  bigConfig.pinB1 = 9;
  bigConfig.pinB2 = 12;

  displayer.start(currentTime, 500 * MSEC, 3 * MSEC, 120, bigConfig);

  delay(100);

  
}

void loop() {
  unsigned long currentTime = millis();
  
  blinker.process(currentTime);
  compass.process(currentTime);
  gps.process(currentTime);
  selector.process(currentTime);
  const pocketwatch::GPSData& gpsData = gps.getGPSData();
  waypointKeeper.process(currentTime,
                         gpsData.latitude,
                         gpsData.longitude);

  pocketwatch::types::SensorData data;
  data.selection = selector.getChoice();
  data.heading = compass.getHeading();
  data.compassX = compass.getX();
  data.compassY = compass.getY();
  data.compassZ = compass.getZ();
  data.yearSince2000 = gpsData.yearSince2000;
  data.month = gpsData.month;
  data.day = gpsData.day;
  data.hour = gpsData.hour;
  data.minute = gpsData.minute;
  data.second = gpsData.second;
  data.latitude = gpsData.latitude;
  data.longitude = gpsData.longitude;
  data.groundSpeedKnots = gpsData.groundSpeedKnots;
  data.trackAngle = gpsData.trackAngle;
  data.altitudeMeters = gpsData.altitudeMeters;
  data.fastWaypointLatitude = waypointKeeper.getFastLat();
  data.fastWaypointLongitude = waypointKeeper.getFastLon();
  data.slowWaypointLatitude = waypointKeeper.getSlowLat();
  data.slowWaypointLongitude = waypointKeeper.getSlowLon();


  displayer.process(currentTime, data);
}
