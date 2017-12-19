// Flora GPS + LED Pixel Code
//
// This code shows how to listen to the GPS module in an interrupt
// which allows the program to have more 'freedom' - just parse
// when a new NMEA sentence is available! Then access data when
// desired.
//
// Tested and works great with the Adafruit Flora GPS module
//    ------> http://adafruit.com/products/1059
// Pick one up today at the Adafruit electronics shop
// and help support open source hardware & software! -ada

#include <Adafruit_GPS.h>
#include <Adafruit_NeoPixel.h>

// what's the name of the hardware serial port?
#define GPSSerial Serial1

// Set GPS_DEBUG to 'false' to turn off echoing the GPS data to the Serial console
#define GPS_DEBUG false

//--------------------------------------------------|
//              WAYPOINT CONFIG                     |
//--------------------------------------------------|
//Please enter the latitude and longitude of your   |
//desired destination:                              |
  #define GEO_LAT                53.342302
  #define GEO_LON               -6.238722
//--------------------------------------------------|

//--------------------------------------------------|
//        WAYPOINT PROXIMITY CONFIG                 |
//--------------------------------------------------|
//Please enter the distance (in meters) from your   |
//destination that you want your LEDs to light up:  |
  #define AT_TARGET_RADIUS          25
  #define CLOSE_TO_TARGET_RADIUS    50
//--------------------------------------------------|

//--------------------------------------------------|
//        BASIC LIGHT CONFIG                        |
//--------------------------------------------------|
#define PIN 8
//--------------------------------------------------|


// Setup Basic Light Strip
Adafruit_NeoPixel strip = Adafruit_NeoPixel(1, PIN, NEO_GRB + NEO_KHZ800);
// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

// Navigation location
float targetLat = GEO_LAT;
float targetLon = GEO_LON;

// Trip distance
float tripDistance;

boolean isStarted = false;

uint32_t timer = millis();


void setup()
{
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  Serial.begin(115200);
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // Turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  delay(5000);

  setupLights();
}

void loop()                     // run over and over again
{
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPS_DEBUG)
        if (c) Serial.print(c);
    // if a sentence is received, we can check the checksum, parse it...
    if (GPS.newNMEAreceived()) {
        if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
            return; // we can fail to parse a sentence in which case we should just wait for another
    }

    // if millis() or timer wraps around, we'll just reset it
    if (timer > millis()) timer = millis();

    // approximately every 2 seconds or so, print out the current stats
    if (millis() - timer > 2000) {
        timer = millis(); // reset the timer
            printBasicGpsData();
        if (GPS.fix) {
            printGpsFixData();
            float fLat = decimalDegrees(GPS.latitude, GPS.lat);
            float fLon = decimalDegrees(GPS.longitude, GPS.lon);

            if (!isStarted) {
                isStarted = true;
                 tripDistance = (double)calc_dist(fLat, fLon, targetLat, targetLon);
            }
            proximityFunction((double)calc_dist(fLat, fLon, targetLat, targetLon));
            //Serial.print("Distance Remaining:"); Serial.println((double)calc_dist(fLat, fLon, targetLat, targetLon));
        } else {
            noGpsFixAction();
        }
    }
}

int calc_bearing(float flat1, float flon1, float flat2, float flon2)
{
    float calc;
    float bear_calc;

    float x = 69.1 * (flat2 - flat1);
    float y = 69.1 * (flon2 - flon1) * cos(flat1/57.3);

    calc=atan2(y,x);

    bear_calc= degrees(calc);

    if(bear_calc<=1){
      bear_calc=360+bear_calc;
    }
    return bear_calc;
}


void proximityFunction(float fDist)
{
    if ((fDist < AT_TARGET_RADIUS)) { 
        gpsFixAtTargetAction();
    }
    else if ((fDist < CLOSE_TO_TARGET_RADIUS)) {
        gpsFixCloseToTargetAction();
    }
    else if ((fDist >= CLOSE_TO_TARGET_RADIUS)) {
        gpsFixFarAwayAction();
  }
}

unsigned long calc_dist(float flat1, float flon1, float flat2, float flon2)
{
  float dist_calc=0;
  float dist_calc2=0;
  float diflat=0;
  float diflon=0;

  diflat=radians(flat2-flat1);
  flat1=radians(flat1);
  flat2=radians(flat2);
  diflon=radians((flon2)-(flon1));

  dist_calc = (sin(diflat/2.0)*sin(diflat/2.0));
  dist_calc2= cos(flat1);
  dist_calc2*=cos(flat2);
  dist_calc2*=sin(diflon/2.0);
  dist_calc2*=sin(diflon/2.0);
  dist_calc +=dist_calc2;

  dist_calc=(2*atan2(sqrt(dist_calc),sqrt(1.0-dist_calc)));

  dist_calc*=6371000.0; //Converting to meters
  return dist_calc;
}

// Convert NMEA coordinate to decimal degrees
float decimalDegrees(float nmeaCoord, char dir) {
    uint16_t wholeDegrees = 0.01*nmeaCoord;
    int modifier = 1;

    if (dir == 'W' || dir == 'S') {
        modifier = -1;
    }

    return (wholeDegrees + (nmeaCoord - 100.0*wholeDegrees)/60.0) * modifier;
}

//--------------------------------------------------|
//         GPS ACTION FUNCTIONS                     |
//--------------------------------------------------|
void noGpsFixAction() {
    flashRed();
}

void gpsFixFarAwayAction() {
    flashGreen();
}

void gpsFixCloseToTargetAction() {
    flashTwoColours();
}

void gpsFixAtTargetAction() {
    flashMultiColour();
}
//--------------------------------------------------|



//--------------------------------------------------|
//        BASIC LIGHT FUNCTIONS                     |
//--------------------------------------------------|
void flashMultiColour() {
    colorWipe(strip.Color(255, 0, 0), 500); // Red
    colorWipe(strip.Color(0, 255, 0), 500); // Green
    colorWipe(strip.Color(0, 0, 255), 500); // Blue
    colorWipe(strip.Color(0, 0, 0), 0); // OFF
}
void flashTwoColours() {
    colorWipe(strip.Color(0, 255, 0), 500); // Green
    colorWipe(strip.Color(0, 0, 255), 500); // Blue
    colorWipe(strip.Color(0, 0, 0), 0); // OFF
}
void flashGreen() {
    colorWipe(strip.Color(0, 255, 0), 500); // Green
    colorWipe(strip.Color(0, 0, 0), 0); // OFF
}
void flashRed() {
    colorWipe(strip.Color(255, 0, 0), 500); // Red
    colorWipe(strip.Color(0, 0, 0), 0); // OFF
}
// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
    for(uint16_t i=0; i<strip.numPixels(); i++) {
        strip.setPixelColor(i, c);
        strip.show();
        delay(wait);
    }
}
void setupLights() {
    // Start up the LED strip
    strip.begin();
    // Update the strip, to start they are all 'off'
    strip.show();
}
//--------------------------------------------------|


//--------------------------------------------------|
//          GPS DEBUG FUNCTIONS                     |
//--------------------------------------------------|
void printBasicGpsData() {
    if (GPS_DEBUG) {
        Serial.print("\nTime: ");
        Serial.print(GPS.hour, DEC); Serial.print(':');
        Serial.print(GPS.minute, DEC); Serial.print(':');
        Serial.print(GPS.seconds, DEC); Serial.print('.');
        Serial.println(GPS.milliseconds);
        Serial.print("Date: ");
        Serial.print(GPS.day, DEC); Serial.print('/');
        Serial.print(GPS.month, DEC); Serial.print("/20");
        Serial.println(GPS.year, DEC);
        Serial.print("Fix: "); Serial.print((int)GPS.fix);
        Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    }
}
void printGpsFixData() {
    if (GPS_DEBUG) {
        Serial.print("\nTime: ");
        Serial.print(GPS.hour, DEC); Serial.print(':');
        Serial.print(GPS.minute, DEC); Serial.print(':');
        Serial.print(GPS.seconds, DEC); Serial.print('.');
        Serial.println(GPS.milliseconds);
        Serial.print("Date: ");
        Serial.print(GPS.day, DEC); Serial.print('/');
        Serial.print(GPS.month, DEC); Serial.print("/20");
        Serial.println(GPS.year, DEC);
        Serial.print("Fix: "); Serial.print((int)GPS.fix);
        Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    }
}


