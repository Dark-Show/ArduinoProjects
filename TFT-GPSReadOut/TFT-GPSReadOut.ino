// Based off of the Adafruit_GPS examples coupled with with Adafruit_TFTLCD examples

#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_TFTLCD.h> // Hardware-specific library

#define GPS_RX 6 // Serial Receive Pin
#define GPS_TX 7 // Serial Transmit Pin

#define LCD_CS A3 // Chip Select goes to Analog 3
#define LCD_CD A2 // Command/Data goes to Analog 2
#define LCD_WR A1 // LCD Write goes to Analog 1
#define LCD_RD A0 // LCD Read goes to Analog 0
#define LCD_RESET A4 // Can alternately just connect to Arduino's reset pin

#define BLACK   0x0000
#define WHITE   0xFFFF

Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);

SoftwareSerial mySerial(GPS_RX, GPS_TX);
Adafruit_GPS GPS(&mySerial);

uint32_t timer = 0;

void setup() {

  tft.reset();

  uint16_t identifier = tft.readID();

  if(identifier == 0x9325) {
    Serial.println(F("Found ILI9325 LCD driver"));
  } else if(identifier == 0x9328) {
    Serial.println(F("Found ILI9328 LCD driver"));
  } else if(identifier == 0x7575) {
    Serial.println(F("Found HX8347G LCD driver"));
  } else if(identifier == 0x9341) {
    Serial.println(F("Found ILI9341 LCD driver"));
  } else if(identifier == 0x8357) {
    Serial.println(F("Found HX8357D LCD driver"));
  } else {
    Serial.print(F("Unknown LCD driver chip: "));
    Serial.println(identifier, HEX);
    return;
  }

  tft.begin(identifier);

  tft.fillScreen(BLACK);
  pinMode(6, INPUT);
  
  Serial.begin(115200);
  
  GPS.begin(9600); // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY); // uncomment this line to turn on only the "minimum recommended" data
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  mySerial.println(PMTK_Q_RELEASE); // Ask for firmware version
}


void loop() {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (c)
    Serial.write(c);

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) {
    pinMode(6, OUTPUT);
    tft.fillScreen(BLACK);
    tft.setTextSize(1);
    tft.setTextColor(WHITE);
    tft.setCursor(0, 0);
    
    tft.print("\nTime: ");
    if (GPS.hour < 10) { tft.print('0'); }
    tft.print(GPS.hour, DEC); tft.print(':');
    if (GPS.minute < 10) { tft.print('0'); }
    tft.print(GPS.minute, DEC); tft.print(':');
    if (GPS.seconds < 10) { tft.print('0'); }
    tft.print(GPS.seconds, DEC); tft.print('.');
    if (GPS.milliseconds < 10) {
      tft.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      tft.print("0");
    }
    tft.println(GPS.milliseconds);
    tft.print("Date: ");
    tft.print(GPS.day, DEC); tft.print('/');
    tft.print(GPS.month, DEC); tft.print("/20");
    tft.println(GPS.year, DEC);
    tft.print("Fix: "); tft.print((int)GPS.fix);
    tft.print(" quality: "); tft.println((int)GPS.fixquality);
    if (GPS.fix) {
      tft.print("Location: ");
      tft.print(GPS.latitude, 4); tft.print(GPS.lat);
      tft.print(", ");
      tft.print(GPS.longitude, 4); tft.println(GPS.lon);

      tft.print("Speed (km/h): "); tft.println(GPS.speed * 1.85); // Convert knots to km (approx)
      tft.print("Angle: "); tft.println(GPS.angle);
      tft.print("Altitude: "); tft.println(GPS.altitude);
      tft.print("Satellites: "); tft.println((int)GPS.satellites);
    }
    pinMode(6, INPUT);
    timer = millis(); // reset the timer
  }
}
