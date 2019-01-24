/*
UBX_I2C_example.ino
Eric De Mey
e.demey@bluewin.ch

Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

 #include <SPI.h>
 #include <SD.h>
const int chipSelect = 53; // sd_card CS

#include "UBX_I2C.h"
#include <Wire.h> //Needed for I2C to GPS
UBX_I2C gps;

void setup()
{
  // serial to display data
  Serial.begin(115200);
  Serial.println("Ublox GPS I2C Test");
  // starting communication with the GPS receiver

  gps.begin(Wire);
  if (gps.isConnected() == false)
  {
    Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }
  Wire.setClock(400000); //Increase I2C clock speed to 400kHz
  gps.sendCfg();

  Serial.println("Initializing SD card...");
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");
}

void loop() {
  uint8_t msg_code;
  String DataStr="";
  // checking to see if a good packet has
  // been received and displaying some
  // of the packet data
  msg_code = gps.readSensor();
  if (msg_code!=gps.MT_NONE){
    File dataFile = SD.open("gpslog.txt", FILE_WRITE);
    if (msg_code==gps.MT_NAV_ATT) {
      DataStr+=("Message NAV_ATT\n");
      DataStr+=(gps.getRoll());
      DataStr+=("\t");
      DataStr+=(gps.getPitch());
      DataStr+=("\t");
      DataStr+=(gps.getHeading());
      DataStr+=("\t");
      DataStr+=(gps.getAccRoll());
      DataStr+=("\t");
      DataStr+=(gps.getAccPitch());
      DataStr+=("\t");
      DataStr+=(gps.getAccHeading());
      DataStr+=("\n");
    }
    if (msg_code==gps.MT_NAV_PVT) {
      DataStr+=("Message NAV_PVT\n");
      DataStr+=(gps.getYear());                ///< [year], Year (UTC)
      DataStr+=("\t");
      DataStr+=(gps.getMonth());               ///< [month], Month, range 1..12 (UTC)
      DataStr+=("\t");
      DataStr+=(gps.getDay());                 ///< [day], Day of month, range 1..31 (UTC)
      DataStr+=("\t");
      DataStr+=(gps.getHour());                ///< [hour], Hour of day, range 0..23 (UTC)
      DataStr+=("\t");
      DataStr+=(gps.getMin());                 ///< [min], Minute of hour, range 0..59 (UTC)
      DataStr+=("\t");
      DataStr+=(gps.getSec());                 ///< [s], Seconds of minute, range 0..60 (UTC)
      DataStr+=("\t");
      DataStr+=(gps.getNumSatellites());       ///< [ND], Number of satellites used in Nav Solution
      DataStr+=("\t");
      DataStr+=(gps.getLatitude_deg());        ///< [deg], Latitude
      DataStr+=("\t");
      DataStr+=(gps.getLongitude_deg());       ///< [deg], Longitude
      DataStr+=("\t");
      DataStr+=(gps.getMSLHeight_m());         ///< [m], Height above mean sea level
      DataStr+=("\n");
    }
    if (msg_code==gps.MT_NAV_VEL) {
      DataStr+=("Message NAV_VEL\n");
      DataStr+=(gps.getVelX());
      DataStr+=("\t");
      DataStr+=(gps.getVelY());
      DataStr+=("\t");
      DataStr+=(gps.getVelZ());
      DataStr+=("\t");
      DataStr+=(gps.getVelAcc());
      DataStr+=("\n");
    }
    if (msg_code==gps.MT_ESF_INS) {
      DataStr+=("Message ESF_INS\n");
      DataStr+=(gps.getBitfield0());
      DataStr+=("\t");
      DataStr+=(gps.getxAngRate());
      DataStr+=("\t");
      DataStr+=(gps.getyAngRate());
      DataStr+=("\t");
      DataStr+=(gps.getzAngRate());
      DataStr+=("\t");
      DataStr+=(gps.getxAccel());
      DataStr+=("\t");
      DataStr+=(gps.getyAccel());
      DataStr+=("\t");
      DataStr+=(gps.getzAccel());
      DataStr+=("\n");
    }
    if (msg_code==gps.MT_ESF_STA)  {
      DataStr+=("Message ESF_STATUS\n");
      DataStr+=(gps.getFusionMode());
      DataStr+=("\t");
      DataStr+=(gps.getNumSens());
      DataStr+=("\n");
    }
    Serial.print(DataStr);
    dataFile.print(DataStr);
    dataFile.close() ;
  }
  delay(10); //Don't pound too hard on the bus
}