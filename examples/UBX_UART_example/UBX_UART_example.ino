/*
UBX_UART_example.ino
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

#include "UBX_UART.h"

#include <SoftwareSerial.h>
SoftwareSerial ss(4, 5); // RX, TX

UBX_UART gps(&ss);

void setup()
{
  // serial to display data
  Serial.begin(115200);
  Serial.println("Ublox GPS UART Test");
  // starting communication with the GPS receiver
  gps.begin(9600);
  gps.sendCfg();
}

void loop() {
  uint8_t msg_code;
  // checking to see if a good packet has
  // been received and displaying some
  // of the packet data
  msg_code = gps.readSensor();
  if (msg_code==gps.MT_NAV_ATT) {
    Serial.println("Message NAV_ATT");
    Serial.print(gps.getRoll());
    Serial.print("\t");
    Serial.print(gps.getPitch());
    Serial.print("\t");
    Serial.print(gps.getHeading());
    Serial.print("\t");
    Serial.print(gps.getAccRoll());
    Serial.print("\t");
    Serial.print(gps.getAccPitch());
    Serial.print("\t");
    Serial.println(gps.getAccHeading());
  }
  if (msg_code==gps.MT_NAV_PVT) {
    Serial.println("Message NAV_PVT");
    Serial.print(gps.getYear());                ///< [year], Year (UTC)
    Serial.print("\t");
    Serial.print(gps.getMonth());               ///< [month], Month, range 1..12 (UTC)
    Serial.print("\t");
    Serial.print(gps.getDay());                 ///< [day], Day of month, range 1..31 (UTC)
    Serial.print("\t");
    Serial.print(gps.getHour());                ///< [hour], Hour of day, range 0..23 (UTC)
    Serial.print("\t");
    Serial.print(gps.getMin());                 ///< [min], Minute of hour, range 0..59 (UTC)
    Serial.print("\t");
    Serial.print(gps.getSec());                 ///< [s], Seconds of minute, range 0..60 (UTC)
    Serial.print("\t");
    Serial.print(gps.getNumSatellites());       ///< [ND], Number of satellites used in Nav Solution
    Serial.print("\t");
    Serial.print(gps.getLatitude_deg());     ///< [deg], Latitude
    Serial.print("\t");
    Serial.print(gps.getLongitude_deg());    ///< [deg], Longitude
    Serial.print("\t");
    Serial.println(gps.getMSLHeight_m());       ///< [m], Height above mean sea level
  }
  if (msg_code==gps.MT_ESF_INS) {
    Serial.println("Message ESF_INS");
    Serial.print(gps.getBitfield0());
    Serial.print("\t");
    Serial.print(gps.getxAngRate());
    Serial.print("\t");
    Serial.print(gps.getyAngRate());
    Serial.print("\t");
    Serial.print(gps.getzAngRate());
    Serial.print("\t");
    Serial.print(gps.getxAccel());
    Serial.print("\t");
    Serial.print(gps.getyAccel());
    Serial.print("\t");
    Serial.println(gps.getzAccel());
  }
  if (msg_code==gps.MT_ESF_STA)  {
    Serial.println("Message ESF_STATUS");
    Serial.print(gps.getFusionMode());
    Serial.print("\t");
    Serial.println(gps.getNumSens());
  }
  delay(10); //Don't pound too hard on the bus
}