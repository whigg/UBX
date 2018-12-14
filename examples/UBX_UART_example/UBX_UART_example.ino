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
SoftwareSerial ss(10, 11); // RX, TX
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
  if (msg_code==gps.MT_NAV_PVT) {
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
    Serial.print(gps.getLatitude_deg(),10);     ///< [deg], Latitude
    Serial.print("\t");
    Serial.print(gps.getLongitude_deg(),10);    ///< [deg], Longitude
    Serial.print("\t");
    Serial.println(gps.getMSLHeight_m());       ///< [m], Height above mean sea level
  }
  if (msg_code==gps.MT_ESF_INS) {
    Serial.print(gps.getBitfield0(),10);
    Serial.print("\t");
    Serial.print(gps.getxAngRate(),10);
    Serial.print("\t");
    Serial.print(gps.getyAngRate(),10);
    Serial.print("\t");
    Serial.print(gps.getzAngRate(),10);
    Serial.print("\t");
    Serial.print(gps.getxAccel(),10);
    Serial.print("\t");
    Serial.print(gps.getyAccel(),10);
    Serial.print("\t");
    Serial.println(gps.getzAccel(),10);
  }
  if (msg_code==gps.MT_ESF_MEA) {
    Serial.print(gps.getMeaData0(),10);
    Serial.print("\t");
    Serial.print(gps.getMeaData1(),10);
    Serial.print("\t");
    Serial.print(gps.getMeaData2(),10);
    Serial.print("\t");
    Serial.print(gps.getMeaData3(),10);
    Serial.print("\t");
    Serial.print(gps.getMeaData4(),10);
    Serial.print("\t");
    Serial.print(gps.getMeaData5(),10);
    Serial.print("\t");
    Serial.println(gps.getMeaData6(),10);
    Serial.print(gps.getMeaCalibTtag0(),10);
    Serial.print("\t");
    Serial.print(gps.getMeaCalibTtag1(),10);
    Serial.print("\t");
    Serial.print(gps.getMeaCalibTtag2(),10);
    Serial.print("\t");
    Serial.print(gps.getMeaCalibTtag3(),10);
    Serial.print("\t");
    Serial.print(gps.getMeaCalibTtag4(),10);
    Serial.print("\t");
    Serial.print(gps.getMeaCalibTtag5(),10);
    Serial.print("\t");
    Serial.println(gps.getMeaCalibTtag6(),10);
  }
  if (msg_code==gps.MT_ESF_RAW)  {
    Serial.print(gps.getRawData0(),10);
    Serial.print("\t");
    Serial.print(gps.getRawData1(),10);
    Serial.print("\t");
    Serial.print(gps.getRawData2(),10);
    Serial.print("\t");
    Serial.print(gps.getRawData3(),10);
    Serial.print("\t");
    Serial.print(gps.getRawData4(),10);
    Serial.print("\t");
    Serial.print(gps.getRawData5(),10);
    Serial.print("\t");
    Serial.println(gps.getRawData6(),10);
    Serial.print(gps.getRawsTtag0(),10);
    Serial.print("\t");
    Serial.print(gps.getRawsTtag1(),10);
    Serial.print("\t");
    Serial.print(gps.getRawsTtag2(),10);
    Serial.print("\t");
    Serial.print(gps.getRawsTtag3(),10);
    Serial.print("\t");
    Serial.print(gps.getRawsTtag4(),10);
    Serial.print("\t");
    Serial.print(gps.getRawsTtag5(),10);
    Serial.print("\t");
    Serial.println(gps.getRawsTtag6(),10);
  }
  if (msg_code==gps.MT_ESF_STA)  {
    Serial.print(gps.getFusionMode(),10);
    Serial.print("\t");
    Serial.println(gps.getNumSens(),10);
  }
  delay(50); //Don't pound too hard on the bus
}