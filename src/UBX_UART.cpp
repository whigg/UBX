/*
UBX.cpp
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

#include "Arduino.h"
#include "UBX_UART.h"

/* uBlox object, input the serial bus and baud rate */
UBX_UART::UBX_UART(SoftwareSerial *bus)
{
  _bus = bus;
}

/* starts the serial communication */
void UBX_UART::begin(uint32_t baud)
{
	_bus->begin(baud);
}

//Polls I2C for data, passing any new bytes to process()
//Times out after given amount of time
uint8_t UBX_UART::readSensor()
{
  _parserState = 0;
  _tempPacket.msg_length = 4;
  _currentMsgType = MT_NONE;
  uint8_t counter = 0;// the buffer is only 64 BYTE long.
  if((_bus->available()))
  {  
    while (((_currentMsgType==MT_NONE))&&(counter<3))
    {   
        while (_bus->available())
		_currentMsgType=_parse(_bus->read());
	delay(50); // wait for buffer refill
	counter++;
    }
  }	
  return (_currentMsgType);

} //end checkUbloxI2C()

/* parse the UBX data */
uint8_t UBX_UART::_parse(uint8_t _byte)
{   
		// identify the packet header
		if (_parserState < 2) {
			if (_byte == _ubxHeader[_parserState]) {
				_parserState++;
			} else {
				_parserState = 0;
			}
		} else {
			if ((_parserState - 2) < (_tempPacket.msg_length+4)) {
				*((uint8_t *) &_tempPacket + _parserState - 2) = _byte;
			}
			_parserState++;
			// compute checksum
			if ((_parserState - 2) == (_tempPacket.msg_length+4)) {
				_calcChecksum(_checksum,((uint8_t *) &_tempPacket),(_tempPacket.msg_length+4));
			} else if ((_parserState - 2) == (_tempPacket.msg_length+5)) {
				if (_byte != _checksum[0]) {
					_parserState = 0;
				}
			} else if ((_parserState - 2) == (_tempPacket.msg_length+6)) {
				_parserState = 0;
				if (_byte == _checksum[1]) {
					switch (_tempPacket.msg_class_id) {
					case 0x0701: //NAV_PVT
						memcpy(&_NavPvtPacket,&_tempPacket,sizeof(_NavPvtPacket));
						return MT_NAV_PVT;
					case 0x1510: //ESF_INS
						memcpy(&_EsfInsPacket,&_tempPacket,sizeof(_EsfInsPacket));
						return MT_ESF_INS;
					case 0x1010: //ESF_STATUS
						memcpy(&_EsfStaPacket,&_tempPacket,sizeof(_EsfStaPacket));
						return MT_ESF_STA;
					default:
						return MT_NONE;
					}
				}
			} else if (_parserState > sizeof(_tempPacket)) {
				_parserState = 0;
			}
		}
	return MT_NONE;
}

/* UBX checksum */
void UBX_UART::_calcChecksum(uint8_t* CK, uint8_t* payload, uint16_t length)
{
	CK[0] = 0;
	CK[1] = 0;
	for (uint8_t i = 0; i < length; i++) {
		CK[0] += payload[i];
		CK[1] += CK[0];
	}
}
/* GPS time of week of nav solution, ms */
uint32_t UBX_UART::getTow_ms()
{
	return _NavPvtPacket.iTOW;
}

/* UTC year */
uint16_t UBX_UART::getYear()
{
	return _NavPvtPacket.year;
}

/* UTC month */
uint8_t UBX_UART::getMonth()
{
	return _NavPvtPacket.month;
}

/* UTC day */
uint8_t UBX_UART::getDay()
{
	return _NavPvtPacket.day;
}

/* UTC hour */
uint8_t UBX_UART::getHour()
{
	return _NavPvtPacket.hour;
}

/* UTC minute */
uint8_t UBX_UART::getMin()
{
	return _NavPvtPacket.min;
}

/* UTC second */
uint8_t UBX_UART::getSec()
{
	return _NavPvtPacket.sec;
}

/* UTC fraction of a second, ns */
int32_t UBX_UART::getNanoSec()
{
	return _NavPvtPacket.nano;
}

/* number of satellites used in nav solution */
uint8_t UBX_UART::getNumSatellites()
{
	return _NavPvtPacket.numSV;
}

/* longitude, deg */
double UBX_UART::getLongitude_deg()
{
	return (double)_NavPvtPacket.lon * 1e-7;
}

/* latitude, deg */
double UBX_UART::getLatitude_deg()
{
	return (double)_NavPvtPacket.lat * 1e-7;
}

/* height above the ellipsoid, ft */
double UBX_UART::getEllipsoidHeight_ft()
{
	return (double)_NavPvtPacket.height * 1e-3 * _m2ft;
}

/* height above mean sea level, ft */
double UBX_UART::getMSLHeight_ft()
{
	return (double)_NavPvtPacket.hMSL * 1e-3 * _m2ft;
}

/* horizontal accuracy estimate, ft */
double UBX_UART::getHorizontalAccuracy_ft()
{
	return (double)_NavPvtPacket.hAcc * 1e-3 * _m2ft;
}

/* vertical accuracy estimate, ft */
double UBX_UART::getVerticalAccuracy_ft()
{
	return (double)_NavPvtPacket.vAcc * 1e-3 * _m2ft;
}

/* NED north velocity, ft/s */
double UBX_UART::getNorthVelocity_fps()
{
	return (double)_NavPvtPacket.velN * 1e-3 * _m2ft;
}

/* NED east velocity, ft/s */
double UBX_UART::getEastVelocity_fps()
{
	return (double)_NavPvtPacket.velE * 1e-3 * _m2ft;
}

/* NED down velocity ft/s */
double UBX_UART::getDownVelocity_fps()
{
	return (double)_NavPvtPacket.velD * 1e-3 * _m2ft;
}

/* 2D ground speed, ft/s */
double UBX_UART::getGroundSpeed_fps()
{
	return (double)_NavPvtPacket.gSpeed * 1e-3 * _m2ft;
}

/* speed accuracy estimate, ft/s */
double UBX_UART::getSpeedAccuracy_fps()
{
	return (double)_NavPvtPacket.sAcc * 1e-3 * _m2ft;
}

/* 2D heading of motion, deg */
double UBX_UART::getMotionHeading_deg()
{
	return (double)_NavPvtPacket.headMot * 1e-5;
}

/* 2D vehicle heading, deg */
double UBX_UART::getVehicleHeading_deg()
{
	return (double)_NavPvtPacket.headVeh * 1e-5;
}

/* heading accuracy estimate, deg */
double UBX_UART::getHeadingAccuracy_deg()
{
	return (double)_NavPvtPacket.headAcc * 1e-5;
}

/* magnetic declination, deg */
float UBX_UART::getMagneticDeclination_deg()
{
	return (float)_NavPvtPacket.magDec * 1e-2;
}

/* magnetic declination accuracy estimate, deg */
float UBX_UART::getMagneticDeclinationAccuracy_deg()
{
	return (float)_NavPvtPacket.magAcc * 1e-2;
}

/* longitude, rad */
double UBX_UART::getLongitude_rad()
{
	return (double)_NavPvtPacket.lon * 1e-7 * _deg2rad;
}

/* latitude, rad */
double UBX_UART::getLatitude_rad()
{
	return (double)_NavPvtPacket.lat * 1e-7 * _deg2rad;
}

/* height above the ellipsoid, m */
double UBX_UART::getEllipsoidHeight_m()
{
	return (double)_NavPvtPacket.height * 1e-3;
}

/* height above mean sea level, m */
double UBX_UART::getMSLHeight_m()
{
	return (double)_NavPvtPacket.hMSL * 1e-3;
}

/* horizontal accuracy estimate, m */
double UBX_UART::getHorizontalAccuracy_m()
{
	return (double)_NavPvtPacket.hAcc * 1e-3;
}

/* vertical accuracy estimate, m */
double UBX_UART::getVerticalAccuracy_m()
{
	return (double)_NavPvtPacket.vAcc * 1e-3;
}

/* NED north velocity, m/s */
double UBX_UART::getNorthVelocity_ms()
{
	return (double)_NavPvtPacket.velN * 1e-3;
}

/* NED east velocity, m/s */
double UBX_UART::getEastVelocity_ms()
{
	return (double)_NavPvtPacket.velE * 1e-3;
}

/* NED down velocity, m/s */
double UBX_UART::getDownVelocity_ms()
{
	return (double)_NavPvtPacket.velD * 1e-3;
}

/* 2D ground speed, m/s */
double UBX_UART::getGroundSpeed_ms()
{
	return (double)_NavPvtPacket.gSpeed * 1e-3;
}

/* speed accuracy estimate, m/s */
double UBX_UART::getSpeedAccuracy_ms()
{
	return (double)_NavPvtPacket.sAcc * 1e-3;
}

/* 2D heading of motion, rad */
double UBX_UART::getMotionHeading_rad()
{
	return (double)_NavPvtPacket.headMot * 1e-5 * _deg2rad;
}

/* 2D vehicle heading, rad */
double UBX_UART::getVehicleHeading_rad()
{
	return (double)_NavPvtPacket.headVeh * 1e-5 * _deg2rad;
}

/* heading accuracy estimate, rad */
double UBX_UART::getHeadingAccuracy_rad()
{
	return (double)_NavPvtPacket.headAcc * 1e-5 * _deg2rad;
}

/* magnetic declination, rad */
float UBX_UART::getMagneticDeclination_rad()
{
	return (float)_NavPvtPacket.magDec * 1e-2 * _deg2rad;
}

/* magnetic declination accuracy estimate, rad */
float UBX_UART::getMagneticDeclinationAccuracy_rad()
{
	return (float)_NavPvtPacket.magAcc * 1e-2 * _deg2rad;
}

/* position dilution of precision */
float UBX_UART::getpDOP()
{
	return (float)_NavPvtPacket.pDOP * 1e-2;
}

/* fix type */
enum UBX_UART::FixType UBX_UART::getFixType()
{
	return (FixType)_NavPvtPacket.fixType;
}

/* power save mode */
enum UBX_UART::PowerSaveMode UBX_UART::getPowerSaveMode()
{
	return (PowerSaveMode)((_NavPvtPacket.flags >> 2) & 0x07);
}

/* carrier phase status */
enum UBX_UART::CarrierPhaseStatus UBX_UART::getCarrierPhaseStatus()
{
	return (CarrierPhaseStatus)((_NavPvtPacket.flags >> 6) & 0x03);
}

/* valid fix, within DOP and accuracy masks */
bool UBX_UART::isGnssFixOk()
{
	return _NavPvtPacket.flags & 0x01;
}

/* differential corrections were applied */
bool UBX_UART::isDiffCorrApplied()
{
	return _NavPvtPacket.flags & 0x02;
}

/* heading of vehicle is valid */
bool UBX_UART::isHeadingValid()
{
	return _NavPvtPacket.flags & 0x20;
}

/* UTC date validity could be confirmed */
bool UBX_UART::isConfirmedDate()
{
	return _NavPvtPacket.flags & 0x40;
}

/* UTC time validity could be confirmed */
bool UBX_UART::isConfirmedTime()
{
	return _NavPvtPacket.flags & 0x80;
}

/* info about UTC date and time validity confirmation is available */
bool UBX_UART::isTimeDateConfirmationAvail()
{
	return _NavPvtPacket.flags2 & 0x20;
}

/* valid UTC date */
bool UBX_UART::isValidDate()
{
	return _NavPvtPacket.valid & 0x01;
}

/* valid UTC time */
bool UBX_UART::isValidTime()
{
	return _NavPvtPacket.valid & 0x02;
}

/* UTC time of day has been fully resolved, no seconds uncertainty */
bool UBX_UART::isTimeFullyResolved()
{
	return _NavPvtPacket.valid & 0x04;
}

/* valid magnetic declination estimate */
bool UBX_UART::isMagneticDeclinationValid()
{
	return _NavPvtPacket.valid & 0x08;
}
