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
#include "UBX_I2C.h"
UBX_I2C::UBX_I2C(void)
{
  // Constructor
}

//Initialize the Serial port
void UBX_I2C::begin(TwoWire &wirePort)
{
	_bus = &wirePort; //Grab which port the user wants us to use
	//We expect caller to begin their I2C port, with the speed of their choice external to the library
	//But if they forget, we start the hardware here.
	_bus->begin();
}

//Returns true if I2C device ack's
bool UBX_I2C::isConnected()
{
  _bus->beginTransmission((uint8_t)_gpsI2Caddress);
  if (_bus->endTransmission() != 0)
    return (false); //Sensor did not ACK
  return (true);
}

//Polls I2C for data, passing any new bytes to process()
//Times out after given amount of time
uint8_t UBX_I2C::readSensor()
{
  if (millis() - lastCheck >= I2C_POLLING_WAIT_MS)
  {
	//Get the number of bytes available from the module
    uint16_t bytesAvailable = 0;
    _bus->beginTransmission(_gpsI2Caddress);
    _bus->write(0xFD); //0xFD (MSB) and 0xFE (LSB) are the registers that contain number of bytes available
    if (_bus->endTransmission(false) != 0) //Send a restart command. Do not release bus.
      return (MT_NONE); //Sensor did not ACK

    _bus->requestFrom((uint8_t)_gpsI2Caddress, (uint8_t)2);
    if (_bus->available())
    {
      uint8_t msb = _bus->read();
      uint8_t lsb = _bus->read();
      bytesAvailable = (uint16_t)msb << 8 | lsb;
    }

    if (bytesAvailable == 0)
      lastCheck = millis(); //Put off checking to avoid I2C bus traffic

    while (bytesAvailable)
    {
      _bus->beginTransmission(_gpsI2Caddress);
      _bus->write(0xFF); //0xFF is the register to read general UBX data from
      if (_bus->endTransmission(false) != 0) //Send a restart command. Do not release bus.
        return (MT_NONE); //Sensor did not ACK

      //Limit to 32 bytes or whatever the buffer limit is for given platform
      uint16_t bytesToRead = bytesAvailable;
      if (bytesToRead > I2C_BUFFER_LENGTH) bytesToRead = I2C_BUFFER_LENGTH;

      _bus->requestFrom((uint8_t)_gpsI2Caddress, (uint8_t)bytesToRead);
      if (_bus->available())
      {
        for (uint16_t x = 0 ; x < bytesToRead ; x++)
        {
          _currentMsgType = _parse(_bus->read());
		  if (_currentMsgType) return (_currentMsgType);
        }
      }
      else
        return (MT_NONE); //Sensor did not respond

      bytesAvailable -= bytesToRead;
    }
  } //end timed read

  return (MT_NONE);

} //end checkUbloxI2C()

/* parse the UBX data */
uint8_t UBX_I2C::_parse(uint8_t _byte)
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
					_parserState = 0;
					switch (_tempPacket.msg_class_id) {
					case 0x0701: //NAV_PVT
						memcpy(&_NavPvtPacket,&_tempPacket,sizeof(_NavPvtPacket));
						return MT_NAV_PVT;
					case 0x1510: //ESF_INS
						memcpy(&_EsfInsPacket,&_tempPacket,sizeof(_EsfInsPacket));
						return MT_ESF_INS;
					case 0x0210: //ESF_MEA
						memcpy(&_EsfMeaPacket,&_tempPacket,sizeof(_EsfMeaPacket));
						return MT_ESF_MEA;
					case 0x0310: //ESF_RAW
						memcpy(&_EsfRawPacket,&_tempPacket,sizeof(_EsfRawPacket));
						return MT_ESF_RAW;
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
void UBX_I2C::_calcChecksum(uint8_t* CK, uint8_t* payload, uint16_t length)
{
	CK[0] = 0;
	CK[1] = 0;
	for (uint8_t i = 0; i < length; i++) {
		CK[0] += payload[i];
		CK[1] += CK[0];
	}
}
bool UBX_I2C::sendCfg()
{
// B5 62 06 00 14 00 00 00 00 00 84 00 00 00 00 00 00 00 01 00 01 00 00 00 00 00 A0 96 //CFG_PRT
// B5 62 06 01 08 00 01 07 01 00 00 00 00 00 18 E2 //NAV_PVT
// B5 62 06 01 08 00 10 15 01 00 00 00 00 00 35 BC //ESF_INS 
// B5 62 06 01 08 00 10 02 01 00 00 00 00 00 22 37 //ESF_MEAS	
// B5 62 06 01 08 00 10 03 01 00 00 00 00 00 23 3E //ESF_RAW
// B5 62 06 01 08 00 10 10 01 00 00 00 00 00 30 99 //ESF_STATUS

const uint8_t msg_cfg_prt[] = {0x06,0x00,0x14,0x00,0x00,0x00,0x00,0x00,0x84,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x01,0x00,0x00,0x00,0x00,0x00};
const uint8_t msg_nav_pvt[] = {0x06,0x01,0x08,0x00,0x01,0x07,0x01,0x00,0x00,0x00,0x00,0x00};
const uint8_t msg_esf_ins[] = {0x06,0x01,0x08,0x00,0x10,0x15,0x01,0x00,0x00,0x00,0x00,0x00};
const uint8_t msg_esf_mea[] = {0x06,0x01,0x08,0x00,0x10,0x02,0x01,0x00,0x00,0x00,0x00,0x00};
const uint8_t msg_esf_raw[] = {0x06,0x01,0x08,0x00,0x10,0x03,0x01,0x00,0x00,0x00,0x00,0x00};
const uint8_t msg_esf_sta[] = {0x06,0x01,0x08,0x00,0x10,0x10,0x01,0x00,0x00,0x00,0x00,0x00};

	for (uint8_t i=0; i < sizeof(msg_cfg_prt); i++)
      *((uint8_t *) &_tempPacket + i) = msg_cfg_prt[i];
	if (not _sendCommand()) return (false);
	for (uint8_t i=0; i < sizeof(msg_nav_pvt); i++)
      *((uint8_t *) &_tempPacket + i) = msg_nav_pvt[i];
	if (not _sendCommand()) return (false);
	for (uint8_t i=0; i < sizeof(msg_esf_ins); i++)
      *((uint8_t *) &_tempPacket + i) = msg_esf_ins[i];
	if (not _sendCommand()) return (false);
	for (uint8_t i=0; i < sizeof(msg_esf_mea); i++)
      *((uint8_t *) &_tempPacket + i) = msg_esf_mea[i];
	if (not _sendCommand()) return (false);
	for (uint8_t i=0; i < sizeof(msg_esf_raw); i++)
      *((uint8_t *) &_tempPacket + i) = msg_esf_raw[i];
	if (not _sendCommand()) return (false);
	for (uint8_t i=0; i < sizeof(msg_esf_sta); i++)
      *((uint8_t *) &_tempPacket + i) = msg_esf_sta[i];
	if (not _sendCommand()) return (false);
return (true);
}
//Given a packet and payload, send everything including CRC bytes
bool UBX_I2C::_sendCommand()
{
	//Point at 0xFF data register
	_bus->beginTransmission((uint8_t)_gpsI2Caddress); //There is no register to write to, we just begin writing data bytes
	_bus->write(0xFF);
	if (_bus->endTransmission() != 0) //Don't release bus
		return (false); //Sensor did not ACK

	//Write payload. Limit the sends into 32 byte chunks
	//This code based on ublox: https://forum.u-blox.com/index.php/20528/how-to-use-i2c-to-get-the-nmea-frames
	uint16_t bytesToSend = _tempPacket.msg_length+4;

	//Write header bytes
	_bus->beginTransmission((uint8_t)_gpsI2Caddress); //There is no register to write to, we just begin writing data bytes
	_bus->write(_ubxHeader[0]);
	_bus->write(_ubxHeader[1]);
	if (_bus->endTransmission(false) != 0) //Don't release bus
		return (false); //Sensor did not ACK

	//"The number of data bytes must be at least 2 to properly distinguish
	//from the write access to set the address counter in random read accesses."
	uint16_t startSpot = 0;
	while (bytesToSend > 1)
	{
		uint8_t len = bytesToSend;
		if (len > I2C_BUFFER_LENGTH) len = I2C_BUFFER_LENGTH;

		_bus->beginTransmission((uint8_t)_gpsI2Caddress);

		for (uint16_t x = 0 ; x < len ; x++)
		{
		_bus->write(*((uint8_t *) &_tempPacket + startSpot + x)); //Write a portion of the payload to the bus
		}

    if (_bus->endTransmission(false) != 0) //Don't release bus
		return (false); //Sensor did not ACK

    //*outgoingUBX.payload += len; //Move the pointer forward
	startSpot += len; //Move the pointer forward
	bytesToSend -= len;
	}
	_calcChecksum(_checksum,((uint8_t *) &_tempPacket),(_tempPacket.msg_length+4));
	//Write checksum
	_bus->beginTransmission((uint8_t)_gpsI2Caddress);
	if (bytesToSend == 1) _bus->write(*((uint8_t *) &_tempPacket + startSpot)); // combine last byte with the checksum
	_bus->write(_checksum[0]);
	_bus->write(_checksum[1]);
	if (_bus->endTransmission() != 0)
	return (false); //Sensor did not ACK

	return (true);
}
/* GPS time of week of nav solution, ms */
uint32_t UBX_I2C::getTow_ms()
{
	return _NavPvtPacket.iTOW;
}

/* UTC year */
uint16_t UBX_I2C::getYear()
{
	return _NavPvtPacket.year;
}

/* UTC month */
uint8_t UBX_I2C::getMonth()
{
	return _NavPvtPacket.month;
}

/* UTC day */
uint8_t UBX_I2C::getDay()
{
	return _NavPvtPacket.day;
}

/* UTC hour */
uint8_t UBX_I2C::getHour()
{
	return _NavPvtPacket.hour;
}

/* UTC minute */
uint8_t UBX_I2C::getMin()
{
	return _NavPvtPacket.min;
}

/* UTC second */
uint8_t UBX_I2C::getSec()
{
	return _NavPvtPacket.sec;
}

/* UTC fraction of a second, ns */
int32_t UBX_I2C::getNanoSec()
{
	return _NavPvtPacket.nano;
}

/* number of satellites used in nav solution */
uint8_t UBX_I2C::getNumSatellites()
{
	return _NavPvtPacket.numSV;
}

/* longitude, deg */
double UBX_I2C::getLongitude_deg()
{
	return (double)_NavPvtPacket.lon * 1e-7;
}

/* latitude, deg */
double UBX_I2C::getLatitude_deg()
{
	return (double)_NavPvtPacket.lat * 1e-7;
}

/* 2D heading of motion, deg */
double UBX_I2C::getMotionHeading_deg()
{
	return (double)_NavPvtPacket.headMot * 1e-5;
}

/* 2D vehicle heading, deg */
double UBX_I2C::getVehicleHeading_deg()
{
	return (double)_NavPvtPacket.headVeh * 1e-5;
}

/* heading accuracy estimate, deg */
double UBX_I2C::getHeadingAccuracy_deg()
{
	return (double)_NavPvtPacket.headAcc * 1e-5;
}

/* magnetic declination, deg */
float UBX_I2C::getMagneticDeclination_deg()
{
	return (float)_NavPvtPacket.magDec * 1e-2;
}

/* magnetic declination accuracy estimate, deg */
float UBX_I2C::getMagneticDeclinationAccuracy_deg()
{
	return (float)_NavPvtPacket.magAcc * 1e-2;
}

/* longitude, rad */
double UBX_I2C::getLongitude_rad()
{
	return (double)_NavPvtPacket.lon * 1e-7 * _deg2rad;
}

/* latitude, rad */
double UBX_I2C::getLatitude_rad()
{
	return (double)_NavPvtPacket.lat * 1e-7 * _deg2rad;
}

/* height above the ellipsoid, m */
double UBX_I2C::getEllipsoidHeight_m()
{
	return (double)_NavPvtPacket.height * 1e-3;
}

/* height above mean sea level, m */
double UBX_I2C::getMSLHeight_m()
{
	return (double)_NavPvtPacket.hMSL * 1e-3;
}

/* horizontal accuracy estimate, m */
double UBX_I2C::getHorizontalAccuracy_m()
{
	return (double)_NavPvtPacket.hAcc * 1e-3;
}

/* vertical accuracy estimate, m */
double UBX_I2C::getVerticalAccuracy_m()
{
	return (double)_NavPvtPacket.vAcc * 1e-3;
}

/* NED north velocity, m/s */
double UBX_I2C::getNorthVelocity_ms()
{
	return (double)_NavPvtPacket.velN * 1e-3;
}

/* NED east velocity, m/s */
double UBX_I2C::getEastVelocity_ms()
{
	return (double)_NavPvtPacket.velE * 1e-3;
}

/* NED down velocity, m/s */
double UBX_I2C::getDownVelocity_ms()
{
	return (double)_NavPvtPacket.velD * 1e-3;
}

/* 2D ground speed, m/s */
double UBX_I2C::getGroundSpeed_ms()
{
	return (double)_NavPvtPacket.gSpeed * 1e-3;
}

/* speed accuracy estimate, m/s */
double UBX_I2C::getSpeedAccuracy_ms()
{
	return (double)_NavPvtPacket.sAcc * 1e-3;
}

/* 2D heading of motion, rad */
double UBX_I2C::getMotionHeading_rad()
{
	return (double)_NavPvtPacket.headMot * 1e-5 * _deg2rad;
}

/* 2D vehicle heading, rad */
double UBX_I2C::getVehicleHeading_rad()
{
	return (double)_NavPvtPacket.headVeh * 1e-5 * _deg2rad;
}

/* heading accuracy estimate, rad */
double UBX_I2C::getHeadingAccuracy_rad()
{
	return (double)_NavPvtPacket.headAcc * 1e-5 * _deg2rad;
}

/* magnetic declination, rad */
float UBX_I2C::getMagneticDeclination_rad()
{
	return (float)_NavPvtPacket.magDec * 1e-2 * _deg2rad;
}

/* magnetic declination accuracy estimate, rad */
float UBX_I2C::getMagneticDeclinationAccuracy_rad()
{
	return (float)_NavPvtPacket.magAcc * 1e-2 * _deg2rad;
}

/* position dilution of precision */
float UBX_I2C::getpDOP()
{
	return (float)_NavPvtPacket.pDOP * 1e-2;
}

/* fix type */
enum UBX_I2C::FixType UBX_I2C::getFixType()
{
	return (FixType)_NavPvtPacket.fixType;
}

/* power save mode */
enum UBX_I2C::PowerSaveMode UBX_I2C::getPowerSaveMode()
{
	return (PowerSaveMode)((_NavPvtPacket.flags >> 2) & 0x07);
}

/* carrier phase status */
enum UBX_I2C::CarrierPhaseStatus UBX_I2C::getCarrierPhaseStatus()
{
	return (CarrierPhaseStatus)((_NavPvtPacket.flags >> 6) & 0x03);
}

/* valid fix, within DOP and accuracy masks */
bool UBX_I2C::isGnssFixOk()
{
	return _NavPvtPacket.flags & 0x01;
}

/* differential corrections were applied */
bool UBX_I2C::isDiffCorrApplied()
{
	return _NavPvtPacket.flags & 0x02;
}

/* heading of vehicle is valid */
bool UBX_I2C::isHeadingValid()
{
	return _NavPvtPacket.flags & 0x20;
}

/* UTC date validity could be confirmed */
bool UBX_I2C::isConfirmedDate()
{
	return _NavPvtPacket.flags & 0x40;
}

/* UTC time validity could be confirmed */
bool UBX_I2C::isConfirmedTime()
{
	return _NavPvtPacket.flags & 0x80;
}

/* info about UTC date and time validity confirmation is available */
bool UBX_I2C::isTimeDateConfirmationAvail()
{
	return _NavPvtPacket.flags2 & 0x20;
}

/* valid UTC date */
bool UBX_I2C::isValidDate()
{
	return _NavPvtPacket.valid & 0x01;
}

/* valid UTC time */
bool UBX_I2C::isValidTime()
{
	return _NavPvtPacket.valid & 0x02;
}

/* UTC time of day has been fully resolved, no seconds uncertainty */
bool UBX_I2C::isTimeFullyResolved()
{
	return _NavPvtPacket.valid & 0x04;
}

/* valid magnetic declination estimate */
bool UBX_I2C::isMagneticDeclinationValid()
{
	return _NavPvtPacket.valid & 0x08;
}


/////////////////////////////////////////////////////////////////////////////
/* ESF Status */
uint8_t UBX_I2C::getFusionMode()
{
	return _EsfStaPacket.fusionMode;
}
uint8_t UBX_I2C::getNumSens()
{
	return _EsfStaPacket.numSens;
}
/* ESF Ins */
uint32_t UBX_I2C::getBitfield0()
{
	return _EsfInsPacket.bitfield0;
}
double UBX_I2C::getxAngRate()
{
	return (double)_EsfInsPacket.xAngRate * 1e-3;
}
double UBX_I2C::getyAngRate()
{
	return (double)_EsfInsPacket.yAngRate * 1e-3;
}
double UBX_I2C::getzAngRate()
{
	return (double)_EsfInsPacket.zAngRate * 1e-3;
}
double UBX_I2C::getxAccel()
{
	return (double)_EsfInsPacket.xAccel * 1e-2;
}
double UBX_I2C::getyAccel()
{
	return (double)_EsfInsPacket.yAccel * 1e-2;
}
double UBX_I2C::getzAccel()
{
	return (double)_EsfInsPacket.zAccel * 1e-2;
}
/* ESF Raw */
uint32_t UBX_I2C::getRawData0()
{
	return _EsfRawPacket.data0;
}
uint32_t UBX_I2C::getRawData1()
{
	return _EsfRawPacket.data1;
}uint32_t UBX_I2C::getRawData2()
{
	return _EsfRawPacket.data2;
}uint32_t UBX_I2C::getRawData3()
{
	return _EsfRawPacket.data3;
}uint32_t UBX_I2C::getRawData4()
{
	return _EsfRawPacket.data4;
}uint32_t UBX_I2C::getRawData5()
{
	return _EsfRawPacket.data5;
}uint32_t UBX_I2C::getRawData6()
{
	return _EsfRawPacket.data6;
}
uint32_t UBX_I2C::getRawsTtag0()
{
	return _EsfRawPacket.sTtag0;
}
uint32_t UBX_I2C::getRawsTtag1()
{
	return _EsfRawPacket.sTtag1;
}uint32_t UBX_I2C::getRawsTtag2()
{
	return _EsfRawPacket.sTtag2;
}uint32_t UBX_I2C::getRawsTtag3()
{
	return _EsfRawPacket.sTtag3;
}uint32_t UBX_I2C::getRawsTtag4()
{
	return _EsfRawPacket.sTtag4;
}uint32_t UBX_I2C::getRawsTtag5()
{
	return _EsfRawPacket.sTtag5;
}uint32_t UBX_I2C::getRawsTtag6()
{
	return _EsfRawPacket.sTtag6;
}
/* ESF Mea */
uint32_t UBX_I2C::getMeaData0()
{
	return _EsfMeaPacket.data0;
}
uint32_t UBX_I2C::getMeaData1()
{
	return _EsfMeaPacket.data1;
}uint32_t UBX_I2C::getMeaData2()
{
	return _EsfMeaPacket.data2;
}uint32_t UBX_I2C::getMeaData3()
{
	return _EsfMeaPacket.data3;
}uint32_t UBX_I2C::getMeaData4()
{
	return _EsfMeaPacket.data4;
}uint32_t UBX_I2C::getMeaData5()
{
	return _EsfMeaPacket.data5;
}uint32_t UBX_I2C::getMeaData6()
{
	return _EsfMeaPacket.data6;
}
uint32_t UBX_I2C::getMeaCalibTtag0()
{
	return _EsfMeaPacket.calibTtag0;
}
uint32_t UBX_I2C::getMeaCalibTtag1()
{
	return _EsfMeaPacket.calibTtag1;
}uint32_t UBX_I2C::getMeaCalibTtag2()
{
	return _EsfMeaPacket.calibTtag2;
}uint32_t UBX_I2C::getMeaCalibTtag3()
{
	return _EsfMeaPacket.calibTtag3;
}uint32_t UBX_I2C::getMeaCalibTtag4()
{
	return _EsfMeaPacket.calibTtag4;
}uint32_t UBX_I2C::getMeaCalibTtag5()
{
	return _EsfMeaPacket.calibTtag5;
}uint32_t UBX_I2C::getMeaCalibTtag6()
{
	return _EsfMeaPacket.calibTtag6;
}