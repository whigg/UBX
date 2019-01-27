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
UBX_UART::UBX_UART(HardwareSerial& bus)
//UBX_UART::UBX_UART(SoftwareSerial *bus)
{
  _bus = &bus;  // for Hardserial
//  _bus = bus; // for Softserial
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
	for(int i=0; i<6; i++){
		while (_bus->available()){
		_currentMsgType=_parse(_bus->read());
		if (_currentMsgType) return (_currentMsgType);
		}
		delay(10);
	}	
    return (MT_NONE);
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
			if ((_parserState - 2) < (_tempPacket._UbxMsgPacket.msg_length+4)) {
				*((uint8_t *) &_tempPacket + _parserState - 2) = _byte;
			}
			_parserState++;
			// compute checksum
			if ((_parserState - 2) == (_tempPacket._UbxMsgPacket.msg_length+4)) {
				_calcChecksum(_checksum,((uint8_t *) &_tempPacket),(_tempPacket._UbxMsgPacket.msg_length+4));
			} else if ((_parserState - 2) == (_tempPacket._UbxMsgPacket.msg_length+5)) {
				if (_byte != _checksum[0]) {
					_parserState = 0;
				}
			} else if ((_parserState - 2) == (_tempPacket._UbxMsgPacket.msg_length+6)) {
				_parserState = 0;
				if (_byte == _checksum[1]) {
					_parserState = 0;
					memcpy(&_validPacket,&_tempPacket,sizeof(_validPacket));
					switch (_tempPacket._UbxMsgPacket.msg_class_id) {
					case 0x0501: //NAV_ATT
						return MT_NAV_ATT;
					case 0x0701: //NAV_PVT
						return MT_NAV_PVT;
					case 0x1101: //NAV_VEL
						return MT_NAV_VEL;
					case 0x1510: //ESF_INS
						return MT_ESF_INS;
					case 0x1010: //ESF_STATUS
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
bool UBX_UART::sendCfg()
{
const uint8_t msg_cfg_esfalg[] = {0x06,0x56,0x0C,0x00,0x00,0x89,0xA8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; //SET esfalg
const uint8_t msg_cfg_nav5[] = {0x06,0x24,0x24,0x00,
0xFF,0xFF,0x0A,0x03,0x00,0x00,0x00,0x00,
0x10,0x27,0x00,0x00,0x0A,0x00,0xFA,0x00,
0xFA,0x00,0x64,0x00,0x5E,0x01,0x00,0x3C,
0x00,0x00,0x10,0x27,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00}; // Bike dynamic
//const uint8_t msg_sos_bck[] = {0x09,0x14,0x04,0x00,0x00,0x00,0x00,0x00};//backup;
//const uint8_t msg_sos_clr[] = {0x09,0x14,0x04,0x00,0x01,0x00,0x00,0x00};//clear
//const uint8_t msg_esf_rstalg[] = {0x10,0x13,0x00,0x00}; // reset esfalg
//const uint8_t msg_cfg_rst[] = {0x06,0x04,0x04,0x00,0x00,0x00,0x08,0x00}; //STOP GNSS
const uint8_t msg_cfg_def[] = {0x06,0x09,0x0D,0x00,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0x03}; // restore def cfg
const uint8_t msg_cfg_uart[] = {0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x80,0x25,0x00,0x00,0x01,0x00,0x01,0x00,0x00,0x00,0x00,0x00}; //9600
//const uint8_t msg_cfg_uart[] = {0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0x4B,0x00,0x00,0x01,0x00,0x01,0x00,0x00,0x00,0x00,0x00}; //19200
//const uint8_t msg_cfg_uart[] = {0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0x96,0x00,0x00,0x01,0x00,0x01,0x00,0x00,0x00,0x00,0x00}; //38400
//const uint8_t msg_cfg_uart[] = {0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0xE1,0x00,0x00,0x01,0x00,0x01,0x00,0x00,0x00,0x00,0x00}; //57600
//const uint8_t msg_cfg_uart[] = {0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0xC2,0x01,0x00,0x01,0x00,0x01,0x00,0x00,0x00,0x00,0x00}; //115200
//const uint8_t msg_cfg_i2c[] = {0x06,0x00,0x14,0x00,0x00,0x00,0x00,0x00,0x84,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x01,0x00,0x00,0x00,0x00,0x00};
const uint8_t msg_nav_att[] = {0x06,0x01,0x08,0x00,0x01,0x05,0x00,0x01,0x00,0x00,0x00,0x00};
const uint8_t msg_nav_pvt[] = {0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x01,0x00,0x00,0x00,0x00};
const uint8_t msg_nav_vel[] = {0x06,0x01,0x08,0x00,0x01,0x11,0x00,0x01,0x00,0x00,0x00,0x00};
const uint8_t msg_esf_ins[] = {0x06,0x01,0x08,0x00,0x10,0x15,0x00,0x01,0x00,0x00,0x00,0x00};
const uint8_t msg_esf_sta[] = {0x06,0x01,0x08,0x00,0x10,0x10,0x00,0x01,0x00,0x00,0x00,0x00};
const uint8_t msg_cfg_save[] = {0x06,0x09,0x0D,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x13};

	for (uint8_t i=0; i < sizeof(msg_cfg_def); i++)
      *((uint8_t *) &_tempPacket + i) = msg_cfg_def[i];
	if (not _sendCommand()) return (false);
	delay(100);
	for (uint8_t i=0; i < sizeof(msg_cfg_uart); i++)
      *((uint8_t *) &_tempPacket + i) = msg_cfg_uart[i];
	if (not _sendCommand()) return (false);
	delay(10);
//	_bus->end();
//	delay(10);
//	_bus->begin(115200);
	for (uint8_t i=0; i < sizeof(msg_cfg_esfalg); i++)
      *((uint8_t *) &_tempPacket + i) = msg_cfg_esfalg[i];
	if (not _sendCommand()) return (false);
	delay(10);
	for (uint8_t i=0; i < sizeof(msg_cfg_nav5); i++)
      *((uint8_t *) &_tempPacket + i) = msg_cfg_nav5[i];
	if (not _sendCommand()) return (false);
	delay(10);
	for (uint8_t i=0; i < sizeof(msg_nav_pvt); i++)
      *((uint8_t *) &_tempPacket + i) = msg_nav_pvt[i];
	if (not _sendCommand()) return (false);
	delay(10);
	for (uint8_t i=0; i < sizeof(msg_nav_vel); i++)
      *((uint8_t *) &_tempPacket + i) = msg_nav_vel[i];
	if (not _sendCommand()) return (false);
	delay(10);
/*	for (uint8_t i=0; i < sizeof(msg_nav_att); i++)
      *((uint8_t *) &_tempPacket + i) = msg_nav_att[i];
	if (not _sendCommand()) return (false);
	delay(10);	for (uint8_t i=0; i < sizeof(msg_esf_ins); i++)
      *((uint8_t *) &_tempPacket + i) = msg_esf_ins[i];
	if (not _sendCommand()) return (false);
	delay(10);
	for (uint8_t i=0; i < sizeof(msg_esf_sta); i++)
      *((uint8_t *) &_tempPacket + i) = msg_esf_sta[i];
	if (not _sendCommand()) return (false);
	delay(10); */
//	for (uint8_t i=0; i < sizeof(msg_cfg_save); i++)
//      *((uint8_t *) &_tempPacket + i) = msg_cfg_save[i];
//	if (not _sendCommand()) return (false);
return (true);
}
//Given a packet and payload, send everything including CRC bytes
bool UBX_UART::_sendCommand()
{
    //Write header bytes
    _bus->write(_ubxHeader[0]);
    _bus->write(_ubxHeader[1]);
	//Write msg
    for (uint16_t x = 0 ; x < _tempPacket._UbxMsgPacket.msg_length+4 ; x++)
	{
      _bus->write(*((uint8_t *) &_tempPacket + x)); //Write a portion of the payload to the bus
	}
	_calcChecksum(_checksum,((uint8_t *) &_tempPacket),(_tempPacket._UbxMsgPacket.msg_length+4));
	//Write checksum
	_bus->write(_checksum[0]);
	_bus->write(_checksum[1]);
	return (true);
}
/* GPS time of week of nav solution, ms */
uint32_t UBX_UART::getTow_ms()
{
	return _validPacket._NavPvtPacket.iTOW;
}

/* UTC year */
uint16_t UBX_UART::getYear()
{
	return _validPacket._NavPvtPacket.year;
}

/* UTC month */
uint8_t UBX_UART::getMonth()
{
	return _validPacket._NavPvtPacket.month;
}

/* UTC day */
uint8_t UBX_UART::getDay()
{
	return _validPacket._NavPvtPacket.day;
}

/* UTC hour */
uint8_t UBX_UART::getHour()
{
	return _validPacket._NavPvtPacket.hour;
}

/* UTC minute */
uint8_t UBX_UART::getMin()
{
	return _validPacket._NavPvtPacket.min;
}

/* UTC second */
uint8_t UBX_UART::getSec()
{
	return _validPacket._NavPvtPacket.sec;
}

/* UTC fraction of a second, ns */
int32_t UBX_UART::getNanoSec()
{
	return _validPacket._NavPvtPacket.nano;
}

/* number of satellites used in nav solution */
uint8_t UBX_UART::getNumSatellites()
{
	return _validPacket._NavPvtPacket.numSV;
}

/* longitude, deg */
double UBX_UART::getLongitude_deg()
{
	return (double)_validPacket._NavPvtPacket.lon * 1e-7;
}

/* latitude, deg */
double UBX_UART::getLatitude_deg()
{
	return (double)_validPacket._NavPvtPacket.lat * 1e-7;
}

/* 2D heading of motion, deg */
double UBX_UART::getMotionHeading_deg()
{
	return (double)_validPacket._NavPvtPacket.headMot * 1e-5;
}

/* 2D vehicle heading, deg */
double UBX_UART::getVehicleHeading_deg()
{
	return (double)_validPacket._NavPvtPacket.headVeh * 1e-5;
}

/* heading accuracy estimate, deg */
double UBX_UART::getHeadingAccuracy_deg()
{
	return (double)_validPacket._NavPvtPacket.headAcc * 1e-5;
}

/* magnetic declination, deg */
float UBX_UART::getMagneticDeclination_deg()
{
	return (float)_validPacket._NavPvtPacket.magDec * 1e-2;
}

/* magnetic declination accuracy estimate, deg */
float UBX_UART::getMagneticDeclinationAccuracy_deg()
{
	return (float)_validPacket._NavPvtPacket.magAcc * 1e-2;
}

/* height above the ellipsoid, m */
double UBX_UART::getEllipsoidHeight_m()
{
	return (double)_validPacket._NavPvtPacket.height * 1e-3;
}

/* height above mean sea level, m */
double UBX_UART::getMSLHeight_m()
{
	return (double)_validPacket._NavPvtPacket.hMSL * 1e-3;
}

/* horizontal accuracy estimate, m */
double UBX_UART::getHorizontalAccuracy_m()
{
	return (double)_validPacket._NavPvtPacket.hAcc * 1e-3;
}

/* vertical accuracy estimate, m */
double UBX_UART::getVerticalAccuracy_m()
{
	return (double)_validPacket._NavPvtPacket.vAcc * 1e-3;
}

/* NED north velocity, m/s */
double UBX_UART::getNorthVelocity_ms()
{
	return (double)_validPacket._NavPvtPacket.velN * 1e-3;
}

/* NED east velocity, m/s */
double UBX_UART::getEastVelocity_ms()
{
	return (double)_validPacket._NavPvtPacket.velE * 1e-3;
}

/* NED down velocity, m/s */
double UBX_UART::getDownVelocity_ms()
{
	return (double)_validPacket._NavPvtPacket.velD * 1e-3;
}

/* 2D ground speed, m/s */
double UBX_UART::getGroundSpeed_ms()
{
	return (double)_validPacket._NavPvtPacket.gSpeed * 1e-3;
}

/* speed accuracy estimate, m/s */
double UBX_UART::getSpeedAccuracy_ms()
{
	return (double)_validPacket._NavPvtPacket.sAcc * 1e-3;
}

/* position dilution of precision */
float UBX_UART::getpDOP()
{
	return (float)_validPacket._NavPvtPacket.pDOP * 1e-2;
}

/* fix type */
enum UBX_UART::FixType UBX_UART::getFixType()
{
	return (FixType)_validPacket._NavPvtPacket.fixType;
}

/* power save mode */
enum UBX_UART::PowerSaveMode UBX_UART::getPowerSaveMode()
{
	return (PowerSaveMode)((_validPacket._NavPvtPacket.flags >> 2) & 0x07);
}

/* carrier phase status */
enum UBX_UART::CarrierPhaseStatus UBX_UART::getCarrierPhaseStatus()
{
	return (CarrierPhaseStatus)((_validPacket._NavPvtPacket.flags >> 6) & 0x03);
}

/* valid fix, within DOP and accuracy masks */
bool UBX_UART::isGnssFixOk()
{
	return _validPacket._NavPvtPacket.flags & 0x01;
}

/* differential corrections were applied */
bool UBX_UART::isDiffCorrApplied()
{
	return _validPacket._NavPvtPacket.flags & 0x02;
}

/* heading of vehicle is valid */
bool UBX_UART::isHeadingValid()
{
	return _validPacket._NavPvtPacket.flags & 0x20;
}

/* UTC date validity could be confirmed */
bool UBX_UART::isConfirmedDate()
{
	return _validPacket._NavPvtPacket.flags & 0x40;
}

/* UTC time validity could be confirmed */
bool UBX_UART::isConfirmedTime()
{
	return _validPacket._NavPvtPacket.flags & 0x80;
}

/* info about UTC date and time validity confirmation is available */
bool UBX_UART::isTimeDateConfirmationAvail()
{
	return _validPacket._NavPvtPacket.flags2 & 0x20;
}

/* valid UTC date */
bool UBX_UART::isValidDate()
{
	return _validPacket._NavPvtPacket.valid & 0x01;
}

/* valid UTC time */
bool UBX_UART::isValidTime()
{
	return _validPacket._NavPvtPacket.valid & 0x02;
}

/* UTC time of day has been fully resolved, no seconds uncertainty */
bool UBX_UART::isTimeFullyResolved()
{
	return _validPacket._NavPvtPacket.valid & 0x04;
}

/* valid magnetic declination estimate */
bool UBX_UART::isMagneticDeclinationValid()
{
	return _validPacket._NavPvtPacket.valid & 0x08;
}


/////////////////////////////////////////////////////////////////////////////
/* ESF Status */
uint8_t UBX_UART::getFusionMode()
{
	return _validPacket._EsfStaPacket.fusionMode;
}
uint8_t UBX_UART::getNumSens()
{
	return _validPacket._EsfStaPacket.numSens;
}
/* ESF Ins */
uint32_t UBX_UART::getBitfield0()
{
	return _validPacket._EsfInsPacket.bitfield0;
}
double UBX_UART::getxAngRate()
{
	return (double)_validPacket._EsfInsPacket.xAngRate * 1e-3;
}
double UBX_UART::getyAngRate()
{
	return (double)_validPacket._EsfInsPacket.yAngRate * 1e-3;
}
double UBX_UART::getzAngRate()
{
	return (double)_validPacket._EsfInsPacket.zAngRate * 1e-3;
}
double UBX_UART::getxAccel()
{
	return (double)_validPacket._EsfInsPacket.xAccel * 1e-2;
}
double UBX_UART::getyAccel()
{
	return (double)_validPacket._EsfInsPacket.yAccel * 1e-2;
}
double UBX_UART::getzAccel()
{
	return (double)_validPacket._EsfInsPacket.zAccel * 1e-2;
}
double UBX_UART::getRoll()
{
	return (double)_validPacket._NavAttPacket.roll * 1e-5;
}
double UBX_UART::getPitch()
{
	return (double)_validPacket._NavAttPacket.pitch * 1e-5;
}
double UBX_UART::getHeading()
{
	return (double)_validPacket._NavAttPacket.heading * 1e-5;
}
double UBX_UART::getAccRoll()
{
	return (double)_validPacket._NavAttPacket.accRoll * 1e-5;
}
double UBX_UART::getAccPitch()
{
	return (double)_validPacket._NavAttPacket.accPitch * 1e-5;
}
double UBX_UART::getAccHeading()
{
	return (double)_validPacket._NavAttPacket.accHeading * 1e-5;
}
double UBX_UART::getVelX()
{
	return (double)_validPacket._NavVelPacket.velX * 1e-2;
}
double UBX_UART::getVelY()
{
	return (double)_validPacket._NavVelPacket.velY * 1e-2;
}
double UBX_UART::getVelZ()
{
	return (double)_validPacket._NavVelPacket.velZ * 1e-2;
}
double UBX_UART::getVelAcc()
{
	return (double)_validPacket._NavVelPacket.sAcc * 1e-2;
}