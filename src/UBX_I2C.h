/*
UBX.h
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
#ifndef UBX_I2C_h
#define UBX_I2C_h

#include "Arduino.h"

#include <Wire.h>
#define I2C_BUFFER_LENGTH 32
#define UBX_MAX_LENGTH 96

//-=-=-=-=- UBX binary specific variables

class UBX_I2C{
  public:
    UBX_I2C(void);
    //By default use the default I2C address, and use Wire port
    void begin(TwoWire &wirePort);
	bool isConnected(); //Returns turn if device answers on _gpsI2Caddress address

    enum FixType {
      NO_FIX = 0,
      DEAD_RECKONING,
      FIX_2D,
      FIX_3D,
      GNSS_AND_DEAD_RECKONING,
      TIME_ONLY
    };
    enum PowerSaveMode {
      NOT_ACTIVE = 0,
      ENABLED,
      ACQUISITION,
      TRACKING,
      OPTIMIZED_TRACKING,
      INACTIVE
    };
    enum CarrierPhaseStatus {
      NO_SOL = 0,
      FLOAT_SOL,
      FIXED_SOL
    };
    bool sendCfg();
    uint8_t readSensor();
    uint32_t getTow_ms();
    uint16_t getYear();
    uint8_t getMonth();
    uint8_t getDay();
    uint8_t getHour();
    uint8_t getMin();
    uint8_t getSec();
    int32_t getNanoSec();
    uint8_t getNumSatellites();

    double getRoll();
    double getPitch();
    double getHeading();
    double getAccRoll();
    double getAccPitch();
    double getAccHeading();

    double getLongitude_deg();
    double getLatitude_deg();
    double getEllipsoidHeight_m();
    double getMSLHeight_m();
    double getHorizontalAccuracy_m();
    double getVerticalAccuracy_m();
    double getNorthVelocity_ms();
    double getEastVelocity_ms();
    double getDownVelocity_ms();
    double getGroundSpeed_ms();
    double getSpeedAccuracy_ms();
    double getMotionHeading_deg();
    double getVehicleHeading_deg();
    double getHeadingAccuracy_deg();
    float getMagneticDeclination_deg();
    float getMagneticDeclinationAccuracy_deg();
    float getpDOP();
    enum FixType getFixType();
    enum PowerSaveMode getPowerSaveMode();
    enum CarrierPhaseStatus getCarrierPhaseStatus();
    bool isGnssFixOk();
    bool isDiffCorrApplied();
    bool isHeadingValid();
    bool isConfirmedDate();
    bool isConfirmedTime();
    bool isTimeDateConfirmationAvail();
    bool isValidDate();
    bool isValidTime();
    bool isTimeFullyResolved();
    bool isMagneticDeclinationValid();
    uint8_t getFusionMode();
    uint8_t getNumSens();
    uint32_t getBitfield0();
    double getxAngRate();
    double getyAngRate();
    double getzAngRate();
    double getxAccel();
    double getyAccel();
    double getzAccel();
	
	double getVelX();
	double getVelY();
	double getVelZ();
	double getVelAcc();
	
    uint32_t getRawData0();
    uint32_t getRawData1();
    uint32_t getRawData2();
    uint32_t getRawData3();
    uint32_t getRawData4();
    uint32_t getRawData5();
    uint32_t getRawData6();
    uint32_t getRawsTtag0();
    uint32_t getRawsTtag1();
    uint32_t getRawsTtag2();
    uint32_t getRawsTtag3();
    uint32_t getRawsTtag4();
    uint32_t getRawsTtag5();
    uint32_t getRawsTtag6();

    uint32_t getMeaData0();
    uint32_t getMeaData1();
    uint32_t getMeaData2();
    uint32_t getMeaData3();
    uint32_t getMeaData4();
    uint32_t getMeaData5();
    uint32_t getMeaData6();
    uint32_t getMeaCalibTtag0();
    uint32_t getMeaCalibTtag1();
    uint32_t getMeaCalibTtag2();
    uint32_t getMeaCalibTtag3();
    uint32_t getMeaCalibTtag4();
    uint32_t getMeaCalibTtag5();
    uint32_t getMeaCalibTtag6();

	enum _ubxMsgType {
	MT_NONE,
	MT_NAV_ATT,
	MT_NAV_PVT,
	MT_NAV_VEL,
	MT_ESF_INS,
	MT_ESF_STA
	};
	private:
	struct _UBX_MSG {
    uint16_t msg_class_id;
	uint16_t msg_length;
    uint8_t payload[UBX_MAX_LENGTH];
	};
    struct _UBX_NAV_ATT {
      uint16_t msg_class_id;
	  uint16_t msg_length;
      uint32_t iTOW;
      uint8_t version;
      uint8_t reserved1[3];
      int32_t roll;
      int32_t pitch;
      int32_t heading;
      uint32_t accRoll;
      uint32_t accPitch;
      uint32_t accHeading;
    };
    struct _UBX_NAV_PVT {
      uint16_t msg_class_id;
	  uint16_t msg_length;
      uint32_t iTOW;
      uint16_t year;
      uint8_t month;
      uint8_t day;
      uint8_t hour;
      uint8_t min;
      uint8_t sec;
      uint8_t valid;
      uint32_t tAcc;
      int32_t nano;
      uint8_t fixType;
      uint8_t flags;
      uint8_t flags2;
      uint8_t numSV;
      int32_t lon;
      int32_t lat;
      int32_t height;
      int32_t hMSL;
      uint32_t hAcc;
      uint32_t vAcc;
      int32_t velN;
      int32_t velE;
      int32_t velD;
      int32_t gSpeed;
      int32_t headMot;
      uint32_t sAcc;
      uint32_t headAcc;
      uint16_t pDOP;
      uint8_t reserved[6];
      int32_t headVeh;
      int16_t magDec;
      uint16_t magAcc;
    };
    struct _UBX_NAV_VEL {
      uint16_t msg_class_id;
	  uint16_t msg_length;
      uint32_t iTOW;
      int32_t velX;
      int32_t velY;
      int32_t velZ;
      uint32_t sAcc;
    };
    struct _UBX_ESF_INS {
      uint16_t msg_class_id;
	  uint16_t msg_length;
      uint32_t bitfield0;
      uint32_t reserved1;
      uint32_t iTOW;
      int32_t xAngRate;
      int32_t yAngRate;
      int32_t zAngRate;
      int32_t xAccel;
      int32_t yAccel;
      int32_t zAccel;
     };
    struct _UBX_ESF_STA {
      uint16_t msg_class_id;
	  uint16_t msg_length;
      uint32_t iTOW;
      uint8_t version;
      uint8_t reserved1[7];
      uint8_t fusionMode;
      uint16_t reserved2;
      uint8_t numSens;
     };

	union _UBX_MSG_U{
    _UBX_MSG     _UbxMsgPacket;
	_UBX_NAV_ATT _NavAttPacket;
	_UBX_NAV_PVT _NavPvtPacket;
	_UBX_NAV_VEL _NavVelPacket;
    _UBX_ESF_INS _EsfInsPacket;
    _UBX_ESF_STA _EsfStaPacket;
    };
	
	_UBX_MSG_U _tempPacket,_validPacket;

	const uint8_t _gpsI2Caddress = 0x42; //Default 7-bit unshifted address of the ublox 6/7/8/M8 series
	const uint8_t I2C_POLLING_WAIT_MS = 25; //Limit checking of new characters to every X ms
    const uint8_t _ubxHeader[2] = {0xB5, 0x62};
	
	// Variables
    TwoWire* _bus; //The generic connection to user's chosen I2C hardware
	unsigned long lastCheck = 0;
    uint8_t _currentMsgType;
  	uint8_t _parserState;
    uint8_t _checksum[2];

	// Functions
	bool _sendCommand(); //Given a packet and payload, send everything including CRC bytes
	uint8_t _parse(uint8_t _byte);
	void _calcChecksum(uint8_t* CK, uint8_t* payload, uint16_t length);
};

#endif
