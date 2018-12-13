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

class UBX_I2C{
  public:
    UBX_I2C(void);
    //By default use the default I2C address, and use Wire port
    void begin(TwoWire &wirePort);
	boolean isConnected(); //Returns turn if device answers on _gpsI2Caddress address

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
    double getLongitude_deg();
    double getLatitude_deg();
    double getEllipsoidHeight_ft();
    double getMSLHeight_ft();
    double getHorizontalAccuracy_ft();
    double getVerticalAccuracy_ft();
    double getNorthVelocity_fps();
    double getEastVelocity_fps();
    double getDownVelocity_fps();
    double getGroundSpeed_fps();
    double getSpeedAccuracy_fps();
    double getMotionHeading_deg();
    double getVehicleHeading_deg();
    double getHeadingAccuracy_deg();
    float getMagneticDeclination_deg();
    float getMagneticDeclinationAccuracy_deg();
    double getLongitude_rad();
    double getLatitude_rad();
    double getEllipsoidHeight_m();
    double getMSLHeight_m();
    double getHorizontalAccuracy_m();
    double getVerticalAccuracy_m();
    double getNorthVelocity_ms();
    double getEastVelocity_ms();
    double getDownVelocity_ms();
    double getGroundSpeed_ms();
    double getSpeedAccuracy_ms();
    double getMotionHeading_rad();
    double getVehicleHeading_rad();
    double getHeadingAccuracy_rad();
    float getMagneticDeclination_rad();
    float getMagneticDeclinationAccuracy_rad();
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

	private:
	enum _ubxMsgType {
	MT_NONE,
	MT_NAV_PVT,
	MT_ESF_INS,
	MT_ESF_STA
	};
    struct _UBX_MSG {
    uint16_t msg_class_id;
	uint16_t msg_length;
    uint8_t payload[UBX_MAX_LENGTH];
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

	_UBX_MSG _tempPacket;
    _UBX_NAV_PVT _NavPvtPacket;
    _UBX_ESF_INS _EsfInsPacket;
    _UBX_ESF_STA _EsfStaPacket;

    const double _PI = 3.14159265358979323846;
    const float _m2ft = 3.28084;
    const float _deg2rad = _PI/180.0;
	
	// Variables
    TwoWire* _bus; //The generic connection to user's chosen I2C hardware
	const uint8_t _gpsI2Caddress = 0x42; //Default 7-bit unshifted address of the ublox 6/7/8/M8 series
	const uint8_t I2C_POLLING_WAIT_MS = 25; //Limit checking of new characters to every X ms
	unsigned long lastCheck = 0;
    uint8_t _currentMsgType = MT_NONE;
	
  	uint8_t _parserState;
    const uint8_t _ubxHeader[2] = {0xB5, 0x62};
    uint8_t _checksum[2];

	// Functions
	uint8_t _parse(uint8_t _byte);
	void _calcChecksum(uint8_t* CK, uint8_t* payload, uint16_t length);
};

#endif
