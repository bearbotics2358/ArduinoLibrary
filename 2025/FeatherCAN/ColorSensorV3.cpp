/*
 * Copyright (c) 2020 REV Robotics
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of REV Robotics nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <Arduino.h>

#include <Wire.h>
#include "wiring_private.h"

#include "ColorSensorV3.h"

// extern TwoWire myWire;

using namespace rev;

// #include <algorithm>

// #include <frc/DriverStation.h>
// #include <frc/util/Color.h>

// confirmed with data sheet with exceptions noted BD
static constexpr int kAddress = 0x52;
static constexpr int kExpectedPartID = 0xC2;
static constexpr ColorSensorV3::GainFactor kDefaultGain = ColorSensorV3::GainFactor::k3x;
static constexpr ColorSensorV3::LEDPulseFrequency kDefaultPulseFreq = ColorSensorV3::LEDPulseFrequency::k60kHz;
static constexpr ColorSensorV3::LEDCurrent kDefaultLEDCurrent = ColorSensorV3::LEDCurrent::kPulse100mA;
static constexpr uint8_t kDefaultPulses = 32; // 8 in the data sheet
static constexpr ColorSensorV3::ProximityResolution kDefaultProxRes = ColorSensorV3::ProximityResolution::k11bit; // 8 in the data sheet
static constexpr ColorSensorV3::ProximityMeasurementRate kDefaultProxRate = ColorSensorV3::ProximityMeasurementRate::k50ms; // 100 ms in the data sheet
static constexpr ColorSensorV3::ColorResolution kDefaultColorRes = ColorSensorV3::ColorResolution::k18bit;
static constexpr ColorSensorV3::ColorMeasurementRate kDefaultColorRate = ColorSensorV3::ColorMeasurementRate::k100ms;

// This is a transformation matrix given by the chip
// manufacturer to transform the raw RGB to CIE XYZ
constexpr double ColorSensorV3::Cmatrix[9] = {
   0.048112847, 0.289453437, -0.084950826,
  -0.030754752, 0.339680186, -0.071569905,
  -0.093947499, 0.072838494,  0.34024948
};


rev::ColorSensorV3::ColorSensorV3()
//  : m_i2c(port, kAddress)
{
	/*
	if (m_simDevice) {
        m_simR = m_simDevice.CreateDouble("Red", false, 0.0);
        m_simG = m_simDevice.CreateDouble("Green", false, 0.0);
        m_simB = m_simDevice.CreateDouble("Blue", false, 0.0);
        m_simIR = m_simDevice.CreateDouble("IR", false, 0.0);
        m_simProx = m_simDevice.CreateDouble("Proximity", false, 0.0);
        return;
    }
	*/


  // can't do anything until Wire is running
  m_pwire = 0;
  return;

    if(!CheckDeviceID())
        return;

    InitializeDevice();

    // Clear the reset flag
    HasReset();
}

uint32_t ColorSensorV3::GetProximity() {
	/*
    if (m_simProx) {
        return m_simProx.Get();
    }
	*/
    return Read11BitRegister(Register::kProximityData);
}

uint32_t ColorSensorV3::GetProximity_raw() {
  uint8_t raw[2];
  uint32_t ret = 0;

  Read(Register::kProximityData, 2, raw);
  ret = ((raw[1] << 8) | raw[0]) & 0x07ff;
  
  return ret;
}

uint32_t ColorSensorV3::GetProximityTheirWay() {
  uint8_t raw[2];
  uint32_t ret = 0;

  Read(Register::kProximityData, 2, raw);
  ret = static_cast<uint16_t>(raw[0]) |
                  (static_cast<uint16_t>(raw[1]) << 8) &
                 0x7FF;

/*
        return (static_cast<uint16_t>(val[0]) |
                (static_cast<uint16_t>(val[1]) << 8)) &
               0x7FF;
*/

  return ret;
}




uint32_t ColorSensorV3::GetColor() {
    uint32_t ret;

    RawColor color = GetRawColor();
    double r = static_cast<double>(color.red);
    double g = static_cast<double>(color.green);
    double b = static_cast<double>(color.blue);
    double mag = r + g + b;
    // return frc::Color(r / mag, g / mag, b / mag);
    ret = (uint32_t)((255 * r) / mag) << 16;
    ret |= (uint32_t)((255 * g) / mag) << 8;
    ret |= (uint32_t)((255 * b) / mag);
    return ret;
    
    // Possible future work: return sRGB values
    //return GetCIEColor().ToRGB();
    return 0;
}

rev::ColorSensorV3::RawColor rev::ColorSensorV3::GetRawColor() {
    uint8_t raw[12];

		/*
    if (m_simDevice) {
        return ColorSensorV3::RawColor( static_cast<uint32_t>(m_simR.Get()),
                                        static_cast<uint32_t>(m_simG.Get()),
                                        static_cast<uint32_t>(m_simB.Get()),
                                        static_cast<uint32_t>(m_simIR.Get()) );
    }
		*/

    if(!Read(Register::kDataInfrared, 12, raw)) {
        return ColorSensorV3::RawColor( To20Bit(&raw[9]),
                                        To20Bit(&raw[3]),
                                        To20Bit(&raw[6]),
                                        To20Bit(&raw[0]) );
    }

    return ColorSensorV3::RawColor(0, 0, 0, 0);
}

/*
rev::CIEColor ColorSensorV3::GetCIEColor() {
    RawColor raw = GetRawColor();
    return rev::CIEColor( Cmatrix[0] * raw.red + Cmatrix[1] * raw.green + Cmatrix[2] * raw.blue,
                          Cmatrix[3] * raw.red + Cmatrix[4] * raw.green + Cmatrix[5] * raw.blue,
                          Cmatrix[6] * raw.red + Cmatrix[7] * raw.green + Cmatrix[8] * raw.blue );
}
*/

double ColorSensorV3::GetIR() {
	/*
    if (m_simDevice) {
        return m_simIR.Get();
    }
	*/
    return static_cast<double>(Read20BitRegister(Register::kDataInfrared));
}

void ColorSensorV3::SetGain(ColorSensorV3::GainFactor gain) {
    Write(Register::kLightSensorGain, static_cast<uint8_t>(gain));
}


bool ColorSensorV3::CheckDeviceID() {
    uint8_t partID = 0;
    if(Read(Register::kPartID, 1, &partID)) {
			// frc::DriverStation::ReportError("Could not find REV color sensor");
      Serial.println("Could not find REV color sensor");
        return false;
    }

    if(partID != kExpectedPartID) {
			// frc::DriverStation::ReportError("Unknown device found with same I2C address as REV color sensor");
      Serial.println("Unknown device found with same I2C address as REV color sensor");
        return false;
    }

    return true;
}

rev::ColorSensorV3::MainStatus rev::ColorSensorV3::GetStatus() {
    MainStatus stat = {};
    Read(Register::kMainStatus, 1, reinterpret_cast<uint8_t*>(&stat));
    return stat;
}

void ColorSensorV3::InitializeDevice() {
    Write(Register::kMainCtrl, 
          static_cast<uint8_t>(MainCtrlFields::kRGBMode) | 
          static_cast<uint8_t>(MainCtrlFields::kLightSensorEnable) | 
          static_cast<uint8_t>(MainCtrlFields::kProximitySensorEnable));

    ConfigureProximitySensorLED(kDefaultPulseFreq, kDefaultLEDCurrent, kDefaultPulses);
    ConfigureProximitySensor(kDefaultProxRes, kDefaultProxRate);
    ConfigureColorSensor(kDefaultColorRes, kDefaultColorRate);    
    SetGain(kDefaultGain);
}

void 
ColorSensorV3::ConfigureProximitySensorLED(LEDPulseFrequency freq,
                                                   LEDCurrent curr, 
                                                   uint8_t pulses) {
    Write(Register::kProximitySensorLED,
          static_cast<uint8_t>(freq) | 
          static_cast<uint8_t>(curr));

    Write(Register::kProximitySensorPulses, pulses);
}

void 
ColorSensorV3::ConfigureProximitySensor(ProximityResolution res, 
                                                ProximityMeasurementRate rate) {
    Write(Register::kProximitySensorRate, 
          static_cast<uint8_t>(res) | 
          static_cast<uint8_t>(rate));
}

void ColorSensorV3::ConfigureColorSensor(ColorResolution res, 
                                                 ColorMeasurementRate rate) {
    Write(Register::kLightSensorMeasurementRate, 
          static_cast<uint8_t>(res) | 
          static_cast<uint8_t>(rate));
}

bool ColorSensorV3::HasReset() {
    if (m_simDevice) {
        return false;
    }
    return GetStatus().PowerOnStatus != 0;
}

uint16_t ColorSensorV3::Read11BitRegister(Register reg) {
    uint8_t raw[2];

    // m_i2c.Read(static_cast<uint8_t>(reg), 2, raw);

    Read(reg, 2, raw);
    
    return To11Bit(raw);
}

uint32_t ColorSensorV3::Read20BitRegister(Register reg) {
    uint8_t raw[3];

    // m_i2c.Read(static_cast<uint8_t>(reg), 3, raw);

    Read(reg, 3, raw);

    return To20Bit(raw);
}

bool ColorSensorV3::Write(Register reg, uint8_t data) {
  // return m_i2c.Write(static_cast<uint8_t>(reg), data);

  if(m_pwire) {
    m_pwire->beginTransmission(kAddress);    // Get the slave's attention, tell it we're sending a command byte
    m_pwire->write((uint8_t)reg);            //  The command byte, sets pointer to register with address of 0x32
    m_pwire->write(data);
    m_pwire->endTransmission();              // "Hang up the line" so others can use it (can have multiple slaves & masters connected)
  }

  return 0;
}


bool ColorSensorV3::Read(Register reg, int count, uint8_t* data) {
  // return m_i2c.Read(static_cast<uint8_t>(reg), count, data);
  int i;

  if(m_pwire) {
    m_pwire->beginTransmission(kAddress);    // Get the slave's attention, tell it we're sending a command byte
    m_pwire->write((uint8_t)reg);            // The command byte, sets pointer to register with address of 0x32
    m_pwire->endTransmission();              // "Hang up the line" so others can use it (can have multiple slaves & masters connected)
  
    m_pwire->requestFrom(kAddress, count);   // Tell slave we need to read 1 byte from the current register
    for(i = 0; i < count; i++) {
      data[i] = m_pwire->read();             // read that byte into 'slaveByte2' variable
    }
    m_pwire->endTransmission();              // "Hang up the line" so others can use it (can have multiple slaves & masters connected)
  }
   
  return 0;
}

void ColorSensorV3::setWire(TwoWire *pWire)
{
  m_pwire = pWire;
}
