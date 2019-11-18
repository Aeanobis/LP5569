/************************************************
   LP5569.cpp
   Implementation file for LP5569
   Ali Rahimi @ Tomorrow-Lab
   Arduino library supporting LP5569
   Distributed as-is. no warranty is given.
*/
#include <Arduino.h>
#include <wire.h>
#include "Lp5569.h"

//register shit
static const uint8_t REG_CONFIG = 0x00;
static const uint8_t REG_CNTRL1 = 0x01;
static const uint8_t REG_CNTRL2 = 0x02;

// LED controller channels
static const uint8_t REG_D0_CTRL = 0x07;
static const uint8_t REG_D1_CTRL = 0x08;
static const uint8_t REG_D2_CTRL = 0x09;
static const uint8_t REG_D3_CTRL = 0x0a;
static const uint8_t REG_D4_CTRL = 0x0b;
static const uint8_t REG_D5_CTRL = 0x0c;
static const uint8_t REG_D6_CTRL = 0x0d;
static const uint8_t REG_D7_CTRL = 0x0e;
static const uint8_t REG_D8_CTRL = 0x0f;

// Direct PWM control registers
static const uint8_t REG_D0_PWM  = 0x16;
static const uint8_t REG_D1_PWM  = 0x17;
static const uint8_t REG_D2_PWM  = 0x18;
static const uint8_t REG_D3_PWM  = 0x19;
static const uint8_t REG_D4_PWM  = 0x1a;
static const uint8_t REG_D5_PWM  = 0x1b;
static const uint8_t REG_D6_PWM  = 0x1c;
static const uint8_t REG_D7_PWM  = 0x1d;
static const uint8_t REG_D8_PWM  = 0x1e;

// Drive current registers
static const uint8_t REG_D0_I_CTL = 0x22;
static const uint8_t REG_D1_I_CTL  = 0x23;
static const uint8_t REG_D2_I_CTL  = 0x24;
static const uint8_t REG_D3_I_CTL  = 0x25;
static const uint8_t REG_D4_I_CTL  = 0x26;
static const uint8_t REG_D5_I_CTL  = 0x27;
static const uint8_t REG_D6_I_CTL  = 0x28;
static const uint8_t REG_D7_I_CTL  = 0x29;
static const uint8_t REG_D8_I_CTL  = 0x2A;

//Charge Pump and friends
static const uint8_t REG_MISC = 0x2F;
static const uint8_t REG_PC1      = 0x30;
static const uint8_t REG_PC2      = 0x31;
static const uint8_t REG_PC3      = 0x32;
static const uint8_t REG_MISC_2 = 0x33;
static const uint8_t REG_STATUS_IRQ = 0x3C;
static const uint8_t REG_INT_GPIO   = 0x3D;
static const uint8_t REG_GLOBAL_VAR = 0x3E;
static const uint8_t REG_RESET      = 0x3F;

/*
  static const uint8_t REG_TEMP_CTL   = 0x3E;
  static const uint8_t REG_TEMP_READ  = 0x3F;
  static const uint8_t REG_TEMP_WRITE = 0x40;
  static const uint8_t REG_TEST_CTL   = 0x41;
  static const uint8_t REG_TEST_ADC   = 0x42;
*/

static const uint8_t REG_ENGINE_A_VAR = 0x42;
static const uint8_t REG_ENGINE_B_VAR = 0x43;
static const uint8_t REG_ENGINE_C_VAR = 0x44;

static const uint8_t REG_MASTER_FADE_1 = 0x46;
static const uint8_t REG_MASTER_FADE_2 = 0x47;
static const uint8_t REG_MASTER_FADE_3 = 0x48;
static const uint8_t  REG_MASTER_FADE_PWM = 0x4A;

static const uint8_t REG_PROG1_START = 0x4B;
static const uint8_t REG_PROG2_START = 0x4C;
static const uint8_t REG_PROG3_START = 0x4D;
static const uint8_t REG_PROG_PAGE_SEL = 0x4F;

// Memory is more confusing - there are 6 pages, sel by addr 4f
static const uint8_t REG_PROG_MEM_BASE = 0x50;
//static const uint8_t REG_PROG_MEM_SIZE = 0x;//
static const uint8_t REG_PROG_MEM_END  = 0x6f;

static const uint8_t REG_ENG1_MAP_MSB = 0x70;
static const uint8_t REG_ENG1_MAP_LSB = 0x71;
static const uint8_t REG_ENG2_MAP_MSB = 0x72;
static const uint8_t REG_ENG2_MAP_LSB = 0x73;
static const uint8_t REG_ENG3_MAP_MSB = 0x74;
static const uint8_t REG_ENG3_MAP_LSB = 0x75;

static const uint8_t REG_GAIN_CHANGE = 0x76;
static const uint8_t PWM_CONFIG = 0x80;

Lp5569::Lp5569(uint8_t address) {
  _address = address;
}

void Lp5569::WriteReg(uint8_t reg, uint8_t val)
{
  Wire.beginTransmission(_address);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t Lp5569::ReadReg(uint8_t reg)
{
  // Wire is awkward because it doesn't really have a register address concept.
  // http://www.arduino.cc/en/Tutorial/SFRRangerReader for reference

  Wire.beginTransmission(_address);
  Wire.write(reg);
  Wire.endTransmission(false);// false keeps connection active so we can read.

  delayMicroseconds(10);

  uint8_t status = Wire.requestFrom(_address, (uint8_t)1);
  if (status)
  {
    return (Wire.read());
  }
  else
  {
    Serial.print("readReg failed? status:");
    Serial.println(status, HEX);
  }
  return 0xff;
}


void Lp5569::Begin()
{
  Wire.begin();
  Reset();
}

void Lp5569::Enable()
{
  WriteReg(REG_CONFIG, 0x40);
  WriteReg(REG_MISC, 0x53);
  //WriteReg(REG_CNTRL1,
}

void Lp5569::Disable()
{
  uint8_t val;
  val = ReadReg(REG_CONFIG);
  val &= ~0x40;
  WriteReg(REG_CONFIG, val);
}

void Lp5569::Reset()
{
  WriteReg(REG_RESET, 0xFF);
}

void Lp5569::Womp()
{

  //WriteReg(REG_CNTRL2, 0x00);
  //WriteReg(REG_D0_PWM, 0xFF);
  //WriteReg(REG_D1_PWM, 0xFF);
  //WriteReg(REG_D2_PWM, 0xFF);
  //WriteReg(REG_D3_PWM, 0xFF);
  //WriteReg(REG_D4_PWM, 0xFF);
  //WriteReg(REG_D5_PWM, 0xFF);
  WriteReg(REG_D6_PWM, 0xFF);
  //WriteReg(REG_D7_PWM, 0xFF);
  //WriteReg(REG_D8_PWM, 0xFF);

}

void Lp5569::Orange() {
  WriteReg(REG_D7_PWM, 0x1E); //green
  WriteReg(REG_D8_PWM, 0xAA); //red

}

void Lp5569::Blue() {
  WriteReg(REG_D6_PWM, 0xEF); //blue
  WriteReg(REG_D7_PWM, 0x97); //green
  WriteReg(REG_D8_PWM, 0x3F); //red

}


Lp5569::lp_err_code Lp5569::SetChannelPWM(uint8_t channel, uint8_t value)
{
  if (channel >= NumChannels)
  {
    return LP_ERR_INVALID_CHANNEL;
  }

  WriteReg(REG_D0_PWM + channel, value);
  return LP_ERR_NONE;
}


Lp5569::lp_err_code Lp5569::SetMasterFader(uint8_t fader, uint8_t value)
{
  if (fader >= NumFaders)
  {
    return LP_ERR_INVALID_FADER;
  }

  WriteReg(REG_MASTER_FADE_1 + fader, value);

  return LP_ERR_NONE;
}

Lp5569::lp_err_code Lp5569::SetLogBrightness(uint8_t channel, bool enable)
{
  uint8_t regVal, bitVal;
  if (channel >= NumChannels)
  {
    return LP_ERR_INVALID_CHANNEL;
  }
  regVal = ReadReg(REG_D1_CTRL + channel);
  bitVal = enable ? 0x20 : 0x00;
  regVal &= ~0x20;
  regVal |= bitVal;
  WriteReg(REG_D1_CTRL + channel, regVal);

  return LP_ERR_NONE;
}
Lp5569::lp_err_code Lp5569::SetDriveCurrent(uint8_t channel, uint8_t value)
{
  if (channel >= NumChannels)
  {
    return LP_ERR_INVALID_CHANNEL;
  }
  WriteReg(REG_D1_I_CTL + channel, value);
  return LP_ERR_NONE;
}

Lp5569::lp_err_code Lp5569::AssignChannelToMasterFader(uint8_t channel, uint8_t fader)
{
  uint8_t regVal, bitVal;

  if (channel >= NumChannels)
  {
    return LP_ERR_INVALID_CHANNEL;
  }
  else if (fader >= NumFaders)
  {
    return LP_ERR_INVALID_FADER;
  }
  regVal = ReadReg(REG_D1_CTRL + channel);
  bitVal = (fader + 1) & 0x03;
  bitVal <<= 6;
  regVal &= ~0xc0;
  regVal |= bitVal;
  WriteReg(REG_D1_CTRL + channel, regVal);
  return LP_ERR_NONE;
}

Lp5569:: lp_err_code Lp5569::Yoho() {

for(int i= 0x07; i <= 0x0f; i++){
  WriteReg(i, 0x20);
  }
}



Lp5569::lp_err_code Lp5569Engines::LoadProgram(const uint16_t* prog, uint8_t len)
{
  uint8_t val;
  uint8_t page;

  if (len >= NumInstructions)
  {
    return LP_ERR_PROGRAM_LENGTH;
  }

  // set up program write
  // start in execution disabled mode (0b00)
  // required to get into load mode.
  // "Load program mode can be entered from the disabled mode only.  be
  // entered from the disabled mode only."
  WriteReg(REG_CNTRL2, 0x00);
  WriteReg(REG_CNTRL2, 0x54);

  WaitForBusy();

  // try to write program from example
  // datasheet says MSB of each instruction is in earlier address
  // TBD: could optimize with a sequence of byte writes, using auto increment

  // use auto-increment of chip - enabled in MISC.
  // If it gets turned off, this breaks.  TBD: set it explicitly?

  // Write complete pages, setting page reg for each.
  for (page = 0; page < (len / 16); page++)
  {
    WriteReg(REG_PROG_PAGE_SEL, page);

    for (uint8_t i = 0; i < 16; i++)
    {
      Wire.beginTransmission(_address);
      Wire.write((REG_PROG_MEM_BASE + (i * 2)));
      // MSB then LSB
      Wire.write((prog[(i + (page * 16))] >> 8) & 0xff);
      Wire.write(prog[i + (page * 16)] & 0xff);
      Wire.endTransmission();
    }
  }

  // plus any incomplete pages
  page = len / 16;
  WriteReg(REG_PROG_PAGE_SEL, page);
  for (uint8_t i = 0; i < (len % 16); i++)
  {
    Wire.beginTransmission(_address);
    Wire.write((REG_PROG_MEM_BASE + (i * 2)));
    // MSB then LSB
    Wire.write((prog[i + (page * 16)] >> 8) & 0xff);
    Wire.write(prog[i + (page * 16)] & 0xff);
    Wire.endTransmission();
  }

  WriteReg(REG_CNTRL2, 0x00);

  return LP_ERR_NONE;
}

void Lp5569Engines::WaitForBusy()
{
  uint8_t val;

  // then wait to change modes
  do
  {
    val = ReadReg(REG_STATUS_IRQ) & 0x10; // engine busy bit
  }
  while (val);

}


Lp5569::lp_err_code Lp5569Engines::VerifyProgram(const uint16_t* prog, uint8_t len)
{
  uint8_t val, page;

  if (len >= NumInstructions)
  {
    return LP_ERR_PROGRAM_LENGTH;
  }

  WriteReg(REG_CNTRL2, 0x00);// engines into disable mode - required for entry to program mode.
  WriteReg(REG_CNTRL2, 0x54);// engines into program mode?
  //try to read  program from chip,
  // datasheet says MSB of each instruction is in earlier address
  // TBD: could optimize with a sequence of byte writes, using auto increment

  // Auto-increment may not work for sequential reads...
  for (page = 0; page < (len / 16); page++)
  {
    WriteReg(REG_PROG_PAGE_SEL, page);

    for (uint8_t i = 0; i < 16; i++)
    {
      uint16_t msb, lsb;
      uint8_t addr = (REG_PROG_MEM_BASE + (i * 2));
      //Serial.print("Verifying: ");
      //Serial.println(addr, HEX);

      msb = ReadReg(addr);
      lsb = ReadReg(addr + 1);

      lsb |= (msb << 8);

      if (lsb != prog[i + (page * 16)])
      {
        // Serial.print("program mismatch.  Idx:");
        // Serial.print(i);
        // Serial.print(" local:");
        // Serial.print(prog[i + (page*16)], HEX);
        // Serial.print(" remote:");
        // Serial.println(lsb, HEX);

        return LP_ERR_PROGRAM_VALIDATION;
      }
    }
  }

  // plus any incomplete pages
  page = len / 16;
  WriteReg(REG_PROG_PAGE_SEL, page);
  for (uint8_t i = 0; i < (len % 16); i++)
  {
    uint16_t msb, lsb;
    uint8_t addr = (REG_PROG_MEM_BASE + (i * 2));
    // Serial.print("Verifying: ");
    // Serial.println(addr, HEX);

    msb = ReadReg(addr);
    lsb = ReadReg(addr + 1);

    lsb |= (msb << 8);

    if (lsb != prog[i + (page * 16)])
    {
      // Serial.print("program mismatch.  Idx:");
      // Serial.print(i);
      // Serial.print(" local:");
      // Serial.print(prog[i + (page*16)], HEX);
      // Serial.print(" remote:");
      // Serial.println(lsb, HEX);
      //
      return LP_ERR_PROGRAM_VALIDATION;
    }
  }

  WriteReg(REG_CNTRL2, 0x00);

  return LP_ERR_NONE;
}

Lp5569::lp_err_code Lp5569Engines::SetEngineEntryPoint(uint8_t engine, uint8_t addr)
{

  if (engine >= NumEngines)
  {
    return LP_ERR_INVALID_ENGINE;
  }

  WriteReg(REG_PROG1_START + engine, addr);

  return LP_ERR_NONE;
}

Lp5569::lp_err_code Lp5569Engines::SetEnginePC(uint8_t engine, uint8_t addr)
{
  uint8_t control_val, control2_val, temp;;

  if (engine >= NumEngines)
  {
    return LP_ERR_INVALID_ENGINE;
  }

  // There are 6 pages of 16 instructions each (0..95)
  if (addr >= NumInstructions)
  {
    return LP_ERR_PROGRAM_PC;
  }

  // In Ctl1 descriptions:
  //00 = hold: Hold causes the execution engine to finish the current instruction and then stop. Program counter
  //(PC) can be read or written only in this mode.

  control_val = ReadReg(REG_CNTRL1);
  control2_val = ReadReg(REG_CNTRL2);

  temp = (control_val & ~(0xC0 >> ((engine - 1) * 2)));

  WriteReg(REG_CNTRL2, 0xfc); // halt engines immediately.
  WriteReg(REG_CNTRL1, temp);// put engine in load mode

  WriteReg(REG_PC1 + engine, addr);

  // restore prev mode?
  WriteReg(REG_CNTRL1, control_val);
  WriteReg(REG_CNTRL2, control2_val);

  return LP_ERR_NONE;
}

uint8_t Lp5569Engines::GetEnginePC(uint8_t engine)
{
  // must set Hold to touch PC...
  uint8_t control_val, pc_val;

  if (engine >= NumEngines)
  {
    return -1;
  }

  pc_val = ReadReg(REG_PC1 + engine);

  return (pc_val);
}


uint8_t Lp5569Engines::GetEngineMode(uint8_t engine)
{
  uint8_t val;

  if (engine >= NumEngines)
  {
    return 0xff;
  }

  val = ReadReg(REG_CNTRL1);
  if (engine = 1) {
    val >>= (engine * 6);
    val &= 0x03;
  }
  else if (engine = 2) {
    val >>= (engine * 2);
    val &= 0x03;
  }
  else if (engine = 3) {
    val >>= 2;
    val &= 0x0C;
  }
  else {
    return LP_ERR_INVALID_ENGINE;
  };
  return (val);
}


uint8_t Lp5569Engines::GetEngineMap(uint8_t engine)
{
  if (engine >= NumEngines)
  {
    return 0xFF;
  }

  return (ReadReg(REG_ENG1_MAP_LSB + engine));
}

Lp5569::lp_err_code Lp5569Engines::SetEngineModeHold(uint8_t engine)
{
  uint8_t val;

  if (engine >= NumEngines)
  {
    return LP_ERR_INVALID_ENGINE;
  }

  // Set the enghine to "free running" execution type
  // bits to 0b00
  val = ReadReg(REG_CNTRL1);
  if (engine = 1) {
    val >>= (engine * 6);
    val &= 0x02;
    val <<= engine * 6;
  }
  else if (engine = 2) {
    val >>= (engine * 2);
    val &= 0x02;
    val <<= engine * 2;
  }
  else if (engine = 3) {
    val >>= 2;
    val &= 0x02;
    val <<= 2;
  }
  WriteReg(REG_CNTRL1, val );

  return LP_ERR_NONE;
}

Lp5569::lp_err_code Lp5569Engines::SetEngineModeStep(uint8_t engine)
{
  uint8_t val;

  if (engine >= NumEngines)
  {
    return LP_ERR_INVALID_ENGINE;
  }

  // Set the enghine to "single step" execution type
  // bits to 0b01
  if (engine = 1) {
    val >>= (engine * 6);
    val &= 0x01;
    val |= 0x01;
    val <<= engine * 6;
  }
  else if (engine = 2) {
    val >>= (engine * 2);
    val &= 0x01;
    val |= 0x01;
    val <<= engine * 2;
  }
  else if (engine = 3) {
    val >>= 2;
    val &= 0x01;
    val |= 0x01;
    val <<= 2;
  }
  WriteReg(REG_CNTRL1, val );

  return LP_ERR_NONE;
}

Lp5569::lp_err_code Lp5569Engines::SetEngineModeOnce(uint8_t engine)
{
  uint8_t val;
  // This mode might not be the most useful.
  // It executes the pointed instruction, then
  // sets exec mode to hold, and resets the PC.
  // It's an astringent form of step (which advances the PC, instead)

  if (engine >= NumEngines)
  {
    return LP_ERR_INVALID_ENGINE;
  }

  // Set the enghine to "one shot" execution type
  // Bits to 0b11
  if (engine = 1) {
    val >>= (engine * 6);
    val &= 0x00;
    val |= 0x11;
    val <<= engine * 6;
  }
  else if (engine = 2) {
    val >>= (engine * 2);
    val &= 0x00;
    val |= 0x11;
    val <<= engine * 2;
  }
  else if (engine = 3) {
    val >>= 2;
    val &= 0x00;
    val |= 0x11;
    val <<= 2;
  }

  WriteReg(REG_CNTRL1, val );

  return LP_ERR_NONE;

}

Lp5569::lp_err_code Lp5569Engines::SetEngineModeFree(uint8_t engine)
{
  uint8_t val;

  if (engine >= NumEngines)
  {
    return LP_ERR_INVALID_ENGINE;
  }

  // Set the engine to "free running" execution type
  val = ReadReg(REG_CNTRL1);
  if (engine = 1) {
    val >>= (engine * 6);
    val &= 0x10;
    val |= 0x10;
    val <<= (engine * 6);
  }
  else if (engine = 2) {
    val >>= (engine * 2);
    val &= 0x10;
    val |= 0x10;
    val <<= (engine * 2);
  }
  else if (engine = 3) {
    val >>= 2;
    val &= 0x10;
    val |= 0x10;
    val <<= 2;
  }
  WriteReg(REG_CNTRL1, val );

  return LP_ERR_NONE;
}

Lp5569::lp_err_code Lp5569Engines::SetEngineRunning(uint8_t engine)
{
  uint8_t val;

  if (engine >= NumEngines)
  {
    return LP_ERR_INVALID_ENGINE;
  }

  // This assumes that a suitable run mode in CNTRL1 was already selected.
  // start execution by setting "run program" mode
  val = ReadReg(REG_CNTRL2);
  val &= ~(0xC0 >> ((engine - 1) * 2));
  val |= (0x80 >> ((engine - 1) * 2));
  WriteReg(REG_CNTRL2, val);

  return LP_ERR_NONE;
}


/********************************************************************************/
/**  Derived class - interrupt related functions. **/
/********************************************************************************/

uint8_t Lp5569Engines::ClearInterrupt()
{
  // TBD: make this more channel specific?
  return ( ReadReg(REG_STATUS_IRQ) & 0x07);
}

void Lp5569Engines::OverrideIntToGPO(bool overrideOn )
{
  uint8_t regVal;
  if (overrideOn)
  {
    regVal = 0x04;
  }
  else
  {
    regVal = 0;
  }
  WriteReg(REG_INT_GPIO, regVal);
}

bool Lp5569Engines::SetIntGPOVal(bool value)
{
  uint8_t regVal;

  regVal = ReadReg(REG_INT_GPIO);

  if (!(regVal & 0x04))
  {
    return LP_ERR_GPIO_OFF;
  }
  if (value)
  {
    regVal |= 0x01;
  }
  else
  {
    regVal &= ~0x01;
  }
  WriteReg(REG_INT_GPIO, regVal);
  return LP_ERR_NONE;
}

float  Lp5569Engines::ReadLEDADC(uint8_t channel)
{
  uint8_t reading;
  float volts;

  if (channel >= NumChannels)
  {
    return 0.0;
  }
  reading = ReadADCInternal(channel & 0x0f);
  volts = (reading * 0.03) - 1.478;
  return volts;
}

float  Lp5569Engines::ReadVoutADC()
{
  uint8_t reading;
  float volts;

  reading = ReadADCInternal(0x0f);

  volts = (reading * 0.03) - 1.478;
  return volts;
}

float  Lp5569Engines::ReadVddADC()
{
  uint8_t reading;
  float volts;

  reading = ReadADCInternal(0x10);

  volts = (reading * 0.03) - 1.478;
  return volts;
}

float  Lp5569Engines::ReadIntADC()
{
  // reads voltage at interrupt pin
  uint8_t reading;
  float volts;
  reading = ReadADCInternal(0x11);

  volts = (reading * 0.03) - 1.478;
  return volts;
}
