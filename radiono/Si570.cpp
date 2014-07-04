/*
 * Si570 Library for Arduino
 *
 * MIT License
 *
 * Copyright Jeff Whitlatch - ko7m - 2014
 * Based on previous work by Thomas Sarlandie which was
 * based on previous work by Ashar Farhan
 */

#include <Arduino.h>
#include <Wire.h>

#include "Si570.h"

//#define IF_FREQ   (19997000L)

// Debug output
static void debug(char const *fmt, ... ) 
{
  char tmp[128]; // resulting string limited to 128 chars
  va_list args;
  va_start (args, fmt );
  vsnprintf(tmp, sizeof(tmp), fmt, args);
  va_end (args);
  Serial.println(tmp);
}

// Debug output without outputting a newline
static void debugx(char const *fmt, ... ) 
{
  char tmp[128]; // resulting string limited to 128 chars
  va_list args;
  va_start (args, fmt );
  vsnprintf(tmp, sizeof(tmp), fmt, args);
  va_end (args);
  Serial.print(tmp);
}

// Memory dump 
static void dump(const void *mem, int n)
{
  const unsigned char *p = reinterpret_cast<const unsigned char *>( mem );
  for (int i = 0; i < n; i++)
  {
    debugx("%02x", p[i]);
  }
  debug("");
}

// Initialize the Si570 and determine its internal crystal frequency given the default output frequency
Si570::Si570(uint8_t si570_address, uint32_t calibration_frequency) 
{
  i2c_address = si570_address;
  debug("Si570 init, calibration frequency = %lu", calibration_frequency);
  Wire.begin();

  // Disable internal pullups - You will need external 3.3v pullups.
  digitalWrite(SDA, 0);
  digitalWrite(SCL, 0);

  // We are about the reset the Si570, so set the current and center frequency to the calibration frequency.
  f_center = frequency = calibration_frequency;

  // Force Si570 to reset to initial freq
  debug("Resetting Si570");
  i2c_write(135,0x01);
  delay(20);

  if (read_si570()) 
  {
    debug("Successfully initialized Si570");
    freq_xtal = (unsigned long) ((uint64_t) calibration_frequency * getHSDIV() * getN1() * (1L << 28) / getRFREQ());
    status = SI570_READY;
  }
  else
  {
    // Use the factory default if we were unable to talk to the chip
    freq_xtal = 114285000L;
    debug("Unable to properly initialize Si570");
    status = SI570_ERROR;
  }
  debug("freq_xtal = %04lu", freq_xtal);
}

// Debug routine for examination of Si570 state
void Si570::debugSi570()
{
  debug(" --- Si570 Debug Info ---");
  debug("Status: %i", status);
  for (int i = 7; i < 15; i++) {
    debug("Register[%i] = %02x", i, dco_reg[i]);
  }
  debug("HSDIV = %i, N1 = %i", getHSDIV(), getN1());
  debug("RFREQ (hex): %04lx%04lx", (uint32_t)(getRFREQ() >> 32), (uint32_t)(getRFREQ() & 0xffffffff));
}

// Return the 8 bit HSDIV value from register 7
uint8_t Si570::getHSDIV()
{
  uint8_t hs_reg_value = dco_reg[7] >> 5;
  return 4 + hs_reg_value;
}

// Compute and return the 8 bit N1 value from registers 7 and 8
uint8_t Si570::getN1()
{
  uint8_t n_reg_value = ((dco_reg[7] & 0x1F) << 2) + (dco_reg[8] >> 6);
  return n_reg_value + 1;
}

// Return 38 bit RFREQ value in a 64 bit integer
uint64_t Si570::getRFREQ()
{
  fdco  = (uint64_t)(dco_reg[8] & 0x3F) << 32;
  fdco |= (uint64_t) dco_reg[9] << 24;
  fdco |= (uint64_t) dco_reg[10] << 16;
  fdco |= (uint64_t) dco_reg[11] << 8;
  fdco |= (uint64_t) dco_reg[12];

  return fdco;
}

// Write a byte to I2C device
void Si570::i2c_write(uint8_t reg_address, uint8_t data)
{
  Wire.beginTransmission(i2c_address);
  Wire.write(reg_address);
  Wire.write(data);
  Wire.endTransmission();
}

// Write length bytes to I2C device.
int Si570::i2c_write(uint8_t reg_address, uint8_t *data, uint8_t length)
{
  Wire.beginTransmission(i2c_address);
  Wire.write(reg_address);
  Wire.write(data, length);

  int error = Wire.endTransmission();
  if (error != 0) 
  {
    debug("Error writing %i bytes to register %i: %i", length, reg_address, error);
    return -1;
  }
  return length;
}

// Read a one byte register from the I2C device
uint8_t Si570::i2c_read(uint8_t reg_address) 
{
  uint8_t rdata = 0xFF;
  Wire.beginTransmission(i2c_address);
  Wire.write(reg_address);
  Wire.beginTransmission(i2c_address);
  Wire.write(reg_address);
  Wire.endTransmission();
  Wire.requestFrom(i2c_address, (uint8_t)1);
  if (Wire.available()) rdata = Wire.read();
  return rdata;
}

// Read multiple bytes fromt he I2C device
int Si570::i2c_read(uint8_t reg_address, uint8_t *output, uint8_t length) {
  Wire.beginTransmission(i2c_address);
  Wire.write(reg_address);

  int error = Wire.endTransmission();
  if (error != 0) 
  {
    debug("Error reading %i bytes from register %i. endTransmission() returned %i", reg_address, error);
    return 0;
  }

  int len = Wire.requestFrom(i2c_address,length);
  if (len != length) 
  {
    debug("Requested %i bytes and only got %i bytes", length, len);
  }
  for (int i = 0; i < len && Wire.available(); i++)
    output[i] = Wire.read();

  return len;
}

// Read the Si570 chip and populate dco_reg values
bool Si570::read_si570(){
  // Try 3 times to read the registers
  for (int i = 0; i < 3; i++) 
  {
    // we have to read eight consecutive registers starting at register 7
    if (i2c_read(7, &(dco_reg[7]), 6) == 6) 
    {
      return true;
    }
    debug("Error reading Si570 registers... Retrying.");
    delay(50);
  }
  return false;
}

// Write dco_reg values to the Si570
void Si570::write_si570()
{
  int idco;

  // Freeze DCO
  idco = i2c_read(137);
  i2c_write(137, idco | 0x10 );

  i2c_write(7, &dco_reg[7], 6);

  // Unfreeze DCO
  i2c_write(137, idco & 0xEF);

  // Set new freq
  i2c_write(135,0x40);
}

// In the case of a frequency change < 3500 ppm, only RFREQ must change
void Si570::qwrite_si570()
{
  int idco;

  // Freeze the M Control Word to prevent interim frequency changes when writing RFREQ registers.
  idco = i2c_read(135);
  i2c_write(135, idco | 0x20);

  // Write RFREQ registers
  i2c_write(7, &dco_reg[7], 6);

  // Unfreeze the M Control Word
  i2c_write(135, idco &  0xdf);
}

#define fDCOMinkHz 4850000	// Minimum DCO frequency in kHz
#define fDCOMaxkHz 5670000  // Maximum DCO frequency in KHz\

// Locate an appropriate set of divisors (HSDiv and N1) give a desired output frequency
int Si570::findDivisors(uint32_t fout)
{
  const uint16_t HS_DIV[] = {11, 9, 7, 6, 5, 4};
  uint32_t fout_kHz = fout / 1000;

  // Floor of the division
  uint16_t maxDivider = fDCOMaxkHz / fout_kHz;
 
  // Ceiling of the division
  n1 = 1 + ((fDCOMinkHz - 1) / fout_kHz / 11);

  if (n1 < 1 || n1 > 128)
    return SI570_ERROR;

  while (n1 <= 128) 
  {
    if (0 == n1 % 2 || 1 == n1)
    {
      // Try each divisor from largest to smallest order to minimize power
      for (int i = 0; i < 6 ; ++i) 
      {
        hs = HS_DIV[i];
        if (hs * n1 <= maxDivider) 
          return SI570_SUCCESS;
      }
    }
    n1++;
  }
  return SI570_ERROR;
}

// Set RFREQ register (38 bits)
void Si570::setRFREQ(uint32_t fnew)
{
  // Calculate new DCO frequency
  fdco = (uint64_t) fnew * hs * n1;

  // Calculate the new RFREQ value
  rfreq = (fdco << 28) / freq_xtal;

  // Round the result
  rfreq = rfreq + ((rfreq & 1<<(28-1))<<1);

  // Reset all Si570 registers
  for (int i = 7; i <= 12; i++)
    dco_reg[i] = 0;
  
  // Set up the RFREQ register values
  dco_reg[12] = rfreq & 0xff;
  dco_reg[11] = rfreq >> 8 & 0xff;
  dco_reg[10] = rfreq >> 16 & 0xff;
  dco_reg[9]  = rfreq >> 24 & 0xff;
  dco_reg[8]  = rfreq >> 32 & 0x3f;

  // set up HS and N1 in registers 7 and 8
  dco_reg[7]  = (hs - 4) << 5;
  dco_reg[7]  = dco_reg[7] | ((n1 - 1) >> 2);
  dco_reg[8] |= ((n1-1) & 0x3) << 6;
  debugSi570();
}

// Set the Si570 frequency
Si570_Status Si570::setFrequency(uint32_t newfreq) 
{
  // If the current frequency has not changed, we are done
  if (frequency == newfreq)
    return status;
    
    //debug("\n\nStart: %lu", newfreq); // DEBUG ###

  // Check how far we have moved the frequency
  uint32_t delta_freq = abs(newfreq - f_center);

  // Calculate a 3500 ppm frequency change
  uint32_t max_delta = f_center * 10035 / 10000;

  // If the jump is small enough, we don't have to fiddle with the dividers
  if (delta_freq < max_delta) 
  {
    setRFREQ(newfreq);
    frequency = newfreq;
    qwrite_si570();
  }
  else 
  {
    // otherwise it is a big jump and we need a new set of divisors and reset center frequency
    int err = findDivisors(newfreq);
    setRFREQ(newfreq);
    f_center = frequency = newfreq;
    write_si570();
  }
  
    //debug("End: %lu", newfreq); // DEBUG ###
  return status;
}
