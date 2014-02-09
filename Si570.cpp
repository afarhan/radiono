/*
 * Si570 Library for Arduino
 *
 * MIT License
 *
 * Copyright Thomas Sarlandie - 2014
 * Based on previous work by Ashar Farhan
 */

#include <Arduino.h>
#include <Wire.h>

#include "Si570.h"

#define FREQ_XTAL (114292532l)

Si570::Si570(char si570_address) {
  i2c_address = si570_address;

  Wire.begin();

  // Disable internal pullups - You will need external 3.3v pullups.
  digitalWrite(SDA, 0);
  digitalWrite(SCL, 0);

  f_center = 0;
  frequency = 0;
  // TODO: make that configurable
  freq_xtal = FREQ_XTAL;
  // Force Si570 to reset to initial freq
  i2c_write(135,0x01);
  delay(20);
  read_si570();
}

void Si570::i2c_write(char reg_address, char data) {
  int rdata = data;
  Wire.beginTransmission(this->i2c_address);
  Wire.write(reg_address);
  Wire.write(rdata);
  Wire.endTransmission();
}

char Si570::i2c_read(int reg_address) {
  unsigned char rdata = 0xFF;
  Wire.beginTransmission(this->i2c_address);
  Wire.write(reg_address);
  Wire.endTransmission();
  Wire.requestFrom(this->i2c_address,1);
  if (Wire.available()) rdata = Wire.read();
  return rdata;
}

void Si570::read_si570(){
  //we have to read eight consecutive registers starting at register 5
  for (int i = 7; i <= 12; i++)
    this->dco_reg[i] = i2c_read(i);
}

void Si570::write_si570()
{
  int idco, i;

  // Freeze DCO
  idco = i2c_read(137);
  i2c_write(137, idco | 0x10 );

  i2c_write(7, this->dco_reg[7]);

  //Set Registers
  for( i=7; i <= 12; i++){
    i2c_write(i, this->dco_reg[i]);
    idco = i2c_read(i);
  }

  // Unfreeze DCO
  idco = i2c_read(137);
  i2c_write (137, idco & 0xEF );

  // Set new freq
  i2c_write(135,0x40);
}

void Si570::qwrite_si570()
{
  int i, idco;
  //Set Registers
  for( i=8; i <= 12; i++){
    i2c_write(i, this->dco_reg[i]);
    idco = i2c_read(i);
  }
}

void Si570::setBitvals(void){
  //set the rfreq values for each bit of the rfreq (integral)
  bitval[28] = (this->freq_xtal) / (hs * n1);
  bitval[29] = bitval[28] << 1;
  bitval[30] = bitval[29] << 1;
  bitval[31] = bitval[30] << 1;
  bitval[32] = bitval[31] << 1;
  bitval[33] = bitval[32] << 1;
  bitval[34] = bitval[33] << 1;
  bitval[35] = bitval[34] << 1;
  bitval[36] = bitval[35] << 1;
  bitval[37] = bitval[36] << 1;

  //set the rfreq values for each bit of the rfreq (integral)
  bitval[27] = bitval[28] >> 1;
  bitval[26] = bitval[27] >> 1;
  bitval[25] = bitval[26] >> 1;
  bitval[24] = bitval[25] >> 1;
  bitval[23] = bitval[24] >> 1;
  bitval[22] = bitval[23] >> 1;
  bitval[21] = bitval[22] >> 1;
  bitval[20] = bitval[21] >> 1;
  bitval[19] = bitval[20] >> 1;
  bitval[18] = bitval[19] >> 1;
  bitval[17] = bitval[18] >> 1;
  bitval[16] = bitval[17] >> 1;
  bitval[15] = bitval[16] >> 1;
  bitval[14] = bitval[15] >> 1;
  bitval[13] = bitval[14] >> 1;
  bitval[12] = bitval[13] >> 1;
  bitval[11] = bitval[12] >> 1;
  bitval[10] = bitval[11] >> 1;
  bitval[9] = bitval[10] >> 1;
  bitval[8] = bitval[9] >> 1;
  bitval[7] = bitval[8] >> 1;
  bitval[6] = bitval[7] >> 1;
  bitval[5] = bitval[6] >> 1;
  bitval[4] = bitval[5] >> 1;
  bitval[3] = bitval[4] >> 1;
  bitval[2] = bitval[3] >> 1;
  bitval[1] = bitval[2] >> 1;
  bitval[0] = bitval[1] >> 1;
}

//select reasonable dividers for a frequency
//in order to avoid overflow, the frequency is scaled by 10
void Si570::setDividers (unsigned long f){
  unsigned int i, j;
  unsigned long f_dco;

  for (i = 2; i <= 127; i+= 2) {
    for (j = 4; j <= 11; j++){
      //skip 8 and 10 as unused
      if (j == 8 || j == 10)
        continue;
      f_dco = (f/10) * i * j;
      if (480000000L < f_dco && f_dco < 560000000L){
        if (hs != j || n1 != i){
          hs = j; n1 = i;
          setBitvals();
        }
        //f_dco = fnew/10 * n1 * hs;
        return;
      }
    }
  }
}

void Si570::setRfreq (unsigned long fnew){
  int i, bit, ireg, byte;
  unsigned long rfreq;

  //reset all the registers
  for (i = 7; i <= 12; i++)
    this->dco_reg[i] = 0;

  //set up HS
  this->dco_reg[7] = (hs - 4) << 5;
  this->dco_reg[7] = this->dco_reg[7] | ((n1 - 1) >> 2);
  this->dco_reg[8] = ((n1-1) & 0x3) << 6;

  ireg = 8; //registers go from 8 to 12 (five of them)
  bit = 5; //the bits keep walking down
  byte = 0;
  rfreq = 0;
  for (i = 37; i >= 0; i--){
    //skip if the bitvalue is set to zero, it means, we have hit the bottom of the bitval table
    if (bitval[i] == 0)
      break;

    if (fnew >= bitval[i]){
      fnew = fnew - bitval[i];
      byte = byte | (1 << bit);
    }
    //else{
    // putchar('0');
    //}

    bit--;
    if (bit < 0){
      bit = 7;
      //use OR instead of = as register[7] has N1 bits already set into it
      this->dco_reg[ireg] |= byte;
      byte = 0;
      ireg++;
    }
  }
}

Si570_Status Si570::setFrequency(unsigned long newfreq) {
  //check that we are not wasting our time here
  if (this->frequency == newfreq)
    return this->status;

  //if the jump is small enough, we don't have to fiddle with the dividers
  if (abs(this->f_center - newfreq) < 50000L) {
    setRfreq(newfreq);
    this->frequency = newfreq;
    qwrite_si570();
    this->status = SUCCESS_SMALLJUMP;
  }
  else {
    //else it is a big jump
    setDividers(newfreq);
    setRfreq(newfreq);
    this->f_center = this->frequency = newfreq;
    write_si570();
    this->status = SUCCESS_BIGJUMP;
  }
  return this->status;
}
