#include <LiquidCrystal.h>
#include "Wire.h"
#include <avr/io.h> 
#include <stdlib.h> 


/*
 The 16x2 LCD is connected as follows:
    LCD's PIN   Raduino's PIN  PURPOSE      ATMEGA328's PIN
    4           13             Reset LCD    19
    6           12             Enable       18
    11          10             D4           17
    12          11             D5           16
    13           9             D6           15
    14           8             D7           14 
   
*/

/* si570 registers and stuff */
//#define FREQ_XTAL (114 278 453) // my si570's crystal was at 114.78453 MHz
//#define FREQ_XTAL (114 228 153L)

//lvds
//#define FREQ_XTAL (114285267)
//cmos si570
//#define FREQ_XTAL (1142121701) 

#define FREQ_XTAL (114292532l)
#define IF_FREQ   (19997000l) //this is for usb, we should probably have the USB and LSB frequencies separately
#define CW_TIMEOUT (600l) // in milliseconds, this is the parameter that determines how long the tx will hold between cw key downs

unsigned char si570_i2c_address = 0x55;
unsigned char dco_reg[13], dco_status='s';
unsigned long bitval[38];
unsigned long f_center=0, frequency=14200000, dco_freq=0;
unsigned int hs, n1;
unsigned long vfoA=14200000L, vfoB=14200000L, ritA, ritB;
unsigned char wasSmall = 1;
unsigned long cwTimeout = 0;


LiquidCrystal lcd(13, 12, 11, 10, 9, 8);
int count = 0;
char b[20], c[20], printBuff[32];

/* tuning pot stuff */
unsigned char refreshDisplay = 0;
unsigned int stepSize = 100;
int tuningPosition = 0;
unsigned char locked = 0; //the tuning can be locked: wait until it goes into dead-zone before unlocking it

/* the digital controls */

#define LSB (2)
#define TX_RX (3)
#define CW_KEY (4)

#define BAND_HI (5)

#define FBUTTON (A3) 
#define ANALOG_TUNING (A2)
#define ANALOG_KEYER (A1)


#define VFO_A 0
#define VFO_B 1

char inTx = 0;
char keyDown = 0;
char isLSB = 0;
char isRIT = 0;
char vfoActive = VFO_A;
/* si570 related routines */
/* modes */
unsigned char isManual = 1;
unsigned ritOn = 0;

/* dds ddschip(DDS9850, 5, 6, 7, 125000000LL); */

/* display routines */
void printLine1(char *c){
  if (strcmp(c, printBuff)){
    lcd.setCursor(0, 0);
    lcd.print(c);
    strcpy(printBuff, c);
    count++;
  }
}

void printLine2(char *c){
  lcd.setCursor(0, 1);
  lcd.print(c);
}

void displayFrequency(unsigned long f){
  int mhz, khz, hz;
  
  mhz = f / 1000000l;
  khz = (f % 1000000l)/1000;
  hz = f % 1000l;
  sprintf(b, "[%02d.%03d.%03d]", mhz, khz, hz);
  printLine1(b);
}

void updateDisplay(){
    sprintf(b, "%08ld", frequency);      
    sprintf(c, "%s:%.2s.%.4s %s", vfoActive == VFO_A ? " A" : " B" , b,  b+2, ritOn ? "+RX" : "   ");       
    printLine1(c);
    sprintf(c, "%s %s %d", isLSB ? "LSB" : "USB", inTx ? " TX" : " RX",  wasSmall);
    printLine2(c);
}


/* 
IMPORTANT, the wire.h is modified so that the internal pull up resisters are not enabled. 
This is required to interface Arduino with the 3.3v Si570's I2C interface.
routines to interface with si570 via i2c (clock on analog pin 5, data on analog pin 4) */

void i2c_write (char slave_address,char reg_address, char data )  {
  int rdata = data;
  Wire.beginTransmission(slave_address);
  Wire.write(reg_address);
  Wire.write(rdata);
  Wire.endTransmission();
}

char i2c_read ( char slave_address, int reg_address ) {
  unsigned char rdata = 0xFF;
  Wire.beginTransmission(slave_address);
  Wire.write(reg_address);
  Wire.endTransmission();
  Wire.requestFrom(slave_address,1);
  if (Wire.available()) rdata = Wire.read();
  return rdata;
}

void read_si570(){
  //we have to read eight consecutive registers starting at register 5
  for (int i = 7; i <= 12; i++) 
    dco_reg[i] = i2c_read( si570_i2c_address, i);
}

void write_si570()
{
  int idco, i;
  
  // Freeze DCO
  idco = i2c_read( si570_i2c_address,137);
  i2c_write(si570_i2c_address, 137, idco | 0x10 );
  	
  i2c_write(si570_i2c_address, 7, dco_reg[7]);
  
  //Set Registers
  for( i=7; i <= 12; i++){
    i2c_write(si570_i2c_address, i, dco_reg[i]);
    idco = i2c_read( si570_i2c_address, i);
  }

  // Unfreeze DCO
  idco = i2c_read( si570_i2c_address, 137 );
  i2c_write (si570_i2c_address, 137, idco & 0xEF );
  
  // Set new freq
  i2c_write(si570_i2c_address,135,0x40);        
}

void qwrite_si570()
{   
  int i, idco;
  //Set Registers
  for( i=8; i <= 12; i++){
    i2c_write(si570_i2c_address, i, dco_reg[i]);
    idco = i2c_read( si570_i2c_address, i);
  }
}

void setBitvals(void){

	//set the rfreq values for each bit of the rfreq (integral)
	bitval[28] = (FREQ_XTAL) / (hs * n1);
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
void setDividers (unsigned long f){
  int i, j;
  unsigned long f_dco;
  
  for (i = 2; i <= 127; i+= 2)
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

void setRfreq (unsigned long fnew){
  int i, bit, ireg, byte;
  unsigned long rfreq;

  //reset all the registers
  for (i = 7; i <= 12; i++)
    dco_reg[i] = 0;

  //set up HS
  dco_reg[7] = (hs - 4) << 5;
  dco_reg[7] = dco_reg[7] | ((n1 - 1) >> 2);
  dco_reg[8] = ((n1-1) & 0x3) << 6;

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
      dco_reg[ireg] |= byte;
      byte = 0;
      ireg++;
    }
  }
}

void setDCO(unsigned long newfreq){
  
  //check that we are not wasting our time here
  if (dco_freq == newfreq)
    return;
  
  //if the jump is small enough, we don't have to fiddle with the dividers
  if ((newfreq > f_center && newfreq - f_center < 50000L) ||
    (f_center > newfreq && f_center - newfreq < 50000L)){
    setRfreq(newfreq);
    dco_freq = newfreq;
    qwrite_si570();
    wasSmall = 1;
    return;
  }
  //else it is a big jump
  setDividers(newfreq);
  setRfreq(newfreq);
  f_center = dco_freq = newfreq;
  write_si570();
  wasSmall = 0;
}

void setup() {                
  lcd.begin(16, 2);
  printBuff[0] = 0;
  printLine1("Raduino v0.02 "); 

  Wire.begin();
  // Disable internal pullups
  digitalWrite(SDA, 0);
  digitalWrite(SCL, 0);

  // Force Si570 to reset to initial freq
  i2c_write(si570_i2c_address,135,0x01);
  delay(20);
  read_si570();


  sprintf(c, "%02x %02x %02x ", dco_reg[7], dco_reg[8], dco_reg[9]);
  printLine1(c);
 
  sprintf(c, "%02x %02x %02x ", dco_reg[10], dco_reg[11], dco_reg[12]);
  printLine2(c);

  //set the initial frequency
  setDCO(26150000L);

  //set up the pins
  pinMode(LSB, OUTPUT);
  pinMode(TX_RX, INPUT);
  pinMode(CW_KEY, OUTPUT);
  
  //set the side-tone off, put the transceiver to receive mode
  digitalWrite(CW_KEY, 0);
  digitalWrite(TX_RX, 1); //old way to enable the built-in pull-ups
}

void setSideband(){
  if (frequency >= 10000000L)
  {
    isLSB = 0;
    digitalWrite(LSB, 0);
  }
  else{
    digitalWrite(LSB, 1);
    isLSB = 1;
  }
}

void setBandswitch(){
  if (frequency >= 15000000L)
  {
    digitalWrite(BAND_HI, 1);
  }
  else {
    digitalWrite(BAND_HI, 0);
  }
}

void readTuningPot(){
    tuningPosition = analogRead(2) - 512;
}

void checkTuning(){
    
  if (-50 < tuningPosition && tuningPosition < 50){
    //we are in the middle, so, let go of the lock
    if (locked)
      locked = 0;
    delay(50);
    return;
  }

  //if the tuning is locked and we are outside the safe band, then we don't move the freq.
  if (locked)
    return;
    
  //dead region between -50 and 50
  if (100 < tuningPosition){
    if (tuningPosition < 100)
      frequency += 3;
    else if (tuningPosition < 150)
      frequency += 10;
    else if (tuningPosition < 200)
      frequency += 30;
    else if (tuningPosition < 250)
      frequency += 100;
    else if (tuningPosition < 300)
      frequency += 300;
    else if (tuningPosition < 350)
      frequency += 1000;
    else if (tuningPosition < 400)
      frequency += 3000;   
    else if (tuningPosition < 450){
      frequency += 100000;   
      updateDisplay();
      delay(300);
    }
    else if (tuningPosition < 500){
      frequency += 1000000;   
      updateDisplay();
      delay(300);
    }
  }

  if (-100 > tuningPosition){
    if (tuningPosition > -100)
      frequency -= 3;
    else if (tuningPosition > -150)
      frequency -= 10;
    else if (tuningPosition > -200)
      frequency -= 30;
    else if (tuningPosition > -250)
      frequency -= 100;
    else if (tuningPosition > -300)
      frequency -= 300;
    else if (tuningPosition > -350)
      frequency -= 1000;
    else if (tuningPosition > -400)
      frequency -= 3000;   
    else if (tuningPosition > -450){
      frequency -= 100000;   
      updateDisplay();
      delay(300);
    }
    else if (tuningPosition > -500){
      frequency -= 1000000;   
      updateDisplay();
      delay(300);
    }
  }
  delay(50);  
  refreshDisplay++;
}


void checkTX(){
	
  //we don't check for ptt when transmitting cw
  if (cwTimeout > 0)
    return;
    
  if (digitalRead(TX_RX) == 0 && inTx == 0){
    refreshDisplay++;
    inTx = 1;
  }
	
  if (digitalRead(TX_RX) == 1 && inTx == 1){
    refreshDisplay++;
    inTx = 0;
  }
}

 
/*CW is generated by keying the bias of a side-tone oscillator.
nonzero cwTimeout denotes that we are in cw transmit mode.
*/

void checkCW(){

  if (keyDown == 0 && analogRead(ANALOG_KEYER) < 50){
    //switch to transmit mode if we are not already in it
    if (inTx == 0){
      //put the  TX_RX line to transmit
      pinMode(TX_RX, OUTPUT);
      digitalWrite(TX_RX, 0);
      //give the relays a few ms to settle the T/R relays
      delay(50);
    }
    inTx = 1;
    keyDown = 1;
    digitalWrite(CW_KEY, 1); //start the side-tone
    refreshDisplay++;
  }

  //reset the timer as long as the key is down
  if (keyDown == 1){
     cwTimeout = CW_TIMEOUT + millis();
  }
  
  //if we have a keyup
  if (keyDown == 1 && analogRead(ANALOG_KEYER) > 150){
    keyDown = 0;
    digitalWrite(CW_KEY, 0);
    cwTimeout = millis() + CW_TIMEOUT;
  }
  
  //if we have keyuup for a longish time while in cw tx mode
  if (inTx == 1 && cwTimeout < millis()){
    //move the radio back to receive
    digitalWrite(TX_RX, 1);
    //set the TX_RX pin back to input mode
    pinMode(TX_RX, INPUT);
    digitalWrite(TX_RX, 1); //pull-up!
    inTx = 0;
    cwTimeout = 0;
    refreshDisplay++;
  }
}

int btnDown(){
  if (analogRead(FBUTTON) < 300)
    return 1;
  else
    return 0;
}

void checkButton(){
  int i, t1, t2;
  //only if the button is pressed
  if (!btnDown())
    return;

  //if the btn is down while tuning pot is not centered, then lock the tuning
  //and return
  if (tuningPosition < -50 || tuningPosition > 50){
    if (locked)
      locked = 0;
    else
      locked = 1;
    delay(200);
    return;
  }
  
  t1 = t2 = i = 0;

  while (t1 < 30 && btnDown() == 1){
    delay(50);
    t1++;
  }
   
  while (t2 < 10 && btnDown() == 0){
    delay(50);
    t2++;
  }
  
  //if the press is momentary and there is no secondary press
  if (t1 < 10 && t2 > 6){
    if (ritOn)
      ritOn = 0;
    else
      ritOn = 1;
    printLine2("RIT on! ");
    refreshDisplay++;
  }
  //there has been a double press
  else if (t1 < 10 && t2 <= 6) {
    if (vfoActive == VFO_B){
      vfoActive = VFO_A;
      vfoB = frequency;
      frequency = vfoA;
    }
    else{
      vfoActive = VFO_B;
      vfoA = frequency;
      frequency = vfoB;
    }
     refreshDisplay++;
     printLine2("VFO swap! ");
  } 
  else if (t1 > 10){
    printLine2("VFOs reset!");
    vfoA= vfoB = frequency;
    refreshDisplay++;
  }

//  sprintf(c, "t1=%d, t2=%d ", t1, t2);
//  printLine2(c);
  
  while (btnDown() == 1){
     delay(50);
  }
}

void loop(){
  int t = 0;

  readTuningPot();
  checkTuning();

  //the order of testing first for cw and then for ptt is important.
  checkCW();
  checkTX();
  checkButton();

  setDCO(frequency+ IF_FREQ);
  setSideband();  
  setBandswitch();
  
  if (refreshDisplay){
    updateDisplay();
    refreshDisplay = 0;
//    delay(10);
  }  
}

