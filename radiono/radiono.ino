/*
 * Radiono - The Minima's Main Arduino Sketch
 * Copyright (C) 2013 Ashar Farhan
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of  MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 *
 * Modified:
 * An Alternate Tuning Method by: Eldon R. Brown (ERB) - WA0UWH, Apr 25, 2014
 *
 */


/*
 * A Long line, to allow for left right scrole, for program construct alignment --------------------------------------------------------------------
 */

#define __ASSERT_USE_STDERR
#include <assert.h>

/*
 * Wire is only used from the Si570 module but we need to list it here so that
 * the Arduino environment knows we need it.
 */
#include <Wire.h>
#include <LiquidCrystal.h>
#define LCD_COL (16)
#define LCD_ROW (2)
#define LCD_STR_CLEAR2END "%-16.16s"
//#define LCD_STR_CLEAR2END "%-20.20s"  // For 20 Character LCD Display


#include <avr/pgmspace.h>
#include <avr/io.h>
#include "Si570.h"
#include "debug.h"

//#define RADIONO_VERSION "0.4"
#define RADIONO_VERSION "0.4.erb" // Modifications by: Eldon R. Brown - WA0UWH

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

#define SI570_I2C_ADDRESS 0x55
//#define IF_FREQ   (0)  // FOR DEBUG ONLY
#define IF_FREQ   (19997000L) //this is for usb, we should probably have the USB and LSB frequencies separately
#define CW_TIMEOUT (600L) // in milliseconds, this is the parameter that determines how long the tx will hold between cw key downs

// When RUN_TESTS is 1, the Radiono will automatically do some software testing when it starts.
// Please note, that those are not hardware tests! - Comment this line to save some space.
//#define RUN_TESTS 1

unsigned long frequency = 14285000UL; //  20m - QRP SSB Calling Freq
unsigned long vfoA = frequency, vfoB = frequency, ritA, ritB;
unsigned long cwTimeout = 0;

Si570 *vfo;
LiquidCrystal lcd(13, 12, 11, 10, 9, 8);

char b[22], c[22];  // General Buffers, used mostly for Formating message for LCD

/* tuning pot stuff */
unsigned char refreshDisplay = 0;

// Added by ERB
#define MAX_FREQ (30000000UL)

#define DEAD_ZONE (40)

#define CURSOR_MODE (1)
#define DIGIT_MODE (2)

#define NO_CURSOR (0)
#define UNDERLINE (1)
#define UNDERLINE_ WINK (2)
#define BLOCK_BLINK (3)

#define MOMENTARY_PRESS (1)
#define DOUBLE_PRESS (2)
#define LONG_PRESS (3)

#define AUTO_SIDEBAND_MODE (0)
#define UPPER_SIDEBAND_MODE (1)
#define LOWER_SIDEBAND_MODE (2)

int tuningDir = 0;
int tuningPosition = 0;
int freqUnStable = 1;
int tuningPositionDelta = 0;
int cursorDigitPosition = 0;
int tuningPositionPrevious = 0;
int cursorCol, cursorRow, cursorMode;
int winkOn;
unsigned long freqPrevious;
char* const sideBandText[] PROGMEM = {"Auto SB","USB","LSB"};
int sideBandMode = 0;

// ERB - Buffers that Stores "const stings" to, and Reads from FLASH Memory
char buf[60];
// ERB - Force format stings into FLASH Memory
#define  FLASH(x) strcpy_P(buf, PSTR(x))
// FLASH2 can be used where Two small (1/2 size) Buffers are needed.
#define FLASH2(x) strcpy_P(buf + sizeof(buf)/2, PSTR(x))


// PROGMEM is used to avoid using the small available variable space
prog_uint32_t bands[] PROGMEM = {  // Lower and Upper Band Limits
      1800000UL,  2000000UL, // 160m
      3500000UL,  4000000UL, //  80m
      7000000UL,  7300000UL, //  40m
     10100000UL, 10150000UL, //  30m
     14000000UL, 14350000UL, //  20m
     18068000UL, 18168000UL, //  17m
     21000000UL, 21450000UL, //  15m
     24890000UL, 24990000UL, //  12m
     28000000UL, 29700000UL, //  10m
   };
   
#define BANDS (sizeof(bands)/sizeof(prog_uint32_t)/2)

// End ERB add


unsigned char locked = 0; //the tuning can be locked: wait until it goes into dead-zone before unlocking it

/* the digital controls */

#define LSB (2)
#define TX_RX (3)
#define CW_KEY (4)

#define BAND_HI (5)
#define PA_BAND_CLK (7)

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

/* modes */
unsigned char isManual = 1;
unsigned ritOn = 0;

/* dds ddschip(DDS9850, 5, 6, 7, 125000000LL); */

// ###############################################################################
// ###############################################################################
// ###############################################################################
// ###############################################################################

// ###############################################################################
/* display routines */
void printLine1(char const *c){
    lcd.setCursor(0, 0);
    lcd.print(c);
}

void printLine2(char const *c){
    lcd.setCursor(0, 1);
    lcd.print(c);
}

// Print LCD Line1 with Clear to End
void printLine1cle(char const *c){
    char buf[LCD_COL+2];
    sprintf(buf, LCD_STR_CLEAR2END, c);
    printLine1(buf);
}

// Print LCD Line2 with Clear to End
void printLine2cle(char const *c){
    char buf[LCD_COL+2];
    sprintf(buf, LCD_STR_CLEAR2END, c);
    printLine2(buf);
}


// ###############################################################################
void displayFrequency(unsigned long f){
  int mhz, khz, hz;

  mhz = f / 1000000l;
  khz = (f % 1000000l)/1000;
  hz = f % 1000l;
  sprintf(b, FLASH("[%02d.%03d.%03d]"), mhz, khz, hz);
  printLine1(b);
}


void updateDisplay(){
  char const *vfoStatus[] = { "ERR", "RDY", "BIG", "SML" };
   
  if (refreshDisplay) {
      refreshDisplay = false;
      cursorOff();
        
      // Top Line of LCD    
      sprintf(b, FLASH("%08ld"), frequency);
      sprintf(c, FLASH("%1s:%.2s.%.6s %3.3s"),
          vfoActive == VFO_A ? "A" : "B" ,
          b,  b+2,
          ritOn ? "RIT" : " ");
      printLine1cle(c);
      
      sprintf(c, FLASH("%3s%1s %2s %3.3s"),
          isLSB ? "LSB" : "USB",
          sideBandMode > 0 ? "*" : " ",
          inTx ? "TX" : "RX",
          freqUnStable ? " " : vfoStatus[vfo->status]);
      printLine2cle(c);
      
      
      setCursorCRM(11 - (cursorDigitPosition + (cursorDigitPosition>6) ), 0, CURSOR_MODE);
  }
  updateCursor();
}


// -------------------------------------------------------------------------------
void setCursorCRM(int col, int row, int mode) {
  // mode 0 = underline
  // mode 1 = underline wink
  // move 2 = block blink
  // else   = no cursor
  cursorCol = col;
  cursorRow = row;
  cursorMode = mode;
}


// -------------------------------------------------------------------------------
void cursorOff() {
  lcd.noBlink();
  lcd.noCursor();
}


// -------------------------------------------------------------------------------
void updateCursor() {
  
  lcd.setCursor(cursorCol, cursorRow); // Postion Curesor
  
  // Set Cursor Display Mode, Wink On and OFF for DigitMode, Solid for CursorMode
  if (cursorMode == CURSOR_MODE) {
      if (millis() & 0x0200 ) { // Update only once in a while
        lcd.noBlink();
        lcd.cursor();
      }
  }
  else if (cursorMode == DIGIT_MODE) { // Winks Underline Cursor
      if (millis() & 0x0200 ) { // Update only once in a while
        if (winkOn == false) {
          lcd.cursor();
          winkOn = true;
        }
      } else {
        if (winkOn == true) {
          lcd.noCursor();
          winkOn = false;
        }
      }
  }
  else if (cursorMode == BLOCK_BLINK) {
      if (millis() & 0x0200 ) { // Update only once in a while
        lcd.blink();
        lcd.noCursor();
      }
  }
  else {
      if (millis() & 0x0200 ) { // Update only once in a while
        cursorOff();
      }
  }
 
}


// ###############################################################################
void setSideband(){
  
  switch(sideBandMode) {
    case AUTO_SIDEBAND_MODE: // Automatic Side Band Mode
      isLSB = (frequency < 10000000UL) ? 1 : 0 ; break;
    case UPPER_SIDEBAND_MODE: // Force USB Mode
      isLSB = 0; break;
    case LOWER_SIDEBAND_MODE: // Force LSB Mode
      isLSB = 1; break;
  } 
  digitalWrite(LSB, isLSB);
}


// ###############################################################################
void setRf386BandSignal(){
  // This setup is compatable with the Minima RF386 RF Power Amplifier
  // See: http://www.hfsignals.org/index.php/RF386

  // Bitbang Clock Pulses to Change PA Band Filter
  int band;
  static int prevBand;
  static unsigned long prevFrequency;

  if (frequency == prevFrequency) return;
  prevFrequency = frequency;
  
  band = freq2Band();

  //debug("Band Index = %d", band);
  
  if (band == prevBand) return;
  prevBand = band;
  
  
  debug(FLASH("BandI = %d"), band);

  digitalWrite(PA_BAND_CLK, 1);  // Output Reset Pulse for PA Band Filter
  delay(500);
  digitalWrite(PA_BAND_CLK, 0);

  while (band-- > 1) { // Output Clock Pulse to Change PA Band Filter
     delay(50);
     digitalWrite(PA_BAND_CLK, 1);
     delay(50);
     digitalWrite(PA_BAND_CLK, 0);
  }
}

// -------------------------------------------------------------------------------
int freq2Band(){
  
  //debug(FLASH("Freq = %lu"), frequency);
  
  if (frequency <  4000000UL) return 4; //   3.5 MHz
  if (frequency < 10200000UL) return 3; //  7-10 MHz
  if (frequency < 18200000UL) return 2; // 14-18 MHz
  if (frequency < 30000000UL) return 1; // 21-28 MHz
  return 1;
}


// ###############################################################################
void setBandswitch(){
  
  if (frequency >= 15000000UL) {
    digitalWrite(BAND_HI, 1);
  }
  else {
    digitalWrite(BAND_HI, 0);
  }
}

// ###############################################################################
void readTuningPot(){
    tuningPosition = analogRead(ANALOG_TUNING);
}



// ###############################################################################
// An Alternate Tuning Strategy or Method
// This method somewhat emulates a normal Radio Tuning Dial
// Tuning Position by Switches on FN Circuit
// Author: Eldon R. Brown - WA0UWH, Apr 25, 2014
void checkTuning() {
  long deltaFreq;
  unsigned long newFreq;
  
  if (!freqUnStable) {
    //we are Stable, so, Set to Non-lock
      locked = 0;
  }
  // Count Down to Freq Stable, i.e. Freq has not changed recently
  if (freqUnStable == 1) refreshDisplay = true;
  freqUnStable = max(--freqUnStable, 0);
  
  // Compute tuningDaltaPosition from tuningPosition
  tuningPositionDelta = tuningPosition - tuningPositionPrevious;
  
  tuningDir = 0;  // Set Default Tuning Directon to neather Right nor Left

  // Check to see if Automatic Digit Change Action is Required, if SO, force the change
  if (tuningPosition < DEAD_ZONE * 2) { // We must be at the Low end of the Tuning POT
    tuningPositionDelta = -DEAD_ZONE;
    delay(100);
    if (tuningPosition > DEAD_ZONE ) delay(100);
  }
  if (tuningPosition > 1023 - DEAD_ZONE * 2) { // We must be at the High end of the Tuning POT
    tuningPositionDelta = DEAD_ZONE; 
    delay(100);
    if (tuningPosition < 1023 - DEAD_ZONE / 8) delay(100);
  }

  // Check to see if Digit Change Action is Required, Otherwise Do Nothing via RETURN 
  if (abs(tuningPositionDelta) < DEAD_ZONE) return;

  tuningDir = tuningPositionDelta < 0 ? -1 : tuningPositionDelta > 0 ? +1 : 0;  
  if (!tuningDir) return;  // If Neather Direction, Abort
  
  if (cursorDigitPosition < 1) return; // Nothing to do here, Abort, Cursor is in Park position

  // Compute deltaFreq based on current Cursor Position Digit
  deltaFreq = tuningDir;
  for (int i = cursorDigitPosition; i > 1; i-- ) deltaFreq *= 10;
  
  newFreq = freqPrevious + deltaFreq;  // Save Least Digits Mode
  //newFreq = (freqPrevious / abs(deltaFreq)) * abs(deltaFreq) + deltaFreq; // Zero Lesser Digits Mode 
  if (newFreq != frequency) {
      // Update frequency if within range of limits, 
      // Avoiding Nagative underRoll of UnSigned Long, and over-run MAX_FREQ  
      if (newFreq <= MAX_FREQ) {
        frequency = newFreq;  
        refreshDisplay = true;
      }
      freqUnStable = 25; // Set to UnStable (non-zero) Because Freq has been changed
      tuningPositionPrevious = tuningPosition; // Set up for the next Iteration
  }
  freqPrevious = frequency;
}


// ###############################################################################
void checkTX(){
  
  if (freqUnStable) return;

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


// ###############################################################################
void checkCW(){
/* CW is generated by keying the bias of a side-tone oscillator.
 *   nonzero cwTimeout denotes that we are in cw transmit mode.
 */

  if (freqUnStable) return;

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




// ###############################################################################
int btnDown(){
  int val = -1, val2 = -2;
  
  val = analogRead(FBUTTON);
  while (val != val2) { // DeBounce Button Press
    delay(10);
    val2 = val;
    val = analogRead(FBUTTON);
  }
  //debug("Val= %d", val);
  if (val>1000) return 0;
  // 47K Pull-up, and 4.7K switch resistors,
  // Val should be approximately = (btnN×4700)÷(47000+4700)×1023
  //sprintf(c,"Val= %d            ", val); printLine2(c); delay(1000);  // For Debug Only
  
  debug(FLASH("btn Val= %d"), val);
  
  if (val > 350) return 7;
  if (val > 300) return 6;
  if (val > 250) return 5;
  if (val > 200) return 4;
  if (val > 150) return 3;
  if (val >  50) return 2;
  return 1;
}

// -------------------------------------------------------------------------------
void deDounceBtnRelease() {
  int i = 2;
  
    while (--i) { // DeBounce Button Release, Check twice
      while (btnDown()) delay(20);
    }
    // The following allows the user to re-center the
    // Tuning POT during any Key Press-n-hold without changing Freq.
    readTuningPot();
    tuningPositionPrevious = tuningPosition;
}

// ###############################################################################
void checkButton() {
  int btn;
  btn = btnDown();
  if (btn) debug(FLASH("btn %d"), btn);
  
  switch (btn) {
    case 0: return; // Abort
    case 1: decodeFN(btn); break;  
    case 2: decodeMoveCursor(btn); break;    
    case 3: decodeMoveCursor(btn); break;
    case 4: decodeSideBandMode(btn); break;
    case 5: decodeBandUpDown(+1); break; // Band Up
    case 6: decodeBandUpDown(-1); break; // Band Down
    case 7: decodeAux(btn); break; // Report Un-Used as AUX Buttons
    default: return;
  }
}


// ###############################################################################
void decodeBandUpDown(int dir) {
  static unsigned long freqCache[] = { // Set Default Values for Cache
      1825000UL, // 160m - QRP SSB Calling Freq
      3985000UL, //  80m - QRP SSB Calling Freq
      7285000UL, //  40m - QRP SSB Calling Freq
     10138700UL, //  30m - QRP QRSS and WSPR
     14285000UL, //  20m - QRP SSB Calling Freq
     18130000UL, //  17m - QRP SSB Calling Freq
     21385000UL, //  15m - QRP SSB Calling Freq
     24950000UL, //  12m - QRP SSB Calling Freq
     28385000UL, //  10m - QRP SSB Calling Freq
   };
  
   static byte sideBandModeCache[BANDS];
   
   switch (dir) {  // Decode Direction of Band Change
     
     case +1:  // For Band Change, Up
       for (int i = 0; i < BANDS; i++) {
         if (frequency <= pgm_read_dword(&bands[i*2+1])) {
           if (frequency >= pgm_read_dword(&bands[i*2])) {
             // Save Current Ham frequency and sideBandMode
             freqCache[i] = frequency;
             sideBandModeCache[i] = sideBandMode;
             i++;
           }
           // Load From Next Cache
           frequency = freqCache[min(i,BANDS-1)];
           sideBandMode = sideBandModeCache[min(i,BANDS-1)];
           freqPrevious = frequency;
           break;
         }
       }
       break;
     
     case -1:  // For Band Change, Down
       for (int i = BANDS-1; i > 0; i--) {
         if (frequency >= pgm_read_dword(&bands[i*2])) {
           if (frequency <= pgm_read_dword(&bands[i*2+1])) {
             // Save Current Ham frequency and sideBandMode
             freqCache[i] = frequency;
             sideBandModeCache[i] = sideBandMode;
             i--;
           }
           frequency = freqCache[max(i,0)];
           sideBandMode = sideBandModeCache[max(i,0)];
           freqPrevious = frequency;
           break;
         }
       }
       break;
     
   }
   
   freqUnStable = 25; // Set to UnStable (non-zero) Because Freq has been changed
   ritOn = 0;
   setSideband();
   refreshDisplay++;
   updateDisplay();
   deDounceBtnRelease(); // Wait for Release
   refreshDisplay++;
}


// ###############################################################################
void decodeSideBandMode(int btn) {
  sideBandMode++;
  sideBandMode %= 3; // Limit to Three Modes
  setSideband();
  cursorOff();
  //sprintf(c, LCD_STR_CLEAR2END, (char *)pgm_read_word(&sideBandText[sideBandMode]));
  //printLine2(c);
  printLine2cle((char *)pgm_read_word(&sideBandText[sideBandMode]));
  deDounceBtnRelease(); // Wait for Release
  refreshDisplay++;
  updateDisplay();
  
}
// ###############################################################################
void decodeMoveCursor(int btn) {
  
      tuningPositionPrevious = tuningPosition;
      switch (btn) {
        case 2: cursorDigitPosition++; break;
        case 3: cursorDigitPosition--; break;
      }
      cursorDigitPosition = min(cursorDigitPosition, 7);
      cursorDigitPosition = max(cursorDigitPosition, 0);
      freqPrevious = frequency;
      freqUnStable = false;  // Set Freq is NOT UnStable, as it is Stable
      refreshDisplay++;
      updateDisplay();
      deDounceBtnRelease(); // Wait for Button Release
}

void decodeAux(int btn) {
  
  //debug("Aux %d", btn);
  cursorOff();
  sprintf(c, FLASH("Btn: %.2d%9s"), btn, " ");
  printLine2(c);
  deDounceBtnRelease(); // Wait for Button Release
  refreshDisplay++;
  updateDisplay();
}

// ###############################################################################
int getButtonPushMode(int btn) {
  int i, t1, t2;
  
  t1 = t2 = i = 0;

  while (t1 < 30 && btnDown() == btn){
    delay(50);
    t1++;
  }
  while (t2 < 10 && !btnDown()){
    delay(50);
    t2++;
  }

  //if the press is momentary and there is no secondary press
  if (t1 < 10 && t2 > 6){
    return MOMENTARY_PRESS;
  }
  //there has been a double press
  else if (t1 < 10 && t2 <= 6) {
    return DOUBLE_PRESS;
  }
  //there has been a long press
  else if (t1 > 10){
    return LONG_PRESS;
  }
}



// ###############################################################################
void decodeFN(int btn) {
  //if the btn is down while tuning pot is not centered, then lock the tuning
  //and return
  
  // I don't understand this origninal code fragment
  if (freqUnStable) {
    if (locked)
      locked = 0;
    else
      locked = 1;
    return;
  }

  switch (getButtonPushMode(btn)) { 
    case MOMENTARY_PRESS:
      ritOn = !ritOn;
      break;
      
    case DOUBLE_PRESS:
      if (vfoActive == VFO_B) {
        vfoActive = VFO_A;
        vfoB = frequency;
        frequency = vfoA;
      }
      else {
        vfoActive = VFO_B;
        vfoA = frequency;
        frequency = vfoB;
      }
      ritOn = 0;
      refreshDisplay++;
      updateDisplay();
      cursorOff();
      
      //sprintf(c, LCD_STR_CLEAR2END, FLASH2("VFO swap!"));
      //printLine2(c);
      printLine2cle(FLASH("VFO swap!"));
      break;
      
    case LONG_PRESS:
      vfoA = vfoB = frequency;
      ritOn = 0;
      refreshDisplay++;
      updateDisplay();
      cursorOff();
      //sprintf(c, LCD_STR_CLEAR2END, FLASH2("VFO reset!"));
      //printLine2(c);
      printLine2cle(FLASH("VFO reset!"));
      break;
    default:
      return;
  }

  refreshDisplay++;
  deDounceBtnRelease(); // Wait for Button Release
}


// ###############################################################################
// ###############################################################################
// ###############################################################################
// ###############################################################################


// ###############################################################################
// ###############################################################################
void setup() {
  
  // Initialize the Serial port so that we can use it for debugging
  Serial.begin(115200);
  debug(FLASH("Radiono - Rev: %s"), RADIONO_VERSION);

  lcd.begin(LCD_COL, LCD_ROW);
  printLine1(FLASH("Farhan - Minima"));
  printLine2(FLASH("  Tranceiver"));
  delay(2000);
  
  sprintf(b, FLASH("Radiono %s"), RADIONO_VERSION);
  //sprintf(c, LCD_STR_CLEAR2END, b);
  //printLine1(c);
  printLine1cle(b);
  //sprintf(c, LCD_STR_CLEAR2END, FLASH2("Rev: CC.03"));
  //printLine2(c);
  printLine2cle(FLASH2("Rev: CC.02"));
  delay(2000);
  
  // Print just the File Name, Added by ERB
  //sprintf(c, FLASH("F: %-13.13s"), FLASH2(__FILE__));
  //printLine2(c);
  //delay(2000);
  
 
#ifdef RUN_TESTS
  run_tests();
#endif


  // The library automatically reads the factory calibration settings of your Si570
  // but it needs to know for what frequency it was calibrated for.
  // Looks like most HAM Si570 are calibrated for 56.320 Mhz.
  // If yours was calibrated for another frequency, you need to change that here
  vfo = new Si570(SI570_I2C_ADDRESS, 56320000);

  if (vfo->status == SI570_ERROR) {
    // The Si570 is unreachable. Show an error for 3 seconds and continue.
    printLine2(FLASH("Si570 comm error"));
    delay(3000);
  }
  
  //sprintf(c, LCD_STR_CLEAR2END, " ");
  //printLine2(c);
  printLine2cle(" ");
 
  // This will print some debugging info to the serial console.
  vfo->debugSi570();

  //set the initial frequency
  vfo->setFrequency(26150000L);

  //set up the pins
  pinMode(LSB, OUTPUT);
  pinMode(TX_RX, INPUT);
  pinMode(CW_KEY, OUTPUT);
  pinMode(ANALOG_TUNING, INPUT);
  pinMode(FBUTTON, INPUT);

  //set the side-tone off, put the transceiver to receive mode
  digitalWrite(CW_KEY, 0);
  digitalWrite(TX_RX, 1); //old way to enable the built-in pull-ups
  digitalWrite(ANALOG_TUNING, 1);
  digitalWrite(FBUTTON, 0); // Use an external pull-up of 47K ohm to AREF
  
  tuningPositionPrevious = tuningPosition = analogRead(ANALOG_TUNING);
  refreshDisplay = 1;
}


// ###############################################################################
// ###############################################################################
void loop(){
  
  readTuningPot();
  checkTuning();

  //the order of testing first for cw and then for ptt is important.
  checkCW();
  checkTX();
  checkButton();

  vfo->setFrequency(frequency + IF_FREQ);
  
  setSideband();
  setBandswitch();
  setRf386BandSignal();

  updateDisplay();
  
}

// ###############################################################################
// ###############################################################################
// ###############################################################################
// ###############################################################################


// ###############################################################################
// ###############################################################################
#ifdef RUN_TESTS

bool run_tests() {
  /* Those tests check that the Si570 libary is able to understand the
   * register values provided and do the required math with them.
   */
  // Testing for thomas - si570
  {
    uint8_t registers[] = { 0xe1, 0xc2, 0xb5, 0x7c, 0x77, 0x70 };
    vfo = new Si570(registers, 56320000);
    assert(vfo->getFreqXtal() == 114347712);
    delete(vfo);
  }

  // Testing Jerry - si570
  {
    uint8_t registers[] = { 0xe1, 0xc2, 0xb6, 0x36, 0xbf, 0x42 };
    vfo = new Si570(registers, 56320000);
    assert(vfo->getFreqXtal() == 114227856);
    delete(vfo);
  }

  Serial.println(FLASH("Tests successful!"));
  return true;
}

#endif

// ###############################################################################
// ###############################################################################
// handle diagnostic informations given by assertion and abort program execution:
void __assert(const char *__func, const char *__file, int __lineno, const char *__sexp) {
  debug(FLASH("ASSERT FAILED - %s (%s:%i): %s"), __func, __file, __lineno, __sexp);
  Serial.flush();
  // Show something on the screen
  lcd.setCursor(0, 0);
  lcd.print(FLASH("OOPS "));
  lcd.print(__file);
  lcd.setCursor(0, 1);
  lcd.print(FLASH("Line: "));
  lcd.print(__lineno);
  // abort program execution.
  abort();
}


