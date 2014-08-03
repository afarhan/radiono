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

//#define RADIONO_VERSION "0.4"
#define RADIONO_VERSION "0.4.erb" // Modifications by: Eldon R. Brown - WA0UWH
#define INC_REV "CD.02.02"           // Incremental Rev Code


/*
 * Wire is only used from the Si570 module but we need to list it here so that
 * the Arduino environment knows we need it.
 */
#include <Wire.h>
#include <LiquidCrystal.h>

#define LCD_COL (16)
#define LCD_ROW (2)
#define LCD_STR_CEL "%-16.16s"
//#define LCD_STR_CEL "%-20.20s"  // For 20 Character LCD Display


#include <avr/pgmspace.h>
#include <avr/io.h>
#include "Si570.h"
#include "debug.h"

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


Si570 *vfo;
LiquidCrystal lcd(13, 12, 11, 10, 9, 8);

unsigned long frequency = 14285000UL; //  20m - QRP SSB Calling Freq
unsigned long vfoA = frequency, vfoB = frequency, ritA, ritB;
unsigned long cwTimeout = 0;

char b[LCD_COL+6], c[LCD_COL+6];  // General Buffers, used mostly for Formating message for LCD

/* tuning pot stuff */
unsigned char refreshDisplay = 0;

int tuningDir = 0;
int tuningPosition = 0;
int tune2500Mode = 0;
int freqUnStable = 1;
int tuningPositionDelta = 0;
int cursorDigitPosition = 0;
int tuningPositionPrevious = 0;
int cursorCol, cursorRow, cursorMode;
int winkOn;
char* const sideBandText[] PROGMEM = {"Auto SB","USB","LSB"};
int sideBandMode = 0;

// ERB - Buffers that Stores "const stings" to, and Reads from FLASH Memory
char buf[64+2];
// ERB - Force format stings into FLASH Memory
#define  P(x) strcpy_P(buf, PSTR(x))
// FLASH2 can be used where Two small (1/2 size) Buffers are needed.
#define P2(x) strcpy_P(buf + sizeof(buf)/2, PSTR(x))


// PROGMEM is used to avoid using the small available variable space
const prog_uint32_t bandLimits[] PROGMEM = {  // Lower and Upper Band Limits
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
   
#define BANDS (sizeof(bandLimits)/sizeof(prog_uint32_t)/2)


unsigned char locked = 0; //the tuning can be locked: wait until Freq Stable before unlocking it
char inTx = 0;
char keyDown = 0;
char isLSB = 0;
char vfoActive = VFO_A;

/* modes */
unsigned char isManual = 1;
unsigned ritOn = 0;
int ritVal = 0;

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

// Print LCD Line1 with Clear to End of Line
void printLine1CEL(char const *c){
    char lbuf[LCD_COL+2];
    sprintf(lbuf, LCD_STR_CEL, c);
    printLine1(lbuf);
}

// Print LCD Line2 with Clear to End of Line
void printLine2CEL(char const *c){
    char lbuf[LCD_COL+2];
    sprintf(lbuf, LCD_STR_CEL, c);
    printLine2(lbuf);
}


// ###############################################################################
void updateDisplay(){
  char const *vfoStatus[] = { "ERR", "RDY", "BIG", "SML" };
  char d[6]; // Buffer for RIT Display Value
  
  if (refreshDisplay) {
      refreshDisplay = false;
      cursorOff();
        
      // Top Line of LCD
      sprintf(d, P("%+03.3d"), ritVal);  
      sprintf(b, P("%08ld"), frequency);
      sprintf(c, P("%1s:%.2s.%.6s%4.4s%s"),
          vfoActive == VFO_A ? "A" : "B" ,
          b,  b+2,
          inTx ? " " : ritOn ? d : " ",
          tune2500Mode ? "*": " "
          );
      printLine1CEL(c);
      

      sprintf(c, P("%3s%1s %2s %3.3s"),
          isLSB ? "LSB" : "USB",
          sideBandMode > 0 ? "*" : " ",
          inTx ? "TX" : "RX",
          freqUnStable ? " " : vfoStatus[vfo->status]
          );
      printLine2CEL(c);
      
      
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
void setRf386BandSignal(unsigned long freq){
  // This setup is compatable with the Minima RF386 RF Power Amplifier
  // See: http://www.hfsignals.org/index.php/RF386

  // Bitbang Clock Pulses to Change PA Band Filter
  int band;
  static int prevBand;
  static unsigned long prevFreq;

  if (freq == prevFreq) return;
  prevFreq = freq;
   
  if      (freq <  4000000UL) band = 4; //   3.5 MHz
  else if (freq < 10200000UL) band = 3; //  7-10 MHz
  else if (freq < 18200000UL) band = 2; // 14-18 MHz
  else if (freq < 30000000UL) band = 1; // 21-28 MHz
  else band = 1;

  //debug("Band Index = %d", band);
  
  if (band == prevBand) return;
  prevBand = band;
  
  debug(P("BandI = %d"), band);

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


// ###############################################################################
void setBandswitch(unsigned long freq){
  
  if (freq >= 15000000UL) {
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
  
  // Decode and implement RIT Tuning
  if (ritOn) {
      ritVal += tuningDir * 10;
      ritVal = max(ritVal, -990);
      ritVal = min(ritVal, 990);
      tuningPositionPrevious = tuningPosition; // Set up for the next Iteration
      refreshDisplay++;
      updateDisplay();
      return;
  }
  
  if (cursorDigitPosition < 1) return; // Nothing to do here, Abort, Cursor is in Park position

  // Select Tuning Mode; Digit or 2500 Step Mode
  if (tune2500Mode) {
      // Inc or Dec Freq by 2.5K, useful when tuning between SSB stations
      cursorDigitPosition = 3;
      deltaFreq += tuningDir * 2500;
      
      newFreq = (frequency / 2500) * 2500 + deltaFreq;
  }
  else {
      // Compute deltaFreq based on current Cursor Position Digit
      deltaFreq = tuningDir;
      for (int i = cursorDigitPosition; i > 1; i-- ) deltaFreq *= 10;
  
      newFreq = frequency + deltaFreq;  // Save Least Digits Mode
  }
  
  //newFreq = (frequency / abs(deltaFreq)) * abs(deltaFreq) + deltaFreq; // Zero Lesser Digits Mode 
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
}


// ###############################################################################
int inBandLimits(unsigned long freq){
    int upper, lower = 0;
    
       for (int i = 0; i < BANDS; i++) {
         lower = i * 2;
         upper = lower + 1;
         if (frequency >= pgm_read_dword(&bandLimits[lower]) &&
             frequency <= pgm_read_dword(&bandLimits[upper]) ) return i;
       }
       return 0;
}
    
    
// ###############################################################################
void checkTX(){
  
  if (freqUnStable) return;

  //we don't check for ptt when transmitting cw
  if (cwTimeout > 0) return;
    
  if(!inBandLimits(frequency)) return;
    
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

  if(!inBandLimits(frequency)) return;
  
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
  
  debug(P("btn Val= %d"), val);
  
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
  if (btn) debug(P("btn %d"), btn);
  
  switch (btn) {
    case 0: return; // Abort
    case 1: decodeFN(btn); break;  
    case 2: decodeMoveCursor(btn); break;    
    case 3: decodeMoveCursor(btn); break;
    case 4: decodeSideBandMode(btn); break;
    case 5: decodeBandUpDown(+1); break; // Band Up
    case 6: decodeBandUpDown(-1); break; // Band Down
    case 7: decodeTune2500Mode(); break; // 
    //case 7: decodeAux(7); break; // Report Un-Used as AUX Buttons
    default: return;
  }
}


// ###############################################################################
void decodeTune2500Mode() {
    cursorDigitPosition = 3; // Set default Tuning Digit
    tune2500Mode = !tune2500Mode;
    if(tune2500Mode) frequency = (frequency / 2500) * 2500;
    refreshDisplay++;
    updateDisplay();
    deDounceBtnRelease(); // Wait for Release
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
         if (frequency <= pgm_read_dword(&bandLimits[i*2+1])) {
           if (frequency >= pgm_read_dword(&bandLimits[i*2])) {
             // Save Current Ham frequency and sideBandMode
             freqCache[i] = frequency;
             sideBandModeCache[i] = sideBandMode;
             i++;
           }
           // Load From Next Cache
           frequency = freqCache[min(i,BANDS-1)];
           sideBandMode = sideBandModeCache[min(i,BANDS-1)];
           break;
         }
       }
       break;
     
     case -1:  // For Band Change, Down
       for (int i = BANDS-1; i > 0; i--) {
         if (frequency >= pgm_read_dword(&bandLimits[i*2])) {
           if (frequency <= pgm_read_dword(&bandLimits[i*2+1])) {
             // Save Current Ham frequency and sideBandMode
             freqCache[i] = frequency;
             sideBandModeCache[i] = sideBandMode;
             i--;
           }
           frequency = freqCache[max(i,0)];
           sideBandMode = sideBandModeCache[max(i,0)];
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
   //refreshDisplay++;
}


// ###############################################################################
void decodeSideBandMode(int btn) {
  sideBandMode++;
  sideBandMode %= 3; // Limit to Three Modes
  setSideband();
  cursorOff();
  printLine2CEL((char *)pgm_read_word(&sideBandText[sideBandMode]));
  refreshDisplay++;
  updateDisplay();
  deDounceBtnRelease(); // Wait for Release
  
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
      freqUnStable = false;  // Set Freq is NOT UnStable, as it is Stable
      refreshDisplay++;
      updateDisplay();
      deDounceBtnRelease(); // Wait for Button Release
}

// -------------------------------------------------------------------------------
void decodeAux(int btn) {
  //debug("Aux %d", btn);
  cursorOff();
  sprintf(c, P("Btn: %.2d"), btn);
  printLine2CEL(c);
  refreshDisplay++;
  updateDisplay();
  deDounceBtnRelease(); // Wait for Button Release
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
      ritVal = 0;
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
      //cursorOff();
      //printLine2CEL(P("VFO swap!"));
      break;
      
    case LONG_PRESS:
      vfoA = vfoB = frequency;
      ritOn = 0;
      refreshDisplay++;
      updateDisplay();
      cursorOff();
      printLine2CEL(P("VFO reset!"));
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
void setup() {
  
  // Initialize the Serial port so that we can use it for debugging
  Serial.begin(115200);
  debug(P("Radiono - Rev: %s"), RADIONO_VERSION);

  lcd.begin(LCD_COL, LCD_ROW);
  printLine1(P("Farhan - Minima"));
  printLine2(P("  Tranceiver"));
  delay(2000);
  
  sprintf(b, P("Radiono %s"), RADIONO_VERSION);
  printLine1CEL(b);
  
  sprintf(b, P("Rev: %s"), INC_REV);
  printLine2CEL(b);
  delay(2000);
  
  // Print just the File Name, Added by ERB
  //sprintf(c, P("F: %-13.13s"), P2(__FILE__));
  //printLine2CLE(c);
  //delay(2000);
  

  // The library automatically reads the factory calibration settings of your Si570
  // but it needs to know for what frequency it was calibrated for.
  // Looks like most HAM Si570 are calibrated for 56.320 Mhz.
  // If yours was calibrated for another frequency, you need to change that here
  vfo = new Si570(SI570_I2C_ADDRESS, 56320000);

  if (vfo->status == SI570_ERROR) {
    // The Si570 is unreachable. Show an error for 3 seconds and continue.
    printLine2CEL(P("Si570 comm error"));
    delay(3000);
  }
  
  printLine2CEL(" ");
 
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
  refreshDisplay = +1;
}


// ###############################################################################
// ###############################################################################
void loop(){
    unsigned long freq;
  
  readTuningPot();
  checkTuning();

  //the order of testing first for cw and then for ptt is important.
  checkCW();
  checkTX();
  checkButton();

  freq = frequency;
  if (!inTx & ritOn) freq += ritVal;
  vfo->setFrequency(freq + IF_FREQ);
  
  setSideband();
  setBandswitch(frequency);
  setRf386BandSignal(frequency);

  updateDisplay();
  
}

// ###############################################################################

//End
