#include <Wire.h>
#include <SPI.h>
#include <Adafruit_PN532.h>
#include "pitches.h"

//==================================================================================================
// NFC
//==================================================================================================

// define just the pins connected to the IRQ and reset lines for I2C connection
#define PN532_IRQ   (7)
#define PN532_RESET (8)

// use an I2C connection:
Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET);

// hack used to not block process
const int NFC_MAX_RETRIES = 10;

//==================================================================================================
// acelerometer
//==================================================================================================

// I2C address of the MPU-6050
const int MPU_addr=0x68;

// constant used to determine the
const int16_t IMPACT_DIFF = 2000;

// variable to store current and previous acelerometer axes values
int16_t AcX,AcY,AcZ;
int16_t pAcX,pAcY,pAcZ;

//==================================================================================================
// LED
//==================================================================================================

const int greenLEDPin = 5;
const int redLEDPin = 6;
const int blueLEDPin = 3;

//==================================================================================================
// audio
//==================================================================================================

// notes in the melody:
int melodyA[] = {
  NOTE_G6, NOTE_E6, NOTE_A6, NOTE_D6, NOTE_C6, NOTE_G7
};

int melodyB[] = {
  NOTE_G1, NOTE_E1, NOTE_A1, NOTE_D1, NOTE_C1, NOTE_G2
};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = {
  8, 8, 8, 8, 8, 4
};


//==================================================================================================
// CoBi setup
//==================================================================================================

void setup(void) {
  Serial.begin(115200);
  
  // LED pin configuration
  pinMode(greenLEDPin, OUTPUT);
  pinMode(redLEDPin, OUTPUT);
  pinMode(blueLEDPin, OUTPUT);
  
  // acelerometer configuration
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  // NFC
  prepareNFC();
}

// prepare NFC board to be used by CoBi
void prepareNFC() {
  nfc.begin();

  // check if it is possible to use NFC reader
  uint32_t versiondata = nfc.getFirmwareVersion();
  if (! versiondata) {
    Serial.print("Didn't find PN53x board");

    // do not continue program
    while (1);
  }
  
  Serial.print("Found chip PN5"); Serial.println((versiondata>>24) & 0xFF, HEX); 
  Serial.print("Firmware ver. "); Serial.print((versiondata>>16) & 0xFF, DEC); 
  Serial.print('.'); Serial.println((versiondata>>8) & 0xFF, DEC);
  
  // configure board to read RFID tags
  nfc.SAMConfig();

  // hack to allow code to be executed even if no tag is being read
  nfc.setPassiveActivationRetries(NFC_MAX_RETRIES);
}

//==================================================================================================
// CoBi main loop
//==================================================================================================

void loop(void) {
  handleAcelerometer();
  readNFC();
}

void handleAcelerometer() {
  // read current acelerometer values
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

  // calculate difference from previous values
  float xdiff = AcX - pAcX;
  if (xdiff < 0) {
    xdiff = xdiff * -1;
  }
  
  float ydiff = AcY - pAcY;
  if (ydiff < 0) {
    ydiff = ydiff * -1;
  }

  float zdiff = AcZ - pAcZ;
  if (zdiff < 0) {
    zdiff = zdiff * -1;
  }

  // check if difference represents that cobi was hitted
  if (xdiff > IMPACT_DIFF || xdiff > IMPACT_DIFF || xdiff > IMPACT_DIFF) {
    hitFeedback();
    delay(1000);
  }

  // replace previous values with new ones
  pAcX = AcX;
  pAcY = AcY;
  pAcZ = AcZ;
}

void hitFeedback() {
  // change LED color
  randomLEDColor();

  // play sound
  playMelody(melodyA);
  playMelody(melodyB);
}

void playMelody(int melody[]) {
  // iterate over the notes of the melody:
  for (int thisNote = 0; thisNote < 6; thisNote++) {
    
    // to calculate the note duration, take one second divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / noteDurations[thisNote];
    tone(9, melody[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    
    // stop the tone playing:
    noTone(9);

    // alternate LED color on each note
    randomLEDColor();
  }
}

void randomLEDColor() {
  analogWrite(redLEDPin, random(255));
  analogWrite(greenLEDPin, random(255));
  analogWrite(blueLEDPin, random(255));
}

void readNFC() {
  uint8_t success;
  uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
  uint8_t uidLength;                        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)
    
  // Wait for an ISO14443A type cards (Mifare, etc.).  When one is found
  // 'uid' will be populated with the UID, and uidLength will indicate
  // if the uid is 4 bytes (Mifare Classic) or 7 bytes (Mifare Ultralight)
  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);
  
  if (success) {
    // Display some basic information about the card
    Serial.println("Found an ISO14443A card");
    Serial.print("  UID Length: ");Serial.print(uidLength, DEC);Serial.println(" bytes");
    Serial.print("  UID Value: ");
    nfc.PrintHex(uid, uidLength);
      playMelody(melodyB);
      playMelody(melodyB);
      playMelody(melodyB);
    
//    if (uidLength == 4) {
//      // We probably have a Mifare Classic card ... 
//      uint32_t cardid = uid[0];
//      cardid <<= 8;
//      cardid |= uid[1];
//      cardid <<= 8;
//      cardid |= uid[2];  
//      cardid <<= 8;
//      cardid |= uid[3]; 
//      Serial.print("Seems to be a Mifare Classic card #");
//      Serial.println(cardid);
//    }
  }
}
