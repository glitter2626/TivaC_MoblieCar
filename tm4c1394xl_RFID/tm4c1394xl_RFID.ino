/*
Pin Connections
===================================      
RFID Module       Tiva C TM4C1294XL LaunchPads        
--------------    -----------------
Pin 1  (SDA)      Pin 63 PB_4 (CS(1))
Pin 2  (SCK)      Pin 64 PB_5 (SCK(1))
Pin 3  (MOSI)     Pin 2  PE_4 (MOSI(1))
Pin 4  (MISO)     Pin 6  PE_5 (MISO(1))
Pin 5  (IRQ)      Not connected
Pin 6  (GND)      Pin 60 GND
Pin 7  (RST)      Pin 54 PQ_3 RESET(NRSTDP)(MISO(5))
Pin 8  (3V3)      3V3
*/

#include "Mfrc522.h"
#include <SPI.h>

int CS = 63;                                 // chip select pin
int NRSTPD = 54;
Mfrc522 Mfrc522(CS,NRSTPD);
unsigned char serNum[5];

void setup()
{             
  Serial.begin(9600);
  
  SPI.setModule(1); 
  pinMode(CS, OUTPUT); 
  digitalWrite(CS, LOW);
  pinMode(NRSTPD, OUTPUT); 
  digitalWrite(NRSTPD, HIGH); 
                      
  Serial.println("Starting RFID-RC522 MIFARE module demonstration...\n");

  Mfrc522.Init();  
}

void loop()
{
  unsigned char status;
  unsigned char str[MAX_LEN];
  /*Searching Card, read card type	
  Request(unsigned char reqMode, unsigned char *TagType);
  reqMode: Search method TagType: return card type
  return MI_OK if successed*/
  status = Mfrc522.Request(PICC_REQIDL, str);
  
  if (status == MI_OK)
  {
    Serial.print("Card detected: ");
    Serial.print(str[0],BIN);
    Serial.print(" , ");
    Serial.print(str[1],BIN);
    Serial.println("");
  }
  /*Read card serial number
  serNum: 4 bytes card serial number, 5th is recheck byte
  return MI_OK if successed*/
  status = Mfrc522.Anticoll(str);
  memcpy(serNum, str, 5);
  if (status == MI_OK)
  {
    Serial.print("The card's number is: ");
    Serial.print(serNum[0]);
    Serial.print(" , ");
    Serial.print(serNum[1]);
    Serial.print(" , ");
    Serial.print(serNum[2]);
    Serial.print(" , ");
    Serial.print(serNum[3]);
    Serial.print(" , ");
    Serial.print(serNum[4]);
    Serial.println("");
 
    delay(1000);
  }
  /*command the card into sleepmode*/
  Mfrc522.Halt();	                        
}

