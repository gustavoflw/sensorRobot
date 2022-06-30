#ifndef RFID_H
#define RFID_H

#include <SPI.h>                          // RFID usa isso
#include <MFRC522.h>                      // RFID
#include "hex.h"

void setupRFID(MFRC522* rfid, MFRC522::MIFARE_Key* key) 
{
    rfid->PCD_Init(); // Init MFRC522
    for (byte i = 0; i < 6; i++) {
      key->keyByte[i] = 0xFF;
    }
}

void serialPrintRFID(MFRC522* rfid, MFRC522::MIFARE_Key* key)
{
  Serial.print("Reader: ");
  rfid->PCD_DumpVersionToSerial();
  Serial.print("RFID Key: ");
  printHex(key->keyByte, MFRC522::MF_KEY_SIZE);
  Serial.println();  
}

#endif

















//{
//	rfid.PICC_ReadCardSerial();
//    	MFRC522::PICC_Type piccType = rfid.PICC_GetType(rfid.uid.sak);
//    Serial.println();
//    Serial.print(F("PICC type: "));
//    Serial.println(rfid.PICC_GetTypeName(piccType));
//
//    if (rfid.uid.uidByte[0] != nuidPICC[0] ||
//     rfid.uid.uidByte[1] != nuidPICC[1] ||
//     rfid.uid.uidByte[2] != nuidPICC[2] ||
//     rfid.uid.uidByte[3] != nuidPICC[3] ) {
//      Serial.println(F("A new card has been detected."));
//      // Armazena NUID novo no array nuidPICC:
//      for (byte i = 0; i < 4; i++)
//        nuidPICC[i] = rfid.uid.uidByte[i];
//      Serial.println(F("The NUID tag is:"));
//      Serial.print(F("In hex: "));
//      printHex(rfid.uid.uidByte, rfid.uid.size);
//      Serial.println();
//     }
//    else 
//      Serial.println(F("Card read previously."));
//
//     rfid.PICC_HaltA(); // Halt PICC
//     rfid.PCD_StopCrypto1(); // Stop encryption on PCD
//}
