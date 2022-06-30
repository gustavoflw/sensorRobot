#ifndef HEX_H
#define HEX_H

void printHex(byte *buffer, byte bufferSize) 
{
 for (byte i = 0; i < bufferSize; i++) {
   Serial.print(buffer[i] < 0x10 ? " 0" : " ");
   Serial.print(buffer[i], HEX);
 }
}

#endif
