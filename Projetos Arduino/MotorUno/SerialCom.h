#ifndef SERIALCOM_H
#define SERIALCOM_H

#include <SoftwareSerial.h>

/*
 * FUNÇÕES RELACIONADAS À COMUNICAÇÃO SOFTWARESERIAL
 */

// Recebe mensagem com string no formato "<Mensagem>" ('<' e '>' são delimitadores)
void swSerialReceiveWithMarkers(SoftwareSerial* mySerial, boolean* newData, char* receivedChars, int numChars) 
{
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char rc;

    while (mySerial->available() > 0 && *newData == false) {
        rc = mySerial->read();

        if (recvInProgress == true) {
            if (rc != '>') {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                *newData = true;
            }
        }

        else if (rc == '<') {
            recvInProgress = true;
        }
    }
}

// Filtra os dados de "<Header, %d, %d>",
// Variáveis: wheelL, wheelR
void parseData(char* tempChars, char* messageFromESP, float* wheelL, float* wheelR)
{
  char* strtokIndx;

  // Corta tempChars em ',', filtrando a string de header
  strtokIndx = strtok(tempChars, ",");  
  strcpy(messageFromESP, strtokIndx);
  
  /* O próximo strtok com arg1 = NULL continua de onde a ultima 
     chamada ao strtok() parou */
  
  // Filtra wheelL
  strtokIndx = strtok(NULL, ",");       
  *wheelL = atof(strtokIndx);

  // Filtra wheelR
  strtokIndx = strtok(NULL, ",");
  *wheelR = atof(strtokIndx);
}

// Processa novos dados
void processNewData(char* tempChars, char* receivedChars, char* messageFromESP,
  float* wheelL, float* wheelR, boolean* newData)
{
  // Trata a mensagem da SoftwareSerial
  strcpy(tempChars, receivedChars);
  parseData(tempChars, messageFromESP, wheelL, wheelR);
  *newData = false;
}


// Envia mensagem "<Header, %d>"
// Variáveis: distância
void swSerialSendWithMarkers(SoftwareSerial* mySerial, int distance, int color)
{
  char buff[32] = {0};
  snprintf(buff, 32, "<Distance, %d, %d>\n", distance, color);
  mySerial->write(buff, 32);
}

#endif
