#ifndef SERIALCOM_H
#define SERIALCOM_H

// Recebe mensagem com string no formato "<Mensagem>" ('<' e '>' são delimitadores)
void serialReceiveWithMarkers(boolean* newData, char* receivedChars, int numChars)
{
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char rc;    
    
    while (Serial.available() > 0 && *newData == false) {
        rc = Serial.read();
        
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
                //Serial.println(receivedChars);
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

// Filtra os dados de "<Header, %d>",
// Variavel: distancia (ultrassom)
void parseData(char* tempChars, char* messageFromUno, int* distance, int* color)
{
  char* strtokIndx;

  // Corta tempChars em ',', filtrando a string de header
  strtokIndx = strtok(tempChars, ",");  
  strcpy(messageFromUno, strtokIndx);
  
  /* O próximo strtok com arg1 = NULL continua de onde a ultima 
     chamada ao strtok() parou */
  
  // Filtra distance
  strtokIndx = strtok(NULL, ",");       
  *distance = atoi(strtokIndx);

  // Filtra color
  strtokIndx = strtok(NULL, ",");       
  *color = atoi(strtokIndx);
}

// Processa novos dados
void processNewData(char* tempChars, char* receivedChars, char* messageFromUno,
  int* distance, int* color, boolean* newData)
{
  // Trata a mensagem da SoftwareSerial
  strcpy(tempChars, receivedChars);
  parseData(tempChars, messageFromUno, distance, color);
  *newData = false;
}

// Manda mensagem de controle dos motores por serial
void serialSendMotorControl(float wheelL, float wheelR)
{
  char buff[64] = {0};
  snprintf(buff, 64, "<MotorControl, %f, %f>\n", wheelL, wheelR);
  Serial.write(buff, 64);
}

// Printa os dados na SERIAL NORMAL
void showParsedData(char* messageFromUno, int distance) {
    Serial.print("Message ");
    Serial.println(messageFromUno);
    Serial.print("distance ");
    Serial.println(distance);
}

#endif
