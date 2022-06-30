#define ROSSERIAL_ARDUINO_TCP

#include <ros.h>
#include <std_msgs/Bool.h>                // http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html
#include <std_msgs/Int32.h>               // http://docs.ros.org/en/lunar/api/std_msgs/html/msg/Int32.html
#include <std_msgs/Float64.h>             // http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64.html
#include <sensor_msgs/Temperature.h>      // https://docs.ros.org/en/api/sensor_msgs/html/msg/Temperature.html
#include <sensor_msgs/RelativeHumidity.h> // http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/RelativeHumidity.html
#include <geometry_msgs/Twist.h>          // http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html
#include <geometry_msgs/Vector3.h>        // http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Vector3.html

#include "wifi.h"
#include "rfid.h"
#include "gyro.h"
#include "cinematica.h"
#include "SerialCom.h"

#define SS_PIN     D8 // RFID
#define RST_PIN    D0 // RFID

/* VARIÁVEIS DE COMPRIMENTO DO ROBÔ [cm]*/
const float robot_L = 14.0, robot_R = 3.5; // Distância entre rodas e raio delas [cm]

/* RFID */
MFRC522             rfid(SS_PIN, RST_PIN);
MFRC522::MIFARE_Key key; 
byte                nuidPICC[4];

/* GIROSCÓPIO E ACELERÔMETRO */
Adafruit_MPU6050 sensor_gyro;
sensors_event_t  acceleration, rotation, temperature;
sensors_event_t  offset_acceleration, offset_rotation;

/* WIFI */
//const char *wifi_ssid      = "UtBot - rede linda";
//const char *wifi_password  = "utbotlaser";
const char *wifi_ssid      = "elonmusk";
const char *wifi_password  = "elonmusk";
int        rosmaster_port  = 11411;
uint8_t    newMACAddress[] = {0x5C, 0xE8, 0x83, 0x36, 0x9A, 0xC3};
//IPAddress  gateway(192, 168, 1, 1);
IPAddress  gateway(192, 168, 4, 1);
IPAddress  rosmaster(gateway[0], gateway[1], gateway[2], 31        );
IPAddress  localIP  (gateway[0], gateway[1], gateway[2], 32        );
IPAddress  dns      (gateway[0], gateway[1], gateway[2], gateway[3]);
IPAddress  subnet   (       255,        255,        255,        0  );

// Serial (recebe mensagens do ultrassom)
const byte numChars = 64;
char       receivedChars [numChars];
char       tempChars     [numChars];
char       messageFromUno[numChars] = {0};
boolean    newData = false;

/* MILLIS */
unsigned long millis_now      = millis();
unsigned long millis_lastLoop = millis_now;
float         loopRate        = 30.0;
unsigned long loopInterval    = (unsigned long)(1000.0 * 1.0/loopRate); // T = 1/f

/* MENSAGENS RECEBIDAS*/
geometry_msgs::Twist          msg_cmdVel;           // Para comandar a velocidade

/* MENSAGENS PUBLICADAS */
std_msgs::Int32               msg_color;
std_msgs::Bool                msg_rfid;             // Define se leu um cartão ou não
std_msgs::Float64             msg_cmd_wheelL;       // Controla a rotação do motor esquerdo
std_msgs::Float64             msg_cmd_wheelR;       // Controla a rotação do motor direito
std_msgs::Int32               msg_distance;         // Distância detectada pelo sensor ultrassônico
sensor_msgs::RelativeHumidity msg_relativeHumidity; // Umidade detectada pelo sensor DHT
geometry_msgs::Vector3        msg_rotation;         // Rotação detectada pelo giroscópio
geometry_msgs::Vector3        msg_acceleration;     // Aceleração detectada pelo acelerômetro
sensor_msgs::Temperature      msg_temperature;      // Temperatura detectada pelo giroscópio e acelerômetro (você leu certo)

/* FUNÇÕES DE CALLBACK
  - são chamadas toda vez que  ros::spinOnce() ou ros::spin() são chamadas
  - associam ações a mensagens recebidas */
void callback_cmdVel(const geometry_msgs::Twist& msg)
{
  msg_cmdVel = msg;
}

/* NODO */
ros::NodeHandle n;

/* SUBSCRIBERS */
ros::Subscriber<geometry_msgs::Twist> sub_cmdVel("/pc/cmd_vel", &callback_cmdVel);

/* PUBLISHERS */
ros::Publisher pub_color            ("/car/color",             &msg_color           );
ros::Publisher pub_rfid             ("/car/rfid",              &msg_rfid            );
ros::Publisher pub_cmd_wheelL       ("/car/cmd_wheelL",        &msg_cmd_wheelL      );
ros::Publisher pub_cmd_wheelR       ("/car/cmd_wheelR",        &msg_cmd_wheelR      );
ros::Publisher pub_distance         ("/car/distance",          &msg_distance        );
ros::Publisher pub_relativeHumidity ("/car/relative_humidity", &msg_relativeHumidity);
ros::Publisher pub_rotation         ("/car/rotation",          &msg_rotation        );
ros::Publisher pub_acceleration     ("/car/acceleration",      &msg_acceleration    );
ros::Publisher pub_temperature      ("/car/temperature",       &msg_temperature     );

void setup() 
{
  // Serial
  Serial.begin(57600);
  for (int i=0; i<400; i=i+100) {
    Serial.print(".");
    delay(100);
  }
  Serial.println("Hello world");
  delay(1000);

  // RFID
  SPI.begin();
  setupRFID(&rfid, &key);
  delay(1000);

  // Giroscópio e acelerômetro (ISSO AQUI BUGA O ESP!!!)
  setupGyro(&sensor_gyro);
  sensor_gyro.getEvent(&offset_acceleration, &offset_rotation, &temperature);
  delay(1000);

  Serial.println    ("Sensors initialized");

  // Wifi
  setupWifiAsClient (wifi_ssid, wifi_password, localIP, gateway, subnet, dns);
  pinMode           (LED_BUILTIN, OUTPUT);
  digitalWrite      (LED_BUILTIN, LOW   ); // LOW acende o led (??????)

  // Inicialização do nodo ROS e configuração dos publishers e subscribers
  n.getHardware()->setConnection(rosmaster, rosmaster_port);
  n.initNode();
  setupRosPublishersSubscribers();
}

void loop() 
{
  millis_now = millis();

  if (millis_now - millis_lastLoop < loopInterval)
    return;
  else
    millis_lastLoop = millis();
  
  // Leitura dos sensores
  readSensors();

  /* ------- COMUNICAÇÃO COM O PC VIA REDE/ROS ------- */
  // Atualiza dados dos tópicos inscritos via rede (ver funções de callback)
  n.spinOnce();

  // Atualiza valores de wheelL e wheelR com base no comando de velocidade (cmdVel)
  inverseKinematics(  &msg_cmd_wheelL.data, &msg_cmd_wheelR.data, 
                      robot_L, robot_R, 
                      msg_cmdVel.linear.x, msg_cmdVel.angular.z);

//  Serial.println(msg_color.data);

  // Publica mensagens via rede
  publishRosMessages();
  /* ------------------------------------------------- */
  
  /* ----- COMUNICAÇÃO COM O ARDUINO VIA SERIAL ------ */
  serialSendMotorControl(msg_cmd_wheelL.data, msg_cmd_wheelR.data);
  /* ------------------------------------------------- */
}

// Configura publishers e subscribers
void setupRosPublishersSubscribers()
{
  // Subscribers
  n.subscribe(sub_cmdVel          );

  // Publishers
  n.advertise(pub_color           );
  n.advertise(pub_rfid            );
  n.advertise(pub_cmd_wheelL      );
  n.advertise(pub_cmd_wheelR      );
  n.advertise(pub_distance        );
  n.advertise(pub_relativeHumidity);
  n.advertise(pub_rotation        );
  n.advertise(pub_acceleration    );
  n.advertise(pub_temperature     );

  // Mensagens publicadas
  msg_cmd_wheelL.data                    = 1.0;
  msg_cmd_wheelR.data                    = 1.0;
  msg_distance.data                      = 1.0;
  msg_temperature.temperature            = 1.0;
  msg_relativeHumidity.relative_humidity = 1.0;
  msg_rfid.data                          = false;
}

// Publica mensagens via rede
void publishRosMessages()
{
  pub_color.publish           (&msg_color           );
  pub_rfid.publish            (&msg_rfid            );
  pub_cmd_wheelL.publish      (&msg_cmd_wheelL      );
  pub_cmd_wheelR.publish      (&msg_cmd_wheelR      );
  pub_distance.publish        (&msg_distance        );
  pub_relativeHumidity.publish(&msg_relativeHumidity);
  pub_rotation.publish        (&msg_rotation        );
  pub_acceleration.publish    (&msg_acceleration    );
  pub_temperature.publish     (&msg_temperature     );
}

void readSensors()
{
  // Giroscópio/acelerômetro
  /* USAR COM CUIDADO, HORRÍVEL!
   * Trava o loop se o sensor não tá conectado? 
  */
  sensor_gyro.getEvent( &acceleration, &rotation, &temperature); 
  gyroToMessages      ( &msg_rotation, &msg_acceleration, &msg_temperature, 
                        &offset_rotation, &offset_acceleration,
                        &rotation, &acceleration, &temperature);

  // RFID
  if (rfid.PICC_IsNewCardPresent()) {
    Serial.println("RFID!");
    msg_rfid.data = true;
  }
  else
    msg_rfid.data = false;

  // Ultrassom e Cor (por serial)
  serialReceiveWithMarkers(&newData, receivedChars, numChars);
  if (newData == true) {
//    Serial.println(msg_color.data);/
    processNewData( tempChars, receivedChars, messageFromUno,
                    &msg_distance.data, &msg_color.data, &newData);
//    showParsedData( messageFromUno, msg_distance.data);
  }
}
