#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ros.h>
#include <std_msgs/Float64.h>             // http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64.html
#include <geometry_msgs/Twist.h>          // http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html


// Wi-Fi
const char *wifi_ssid = "UtBot - rede linda";
const char *wifi_password = "utbotlaser";
int rosmaster_port = 11411;
IPAddress gateway(192, 168, 1, 1);
IPAddress rosmaster(gateway[0], gateway[1], gateway[2], 31);
IPAddress localIP(gateway[0], gateway[1], gateway[2], 32);
IPAddress dns(gateway[0], gateway[1], gateway[2], gateway[3]);
IPAddress subnet(255, 255, 255, 0);

/* MENSAGENS RECEBIDAS*/
geometry_msgs::Twist msg_cmdVel;                    // Para comandar a velocidade

/* MENSAGENS PUBLICADAS */
std_msgs::Float64 msg_cmd_wheelL;                     // Controla a rotação do motor esquerdo
std_msgs::Float64 msg_cmd_wheelR;                     // Controla a rotação do motor direito

/* NODO ROS */
ros::NodeHandle n;

void callback_cmdVel(const geometry_msgs::Twist& msg)
{
  msg_cmdVel = msg;
}

/* SUBSCRIBERS */
ros::Subscriber<geometry_msgs::Twist> sub_cmdVel("/pc/cmd_vel", &callback_cmdVel);

/* PUBLISHERS */
ros::Publisher pub_cmd_wheelL("/car/cmd_wheelL", &msg_cmd_wheelL);
ros::Publisher pub_cmd_wheelR("/car/cmd_wheelR", &msg_cmd_wheelR);

/* Millis */
unsigned long millis_now = millis();
unsigned long millis_lastLoop = millis_now;
float loopRate = 30.0;
unsigned long loopInterval = (unsigned long)(1000.0 * 1.0/loopRate); // T = 1/f

void setup(void)
{
  Serial.begin(57600);
  
  // To use as client
  WiFi.config(localIP, gateway, subnet, dns, dns);
  WiFi.begin(wifi_ssid, wifi_password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
  }

  n.getHardware()->setConnection(rosmaster, rosmaster_port);
  n.subscribe(sub_cmdVel);
  n.advertise(pub_cmd_wheelL);
  n.advertise(pub_cmd_wheelR);
  n.initNode();
}

void loop(void)
{
  millis_now = millis();

  if (millis_now - millis_lastLoop < loopInterval)
    return;
  else
    millis_lastLoop = millis();

  Serial.println(millis_now);
  
  n.spinOnce();

  msg_cmd_wheelL.data = msg_cmdVel.linear.x;
  msg_cmd_wheelR.data = msg_cmdVel.angular.z;

  pub_cmd_wheelL.publish(&msg_cmd_wheelL);
  pub_cmd_wheelR.publish(&msg_cmd_wheelR);
}
