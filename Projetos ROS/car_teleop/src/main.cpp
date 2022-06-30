#include <stdlib.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h> 					// http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html
#include <std_msgs/Int32.h>               	// http://docs.ros.org/en/lunar/api/std_msgs/html/msg/Int32.html
#include <std_msgs/Float64.h>				// http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64.html
#include <sensor_msgs/Temperature.h>		// https://docs.ros.org/en/api/sensor_msgs/html/msg/Temperature.html
#include <sensor_msgs/RelativeHumidity.h>	// http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/RelativeHumidity.html
#include <geometry_msgs/Twist.h>			// http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html
#include <geometry_msgs/Vector3.h>        	// http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Vector3.html
#include <conio.h>

/* MENSAGENS RECEBIDAS */
std_msgs::Float64 msg_range;						// Distância detectada pelo sensor ultrassônico
sensor_msgs::RelativeHumidity msg_relativeHumidity; // Umidade detectada pelo sensor DHT
std_msgs::Int32 msg_cmd_wheelL;                     // Controla a rotação do motor esquerdo
std_msgs::Int32 msg_cmd_wheelR;                     // Controla a rotação do motor direito
geometry_msgs::Vector3 msg_rotation;                // Rotação detectada pelo giroscópio
geometry_msgs::Vector3 msg_acceleration;            // Aceleração detectada pelo acelerômetro
sensor_msgs::Temperature msg_temperature;           // Temperatura detectada pelo giroscópio e acelerômetro (você leu certo)

/* MENSAGENS PUBLICADAS */
geometry_msgs::Twist msg_cmdVel; 					// Para comandar a velocidade

char key = '0';										// Guarda qual tecla está pressionada

/* FUNÇÕES DE CALLBACK
 - são chamadas toda vez que ros::spinOnce() ou ros::spin() são chamadas
 - associam ações a mensagens recebidas */
void callback_Range(const std_msgs::Float64::ConstPtr& msg);
void callback_RelativeHumidity(const sensor_msgs::RelativeHumidity::ConstPtr& msg);
void callback_CmdWheelL(const std_msgs::Int32::ConstPtr& msg);
void callback_CmdWheelR(const std_msgs::Int32::ConstPtr& msg);
void callback_Rotation(const geometry_msgs::Vector3::ConstPtr& msg);
void callback_Acceleration(const geometry_msgs::Vector3::ConstPtr& msg);
void callback_Temperature(const sensor_msgs::Temperature::ConstPtr& msg);

/* FUNÇÕES DE CONTROLE */
// Controla o carro com o teclado (teclas W, A, S, D)
void teleop(ros::Publisher pub_cmdVel);
// Controla o carro automaticamente
void automatic();

// Printa logs
void printLogs(ros::Time* t, ros::Time* last_t, double max_dt);

// FUNÇÃO MAIN
int main(int argc, char **argv)
{
	// INICIALIZAÇÃO DO NODO
	ros::init(argc, argv, "main");
	ros::NodeHandle n;
	
	// SUBSCRIBERS
	ros::Subscriber sub_range = n.subscribe("/range", 1, callback_Range);
	ros::Subscriber sub_relativeHumidity = n.subscribe("/car/relative_humidity", 1, callback_RelativeHumidity);
	ros::Subscriber sub_cmd_wheelL = n.subscribe("/car/cmd_wheelL", 1, callback_CmdWheelL);
	ros::Subscriber sub_cmd_wheelR = n.subscribe("/car/cmd_wheelR", 1, callback_CmdWheelR);
	ros::Subscriber sub_rotation = n.subscribe("/car/rotation", 1, callback_Rotation);
	ros::Subscriber sub_acceleration = n.subscribe("/car/acceleration", 1, callback_Acceleration);
	ros::Subscriber sub_temperature = n.subscribe("/car/temperature", 1, callback_Temperature);
	
	// PUBLISHERS
	ros::Publisher pub_cmdVel = n.advertise<geometry_msgs::Twist>("/pc/cmd_vel", 1);
	
	// VARIÁVEIS DE TEMPO
	ros::Rate loopRate(10); 				// Frequência de loop
	ros::Time t = ros::Time::now(); 		// Tempo atual no loop
	ros::Time last_t = t;					// Tempo da última iteração no loop
	double dt = t.toSec() - last_t.toSec(); // Diferença de tempo no loop
	ros::Time t_log = t;					// Tempo em que o log apareceu pela última vez
	
	printf("ESCOLHA UM MODO \n 1 - teleoperacao\n 2 - automatico\n > ");
	// char mode = getchar();
	char mode = '1';
	
	// Loop principal
	while (ros::ok()) {
		t = ros::Time::now();
		dt = t.toSec() - last_t.toSec();
		last_t = t;
		
		// Atualiza dados dos tópicos inscritos (callback)
		ros::spinOnce();

		if (mode == '1')
			teleop(pub_cmdVel);
		else if (mode == '2')
			automatic();

		printLogs(&t, &t_log, 0.0);	
		
		loopRate.sleep();
	}
	
	return 0;
}

void callback_Range(const std_msgs::Float64::ConstPtr& msg)
{
	msg_range = *msg;
	ROS_INFO("DISTÂNCIA %f", msg_range.data);
}

void callback_RelativeHumidity(const sensor_msgs::RelativeHumidity::ConstPtr& msg) 
{
	msg_relativeHumidity = *msg;
}

void callback_CmdWheelL(const std_msgs::Int32::ConstPtr& msg)
{
	msg_cmd_wheelL = *msg;
}

void callback_CmdWheelR(const std_msgs::Int32::ConstPtr& msg)
{
	msg_cmd_wheelR = *msg;
}

void callback_Rotation(const geometry_msgs::Vector3::ConstPtr& msg)
{
	msg_rotation = *msg;
}

void callback_Acceleration(const geometry_msgs::Vector3::ConstPtr& msg)
{
	msg_acceleration = *msg;
}

void callback_Temperature(const sensor_msgs::Temperature::ConstPtr& msg) 
{
	msg_temperature = *msg;
}

void printLogs(ros::Time* t, ros::Time* last_t, double max_dt)
{
	if (t->toSec() - last_t->toSec() > max_dt) {
		*last_t = *t;
		system("CLS"); 								// limpa terminal
		ROS_INFO("Temperatura %f", 					msg_temperature.temperature);
		ROS_INFO("Umidade %f", 						msg_relativeHumidity.relative_humidity);
		ROS_INFO("cmdVel linear.x:%f angular.z:%f", msg_cmdVel.linear.x, msg_cmdVel.angular.z);
		ROS_INFO("cmd_wheelL:%i cmd_wheelR:%i", 	msg_cmd_wheelL.data, msg_cmd_wheelR.data);
		ROS_INFO("Rotation: %f, %f, %f", 			msg_rotation.x, msg_rotation.y, msg_rotation.z );
		ROS_INFO("Acceleration: %f, %f, %f", 		msg_acceleration.x, msg_acceleration.y, msg_acceleration.z );
		ROS_INFO("key %c", 							key);
	}
}

void teleop(ros::Publisher pub_cmdVel)
{
	/* CONVENÇÃO
		- Frente > 0, trás < 0
		- Horário > 0, anti-horário < 0 */

	if (_kbhit()) {
		key = _getch();

		// Frente reto
		if (key == 'W' || key == 'w') {
			msg_cmdVel.linear.x = 10.0;
			msg_cmdVel.angular.z = 0.0;
		}
		// Frente horário
		else if (key == 'D' || key == 'd') {
			msg_cmdVel.linear.x = 10.0;
			msg_cmdVel.angular.z = 10.0;
		}
		// Frente anti-horário
		if (key == 'A' || key == 'a') {
			msg_cmdVel.linear.x = 10.0;
			msg_cmdVel.angular.z = -10.0;
		}
		// Trás reto
		else if (key == 'S' || key == 's') {
			msg_cmdVel.linear.x = -10.0;
			msg_cmdVel.angular.z = 0.0;
		}
		// Parar
		else if (key == 'p' || 'key' == 'P'){
			msg_cmdVel.linear.x = 0.0;
			msg_cmdVel.angular.z = 0.0;
		}

		pub_cmdVel.publish(msg_cmdVel);
	}

	
}

void automatic()
{
	
}