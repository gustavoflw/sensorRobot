#include <stdlib.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h> 					// http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html
#include <std_msgs/Int32.h>               	// http://docs.ros.org/en/lunar/api/std_msgs/html/msg/Int32.html
#include <std_msgs/Float64.h>				// http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64.html
#include <sensor_msgs/Temperature.h>		// https://docs.ros.org/en/api/sensor_msgs/html/msg/Temperature.html
#include <sensor_msgs/RelativeHumidity.h>	// http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/RelativeHumidity.html
#include <nav_msgs/Odometry.h>				// http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html
#include <geometry_msgs/Twist.h>			// http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html
#include <geometry_msgs/Vector3.h>        	// http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Vector3.html
#include "__kbhit.h"

/* NOTAS
	- Distância pro RFID pegar: 3 cm
	- Altura da tag: 15 cm
	- Raio da plataforma: ??	
*/

/* CONVENÇÃO
	- Frente > 0, trás < 0
	- Horário > 0, anti-horário < 0 
*/

/* MENSAGENS RECEBIDAS */
std_msgs::Int32				  msg_color;            // Define a cor do IR
std_msgs::Bool				  msg_rfid;				// Define se leu um cartão ou não
std_msgs::Int32   			  msg_distance;			// Distância detectada pelo sensor ultrassônico
std_msgs::Float64			  msg_cmd_wheelL;       // Controla a rotação do motor esquerdo
std_msgs::Float64 		 	  msg_cmd_wheelR;       // Controla a rotação do motor direito
sensor_msgs::Temperature 	  msg_temperature;      // Temperatura detectada pelo giroscópio e acelerômetro (você leu certo)
sensor_msgs::RelativeHumidity msg_relativeHumidity; // Umidade detectada pelo sensor DHT
geometry_msgs::Vector3 		  msg_acceleration;     // Aceleração linear detectada pelo acelerômetro
geometry_msgs::Vector3 		  msg_rotation;         // Aceleração angular detectada pelo giroscópio

/* MENSAGENS PUBLICADAS */
geometry_msgs::Twist   msg_cmdVel; 					// Para comandar a velocidade
geometry_msgs::Vector3 msg_odomVelLinear;			// Velocidade linear calculada
geometry_msgs::Vector3 msg_odomPosition;			// Posição calculada
geometry_msgs::Vector3 msg_odomVelAngular;			// Velocidade angular calculada
geometry_msgs::Vector3 msg_odomOrientation;			// Orientação calculada

/* FUNÇÕES DE CALLBACK
 - são chamadas toda vez que ros::spinOnce() ou ros::spin() são chamadas
 - associam ações a mensagens recebidas */
void callback_Color			  (const std_msgs::Int32::ConstPtr&				  msg);
void callback_Rfid			  (const std_msgs::Bool::ConstPtr& 				  msg);
void callback_Distance		  (const std_msgs::Int32::ConstPtr& 			  msg);
void callback_CmdWheelL		  (const std_msgs::Float64::ConstPtr& 			  msg);
void callback_CmdWheelR		  (const std_msgs::Float64::ConstPtr& 			  msg);
void callback_Temperature	  (const sensor_msgs::Temperature::ConstPtr& 	  msg);
void callback_RelativeHumidity(const sensor_msgs::RelativeHumidity::ConstPtr& msg);
void callback_Rotation		  (const geometry_msgs::Vector3::ConstPtr& 		  msg);
void callback_Acceleration	  (const geometry_msgs::Vector3::ConstPtr& 		  msg);

float v = 0.9, w = 0.14;

// VARIÁVEIS DO CONTROLE AUTONOMO
int dist_foundColumn = 65, dist_nearColumn = 10, dist_willCrash = 7;
bool foundColumn  = false;
bool nearColumn   = false;
bool foundCard    = false;
bool willCrash    = false;
bool isColorBlack = false;
#define AUTO_LOOKFORCOLUMN 1
#define AUTO_GOTOCOLUMN 2
#define AUTO_AROUNDCOLUMN 3
#define AUTO_BACKWARDS 4
int automaticMode = 0;

// Controla o carro com o teclado
void teleop(ros::Publisher* pub_cmdVel);

// Printa logs
void printLogs();

// Integra aceleração linear e angular para calcular odometria
void integrateSystem(double dt);

// Guarda a tecla pressionada
char key = '0';

// Modos de operação
#define OPERATION_TELEOP 1
#define OPERATION_AUTOMATIC 2
int operationMode = OPERATION_TELEOP;

// Navega de forma autônoma
void automatic(ros::Publisher* pub_cmdVel, ros::Rate* loopRate,
	ros::Time* t, ros::Time* last_t);

// [*** TESTAR ***] Não sabe onde a coluna está -> gira até ajustar a orientação
void automatic_lookForColumn(ros::Publisher* pub_cmdVel, ros::Rate* loopRate);

// [*** TESTAR ***] Perto da coluna -> navega ao redor dela
void automatic_aroundColumn(ros::Publisher* pub_cmdVel, ros::Rate* loopRate);

// [*** TESTAR ***] Sabe onde a coluna está e não está perto dela -> navega reto até ela
void automatic_toColumn(ros::Publisher* pub_cmdVel, ros::Rate* loopRate,
	ros::Time* t, ros::Time* last_t);

// FUNÇÃO MAIN
int main(int argc, char **argv)
{
	// INICIALIZAÇÃO DO NODO
	ros::init(argc, argv, "main");
	ros::NodeHandle n;
	
	// SUBSCRIBERS
	ros::Subscriber sub_color 		 	 = n.subscribe("/car/color", 		 	 1, callback_Color);
	ros::Subscriber sub_rfid 		 	 = n.subscribe("/car/rfid", 		 	 1, callback_Rfid);
	ros::Subscriber sub_distance 		 = n.subscribe("/car/distance", 		 1, callback_Distance);
	ros::Subscriber sub_cmd_wheelL 		 = n.subscribe("/car/cmd_wheelL", 		 1, callback_CmdWheelL);
	ros::Subscriber sub_cmd_wheelR 		 = n.subscribe("/car/cmd_wheelR", 		 1, callback_CmdWheelR);
	ros::Subscriber sub_temperature 	 = n.subscribe("/car/temperature",		 1, callback_Temperature);
	ros::Subscriber sub_relativeHumidity = n.subscribe("/car/relative_humidity", 1, callback_RelativeHumidity);
	ros::Subscriber sub_rotation 		 = n.subscribe("/car/rotation", 		 1, callback_Rotation);
	ros::Subscriber sub_acceleration 	 = n.subscribe("/car/acceleration", 	 1, callback_Acceleration);
	
	// PUBLISHERS
	ros::Publisher pub_cmdVel 		 	= n.advertise<geometry_msgs::Twist>	 ("/pc/cmd_vel", 		  1);
	ros::Publisher pub_odomPosition  	= n.advertise<geometry_msgs::Vector3>("/pc/odom_position", 	  1);
	ros::Publisher pub_odomOrientation  = n.advertise<geometry_msgs::Vector3>("/pc/odom_orientation", 1);							
	
	// VARIÁVEIS DE TEMPO
	ros::Rate loopRate(30); 						// Frequência de loop
	ros::Time t		 = ros::Time::now(); 			// Tempo atual no loop
	ros::Time last_t = t;							// Tempo da última iteração no loop
	ros::Time t_log  = t;							// Tempo em que o log apareceu pela última vez
	double dt 		 = t.toSec() - last_t.toSec(); 	// Diferença de tempo entre iterações seguidas do loop

	// MENSAGENS = 0 (PRECAUÇÃO)
	msg_odomVelLinear.x  = 0;
	msg_odomVelLinear.y  = 0;
	msg_odomVelLinear.z  = 0;
	msg_odomVelAngular.x = 0;
	msg_odomVelAngular.y = 0;
	msg_odomVelAngular.z = 0;
	msg_color.data = 0;
	
	// LOOP PRINCIPAL
	while (ros::ok()) {
		loopRate.sleep();

		// Atualiza variáveis de tempo
		t  	   = ros::Time::now();
		dt	   = t.toSec() - last_t.toSec();
		last_t = t;
		
		// Atualiza dados dos tópicos inscritos
		ros::spinOnce();

		integrateSystem(dt);

		if (isColorBlack)
			automatic(&pub_cmdVel, &loopRate, &t, &last_t);
		else
			teleop(&pub_cmdVel);
	}
	
	return 0;
}

void callback_Rfid			  (const std_msgs::Bool::ConstPtr& 				  msg)
{
	msg_rfid = *msg;
	foundCard = msg_rfid.data == true;
}

void callback_Color			  (const std_msgs::Int32::ConstPtr&				  msg)
{
	msg_color = *msg;
	// msg_color.data = 400; // APAGAR;
	isColorBlack = msg_color.data >= 300;
}

void callback_Distance(const std_msgs::Int32::ConstPtr& msg)
{
	msg_distance = *msg;

	foundColumn = msg_distance.data < dist_foundColumn;
	nearColumn  = msg_distance.data < dist_nearColumn;
	willCrash   = msg_distance.data <  dist_willCrash;
}

void callback_RelativeHumidity(const sensor_msgs::RelativeHumidity::ConstPtr& msg) 
{
	msg_relativeHumidity = *msg;
}

void callback_CmdWheelL(const std_msgs::Float64::ConstPtr& msg)
{
	msg_cmd_wheelL = *msg;
}

void callback_CmdWheelR(const std_msgs::Float64::ConstPtr& msg)
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

void printLogs()
{
	system("clear"); 								// limpa terminal
	puts("Controles:");
	puts(" - M: selecionar modo de operacao");
	puts(" - Z, A, Q, W, E, D, C, X, S: teleoperacao");
	puts("");
	ROS_INFO("key %c", 								key);
	// ROS_INFO("mode %c", 						    mode);
	ROS_INFO("---------------------------------");
	ROS_INFO("rfid %d\n", msg_rfid.data);
	if (msg_rfid.data)
		printf("\a");

	ROS_INFO("Color %d",							msg_color.data);
	ROS_INFO("Temperature %f", 						msg_temperature.temperature);
	// ROS_INFO("Humidity %f", 						msg_relativeHumidity.relative_humidity);
	ROS_INFO("Distance %d\n", 						msg_distance.data);

	ROS_INFO("cmdVel linear.x:%f angular.z:%f", 	msg_cmdVel.linear.x, msg_cmdVel.angular.z);
	ROS_INFO("cmd_wheelL:%f cmd_wheelR:%f\n", 		msg_cmd_wheelL.data, msg_cmd_wheelR.data);

	ROS_INFO("Linear acceleration: (x) %f", 		msg_acceleration.x);
	// ROS_INFO("Linear acceleration: %f, %f, %f", 	msg_acceleration.x, msg_acceleration.y, msg_acceleration.z );
	// ROS_INFO("Linear vel: %f, %f, %f", 			msg_odomVelLinear.x, msg_odomVelLinear.y, msg_odomVelLinear.z );
	// ROS_INFO("Position: %f, %f, %f\n", 			msg_odomPosition.x, msg_odomPosition.y, msg_odomPosition.z );

	ROS_INFO("Angular acceleration (z): %f",		msg_rotation.z);
	ROS_INFO("Angular acceleration: %f, %f, %f",	msg_rotation.x, msg_rotation.y, msg_rotation.z );
	ROS_INFO("Angular vel: %f, %f, %f", 			msg_odomVelAngular.x, msg_odomVelAngular.y, msg_odomVelAngular.z );
	ROS_INFO("Orientation: %f, %f, %f", 			msg_odomOrientation.x, msg_odomOrientation.y, msg_odomOrientation.z );
	ROS_INFO("---------------------------------");

	if (operationMode == OPERATION_AUTOMATIC) {
		ROS_INFO("AUTOMATIC");
		ROS_INFO("Found column: %d",		foundColumn);
		ROS_INFO("Near column: %d",			nearColumn);
		ROS_INFO("Found card: %d",			foundCard);

		if (automaticMode == AUTO_LOOKFORCOLUMN)
			ROS_INFO("AUTO_LOOKFORCOLUMN");
		else if (automaticMode == AUTO_GOTOCOLUMN)
			ROS_INFO("AUTO_GOTOCOLUMN");
		else if (automaticMode == AUTO_AROUNDCOLUMN)
			ROS_INFO("AUTO_AROUNDCOLUMN");
			else if (automaticMode == AUTO_BACKWARDS)
			ROS_INFO("AUTO_BACKWARDS");

		ROS_INFO("---------------------------------");
	}
	else
		ROS_INFO("TELEOP");
}

void teleop(ros::Publisher* pub_cmdVel)
{
	operationMode = OPERATION_TELEOP;

	if (kbhit()) {
		key = getchar();

			// --------------------------------
		// Frente
		if (key == 'W' || key == 'w') {
			msg_cmdVel.linear.x = v;
			msg_cmdVel.angular.z = 0;
		}
		// Frente horário
		else if (key == 'E' || key == 'e') {
			msg_cmdVel.linear.x = v;
			msg_cmdVel.angular.z = w;
		}
		// Frente anti-horário
		else if (key == 'Q' || key == 'q') {
			msg_cmdVel.linear.x = v;
			msg_cmdVel.angular.z = -w;
		}
		// --------------------------------
		// Parado
		else if (key == 'S' || key == 's') {
			msg_cmdVel.linear.x = 0.0;
			msg_cmdVel.angular.z = 0.0;
		}
		// Parado horário
		else if (key == 'D' || key == 'd') {
			msg_cmdVel.linear.x = 0.0;
			msg_cmdVel.angular.z = w;
		}
		// Parado anti-horário
		else if (key == 'A' || key == 'a') {
			msg_cmdVel.linear.x = 0.0;
			msg_cmdVel.angular.z = -w;
		}
		// --------------------------------
		// Trás
		else if (key == 'X' || key == 'x') {
			msg_cmdVel.linear.x = -v;
			msg_cmdVel.angular.z = 0.0;
		}
		// Trás horário
		else if (key == 'Z' || key == 'z') {
			msg_cmdVel.linear.x = -v;
			msg_cmdVel.angular.z = w;
		}
		// Trás anti-horário
		else if (key == 'C' || key == 'c') {
			msg_cmdVel.linear.x = -v;
			msg_cmdVel.angular.z = -w;
		}
		// --------------------------------

		pub_cmdVel->publish(msg_cmdVel);
	}

	printLogs();
}

void integrateSystem(double dt)
{
	// Calcula a velocidade linear a partir da integração da aceleração
	msg_odomVelLinear.x = msg_odomVelLinear.x + msg_acceleration.x * dt;
	msg_odomVelLinear.y = msg_odomVelLinear.y + msg_acceleration.y * dt;
	msg_odomVelLinear.z = msg_odomVelLinear.z + msg_acceleration.z * dt;

	// Calcula a posição a partir da integração da velocidade
	msg_odomPosition.x = msg_odomPosition.x + msg_odomVelLinear.x * dt;
	msg_odomPosition.y = msg_odomPosition.y + msg_odomVelLinear.y * dt;
	msg_odomPosition.z = msg_odomPosition.z + msg_odomVelLinear.z * dt;

	// Calcula a velocidade angular a partir da integração da aceleração angular
	msg_odomVelAngular.x = msg_odomVelAngular.x + msg_rotation.x * dt;
	msg_odomVelAngular.y = msg_odomVelAngular.y + msg_rotation.y * dt;
	msg_odomVelAngular.z = msg_odomVelAngular.z + msg_rotation.z * dt;

	// Calcula a orientação a partir da integração da velocidade angular
	msg_odomOrientation.x = msg_odomOrientation.x + msg_odomVelAngular.x * dt;
	msg_odomOrientation.y = msg_odomOrientation.y + msg_odomVelAngular.y * dt;
	msg_odomOrientation.z = msg_odomOrientation.z + msg_odomVelAngular.z * dt;
}

void automatic(ros::Publisher* pub_cmdVel, ros::Rate* loopRate,
	ros::Time* t, ros::Time* last_t)
{
	operationMode = OPERATION_AUTOMATIC;
	
	// Itera até encontrar o cartão e enquanto estiver na zona autônoma
	while (!foundCard && isColorBlack && ros::ok()) {
		ros::spinOnce();
		printLogs();
		loopRate->sleep();

		if (!foundColumn)
			automatic_lookForColumn(pub_cmdVel, loopRate);
		else if (nearColumn)
			automatic_aroundColumn(pub_cmdVel, loopRate);
		else
			automatic_toColumn(pub_cmdVel, loopRate, t, last_t);
	}
}

void automatic_lookForColumn(ros::Publisher* pub_cmdVel, ros::Rate* loopRate)
{
	automaticMode = AUTO_LOOKFORCOLUMN;

	// Itera até que a coluna esteja na frente do carro e enquanto estiver na zona autônoma
	while (!foundColumn && isColorBlack && ros::ok()) {
		ros::spinOnce();
		printLogs();
		loopRate->sleep();

		// Gira um pouquinho
		msg_cmdVel.linear.x = 0;
		msg_cmdVel.angular.z = w;
		pub_cmdVel->publish(msg_cmdVel);
		ros::Duration(0.05).sleep();

		// Para
		msg_cmdVel.linear.x = 0;
		msg_cmdVel.angular.z = 0;
		pub_cmdVel->publish(msg_cmdVel);
		ros::Duration(0.05).sleep();
	}
}

void automatic_aroundColumn(ros::Publisher* pub_cmdVel, ros::Rate* loopRate)
{	// TODO: PARTE COMPLICADA
	automaticMode = AUTO_AROUNDCOLUMN;

	msg_cmdVel.linear.x  = 0;
	msg_cmdVel.angular.z = 0;
	pub_cmdVel->publish(msg_cmdVel);

	// [*** TESTAR ***] Enquanto está perto da coluna -> gira um pouquinho, vai pra trás e gira dnv e vai pra frente
	while (nearColumn && foundColumn && msg_color.data >= 300  && ros::ok()) {
		ros::spinOnce();
		printLogs();
		loopRate->sleep();

		// Gira um pouquinho
		msg_cmdVel.linear.x = 0;
		msg_cmdVel.angular.z = w;
		pub_cmdVel->publish(msg_cmdVel);
		ros::Duration(0.05).sleep();

		// Vai pra trás
		msg_cmdVel.linear.x = -v;
		msg_cmdVel.angular.z = 0;
		pub_cmdVel->publish(msg_cmdVel);
		ros::Duration(0.3).sleep();

		// Gira ao contrário
		msg_cmdVel.linear.x = 0;
		msg_cmdVel.angular.z = -w;
		pub_cmdVel->publish(msg_cmdVel);
		ros::Duration(0.05).sleep();

		// Vai pra frente
		msg_cmdVel.linear.x = v;
		msg_cmdVel.angular.z = 0;
		pub_cmdVel->publish(msg_cmdVel);
		ros::Duration(0.3).sleep();

		// Gira um pouquinho
		msg_cmdVel.linear.x = 0;
		msg_cmdVel.angular.z = w;
		pub_cmdVel->publish(msg_cmdVel);
		ros::Duration(0.05).sleep();
	}
}

void automatic_toColumn(ros::Publisher* pub_cmdVel, ros::Rate* loopRate,
	ros::Time* t, ros::Time* last_t)
{
	automaticMode = AUTO_GOTOCOLUMN;

	// msg_cmdVel.linear.x  = 0;
	// msg_cmdVel.angular.z = 0;
	// pub_cmdVel->publish(msg_cmdVel);

	*t = ros::Time::now();
	*last_t = *t;
	float error_orientation = msg_odomOrientation.z;

	// Enquanto está longe da coluna -> vai até ela em linha reta
	while (!nearColumn && foundColumn && msg_color.data >= 300  && ros::ok()) {
		*t        = ros::Time::now();
		double dt = t->toSec() - last_t->toSec();
		*last_t   = *t;
		ros::spinOnce();
		printLogs();
		loopRate->sleep();

		msg_cmdVel.linear.x  = v;
		msg_cmdVel.angular.z = 0;
		pub_cmdVel->publish(msg_cmdVel);

	}
}