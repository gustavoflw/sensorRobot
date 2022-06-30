#ifndef CINEMATICA_H
#define CINEMATICA_H

/* CINEMÁTICA INVERSA 
 *  - converte comando de velocidade para velocidade de cada motor */
void inverseKinematics(float* VL, float* VR, 
  const float L, const float R, const float vx, const float ω)
{
  /*
   *  * wheelL = velocidade da roda L [cm]
   *  * wheelR - velocidade da roda R [cm]
   *  * L = distância entre rodas     [cm]
   *  * R = raio das rodas            [cm]
   *  * vx = v.linear.x               [cm/s]
   *  * ω = v.angular.z               [rad/s]
   */
  *VL = ((ω*L)/(2*R)) + (vx/R);
  *VR = -((ω*L)/(2*R)) + (vx/R);
}

// FUNÇÃO DA CINEMÁTICA INVERSA (converter cmdVel em atuação no motor)
//void kinematics(int* wheelL, int* wheelR, geometry_msgs::Twist cmdVel)
//{
//  int wheel_default = 10;
//
//  
//  if (cmdVel.linear.x > 0) { 
//    if (cmdVel.angular.z > 0) { 
//      // Para frente && Curva horária
//      *wheelL = wheel_default;
//      *wheelR = 0;
//    }
//    else if (cmdVel.angular.z < 0) {
//      // Para frente && Curva anti-horária
//      *wheelL = 0;
//      *wheelR = wheel_default;
//    }
//    else {
//      // Para frente && Reto
//      *wheelL = wheel_default;
//      *wheelR = wheel_default;
//    }
//  }
//  
//  else if (cmdVel.linear.x < 0) {
//    if (cmdVel.angular.z > 0) { 
//      // Para trás && Curva horária
//      *wheelL = -wheel_default;
//      *wheelR = 0;
//    }
//    else if (cmdVel.angular.z < 0) { 
//      // Para trás && Curva anti-horária
//      *wheelL = 0;
//      *wheelR = -wheel_default;
//    }
//    else { 
//      // Para trás && Reto
//      *wheelL = -wheel_default;
//      *wheelR = -wheel_default;
//    }
//  }
//  else {
//    *wheelL = 0;
//    *wheelR = 0;
//  }
//}

#endif
