/**********
****	A modo de ejemplo, este subscriptor da un tratamiento a la informacion de feedback.
****	En nuestro caso, publicamos la informacion en el nodo de control.
**********/

#include <cstdlib>
#include <iostream>

#include "/home/samper/arduino_ros/src/Librerias_2Socket.h"  //Contiene la definición de las clases de cada uno de los nodos.


int main(int argc, char **argv)
  {
     ros::init(argc, argv, "feedback_sub");
     // puesto que solo hay cuatro nodos podemos encargar toda la tarea de comunicación al mismo.
     ros::NodeHandle nh;   
     sub_feedback_coche sub(nh);

     ros::spin();

   return 0;
  }

sub_feedback_coche::sub_feedback_coche(ros::NodeHandle nh){
// cuando se usan métodos se debe especificar la funcion de clase y el objeto de clase correspondiente. 
       ros::Subscriber Sub_Feedback = nh.subscribe("car_feedback", 1, &sub_feedback_coche::callbackFeedback,this);
  	   ros::Publisher Feedback_A_Control = nh.advertise<std_msgs::String>("car_control",1, true);

   }

// Funcion que da tratamiento al dato recibido y enviado por el publicador encargado de la realimentacion

void sub_feedback_coche::callbackFeedback(const std_msgs::String::ConstPtr& msg){
	//añadir el tratamiento de la informacion que sea necesario.
	Feedback_A_Control.publish(msg);
}
