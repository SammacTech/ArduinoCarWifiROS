 /*  
   @Developer José Luis Samper Escudero <Jluisse@gmail.com>, 
	      Joaquín Macanás Valera <macanas_92@hotmail.com
   @date   Thu 10 Jul 2014 10:00:00


    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

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
