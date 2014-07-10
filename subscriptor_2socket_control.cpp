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
/************
**** Este subscriptor se encarga de transmitir todos los comandos de control (independientemente de su procedencia)
**** al coche. Para ello abre una comunicacion con el servidor de control en Arduino Mega.
**** Nombre del nodo: car_control_sub
**** Topic subscrito: car_control.
************/

#include <cstdlib>
#include <iostream>

using namespace std;

#define TAM 1

#define perdidas_consecutivas 3


#include "/home/samper/arduino_ros/src/Librerias_2Socket.h"  //Contiene la definición de las clases de cada uno de los nodos

extern bool problemas_comunicacion;  // Permite a los nodos de feedback saber si se ha producido un problema en la comunicacion con el coche.


// ---->     MAIN DEL SUBSCRIPTOR  <---- //
int main(int argc, char **argv)
  {
     subscriptor_control_coche sub;
     ros::init(argc, argv, "car_control_sub");
     ros::NodeHandle nh; 
     
     sub.iniciar_subscriptor(nh);
     ros::spin();
     return 0;
  }

// ---->     FUNCIONES DE LA CLASE  <---- //

subscriptor_control_coche::subscriptor_control_coche (void){
      timeout.tv_sec=0;
      timeout.tv_usec=tiempo_en_us_espera_OK;
      sockfd=-1;
      problemas=0;
      socket_creado=false;
      if(!iniciar_socket()){
        socket_creado=true;
        cout << " Conectado con exito"<<endl;
        problemas_comunicacion=false;

      } else {
        cout <<"\n Problemas con la creación del socket"<< endl;
        problemas_comunicacion=true;

      }
}

void subscriptor_control_coche::iniciar_subscriptor(ros::NodeHandle n) {
// cuando se usan métodos se debe especificar la funcion de clase y el objeto de clase correspondiente. 
    if( socket_creado) sub = n.subscribe("car_control", 1, &subscriptor_control_coche::controlcocheCallback,this);

}


void subscriptor_control_coche::error(char *msg)
{
    perror(msg);
    exit(0);
}

int subscriptor_control_coche::iniciar_socket()
  {  

        int numbytes;  
        char buf[TAM];
        char ip[]=IP;
        struct addrinfo hints, *servinfo;
        int rv;
        char s[INET6_ADDRSTRLEN];

        cout << "\n Abriendo socket hacia la IP:" << ip << endl;
        // creo la estructura que contendra los datos de la conexión.
        memset(&hints, 0, sizeof hints);
        hints.ai_family = AF_UNSPEC; //protocolo que puede ser ipv4 o ipv6
        hints.ai_socktype = SOCK_STREAM; //socket de stream

        //guardo la información de la dirección con que comunicar en servinfo.
        if ((rv = getaddrinfo(&ip[0], PUERTO_CONTROL, &hints, &servinfo)) != 0) {
          //en caso de error lo muestro y salgo.
          fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
          return 1;
        }

        //bucle for para comprobar si nos hemos conectado (devuelven -1 si hay algún error)
        for(p = servinfo; p != NULL; p = p->ai_next) {
          
              //creo el socket
              if ((sockfd = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) == -1) {
                //en caso de que no pueda crear el socket sigo en el bucle
                perror("client: socket");
                continue;
              }
              // me conecto al servidor
              if (connect(sockfd, p->ai_addr, p->ai_addrlen) == -1) {
                //sigo en el bucle hasta que conecte. Borro el socket creado por si hay algo mal con él.
                close(sockfd);
                perror("client: connect");
                continue;
              }
          // salimos de bucle cuando nos hayamos conectado.
          break;
        }

        //en p se almacenó el resultado de conectar, si nos devuelve un 0 es porque el servidor ha denegado la conexión o se ha cerrado el socket.
        if (p == NULL) {
          fprintf(stderr, "client: failed to connect\n");
          return 2;
        }

        freeaddrinfo(servinfo); // ya tengo el socket abierto puedo liberar esta  parte de la memoria.

        int i =2000;
        setsockopt(sockfd,SOL_SOCKET,SO_RCVTIMEO,(const char *)&i,sizeof(i));
        return 0;
  }



  void subscriptor_control_coche::controlcocheCallback(const std_msgs::String::ConstPtr& msg)
 {
   //char *msgvuelta =(char*)malloc(tam * sizeof(char));
   std::string enviado;
   std::string recibido;   
   enviado=msg->data.c_str();
   char prueba;
   ssize_t n;
  fd_set readSet;
  FD_ZERO(&readSet);
  FD_SET(sockfd, &readSet);

  ROS_INFO("Comando recibido. Le paso la orden %s a Arduino por el socket.", msg->data.c_str());

  send( sockfd, enviado.c_str(), enviado.size()+1,0 );
  n= select(sockfd+1, &readSet,NULL,NULL,&timeout);
  ROS_INFO (" Esperando acuse de recibo");

  if(n>0) {
    recv(sockfd, &prueba,1,0);
    ROS_INFO("Confirmacion Recibida");
    problemas=0;
  }
  if ( n <= 0 ) {
    problemas++;
    ROS_WARN("Problema de recepcion");
  }
  
  if(problemas>=perdidas_consecutivas){
    ROS_FATAL("Problemas graves de comunicacion. Revise el sistema.");
    problemas_comunicacion=true;
    close(sockfd);
    sub.shutdown();


  }
 
}


