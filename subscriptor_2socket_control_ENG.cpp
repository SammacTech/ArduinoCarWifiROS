/************
**** This subscriptor sends all the control commands published at car_control's topic to Arduino Mega. 
**** For achieving this purpose, the subscriptor creates a socket with the control server in the car.
**** Node's Name: car_control_sub
**** Topic : car_control.
************/

#include <cstdlib>
#include <iostream>

using namespace std;

#define TAM 1

#define perdidas_consecutivas 3


#include "/home/samper/arduino_ros/src/Librerias_2Socket.h"  //Contains all the functions and classes that have been developed for this project.

extern bool problemas_comunicacion; 

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

// ---->     CLASS FUNCTIONS  <---- //

subscriptor_control_coche::subscriptor_control_coche (void){  //CONSTRUCTOR
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
        // structure that contains all the connection info.
        memset(&hints, 0, sizeof hints);
        hints.ai_family = AF_UNSPEC; //protocol, ipv4 or ipv6
        hints.ai_socktype = SOCK_STREAM; //stream socket

        //save the address info in servinfo.
        if ((rv = getaddrinfo(&ip[0], PUERTO_CONTROL, &hints, &servinfo)) != 0) {
          //en caso de error lo muestro y salgo.
          fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
          return 1;
        }

	// connection attempt loop
        for(p = servinfo; p != NULL; p = p->ai_next) {
          

              if ((sockfd = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) == -1) {
		//managing the errors.
                perror("client: socket");
                continue;
              }
              // establish the connection with the server
              if (connect(sockfd, p->ai_addr, p->ai_addrlen) == -1) {
		//managing the errors.
                close(sockfd);
                perror("client: connect");
                continue;
              }
          // loop end
          break;
        }

        //check if the connection has been successfull.
        if (p == NULL) {
          fprintf(stderr, "client: failed to connect\n");
          return 2;
        }

        freeaddrinfo(servinfo); 
        int i =2000;
        setsockopt(sockfd,SOL_SOCKET,SO_RCVTIMEO,(const char *)&i,sizeof(i));
        return 0;
  }



  void subscriptor_control_coche::controlcocheCallback(const std_msgs::String::ConstPtr& msg)
 {

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


