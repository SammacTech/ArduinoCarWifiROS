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
/*************
**** This publisher establishes the communication with the feedback server located on the arduino mega
**** that controls the car.
**** When the publisher recieves information from the server, it decodes the information and publishes
**** the received parameters on the feedback topic.
*************/
#include <iostream>
#include <sstream>

#include "/home/samper/arduino_ros/src/Librerias_2Socket.h"  //this library contains all the classes that have been developed for this project. 

using namespace std;



#define BACKLOG 10     // how many pending connections queue will hold

extern bool problemas_comunicacion; // Count the number of consecutive disconnections or failed packages. If this number is higher than the secure value, the publisher stops.


// ---->     MAIN DEL PUBLICADOR     <---- //

int main (int argc, char **argv){

        int count = 0;
        char parar= 'N'; // stop selector

        pub_feedback_2servidores_coche publicador_2;

        ros::init(argc, argv, "car_feedback_pub");
        ros::NodeHandle nh; 
        ros::Rate loop_rate(10); //If we want to change this frecuency we must keep in mind the other timeouts and time-based responses, especially at recibir_publicar() function. 


        if(publicador_2.iniciar_publicador(nh)==true){

            while (ros::ok())
              {
                publicador_2.recibir_publicar();

                ros::spinOnce();

                loop_rate.sleep();

              } //end while ros ok
        
    }


    return 0;
}


// ---->     CLASS FUNCTIONS       <---- //

// ---->     PUBLIC FUNCTIONS       <---- //

 pub_feedback_2servidores_coche::pub_feedback_2servidores_coche(){ //class constructor

    timeout.tv_sec=0;
    timeout.tv_usec=tiempo_en_us_espera_OK;   
    sockfd=-1;
    socket_creado=false;
    if(!iniciar_socket_feedback()){
        socket_creado=true;
        cout << " Conectado con exito"<<endl;
      } else {
        cout <<"\n Problemas con la creación del socket"<< endl;
      }

 }

bool pub_feedback_2servidores_coche::iniciar_publicador(ros::NodeHandle n){

      if(socket_creado) pub = n.advertise<std_msgs::String>("car_feedback",1, true);

}

bool pub_feedback_2servidores_coche::recibir_publicar(void){
    
    int n;
    fd_set readSet;
    FD_ZERO(&readSet);
    FD_SET(sockfd, &readSet);

    if (problemas_comunicacion){
        ROS_FATAL(" Se ha producido un problema en la comunicacion. Ejecucion Suspendida prematuramente. ");
        ROS_FATAL(" Revise el estados de los nodos encargados del control del coche");
        close(sockfd); //stop the client and the publisher
        pub.shutdown();    
    } else {
        if(   n= select(sockfd+1, &readSet,NULL,NULL,&timeout) > 0){
        for( int i;i<TAM_BUFFER_ENTRADA;i++) buffer_entrada[i]='\0'; //clean the buffer before reading. 
        recv(sockfd, buffer_entrada, TAM_BUFFER_ENTRADA,0);
        ss =(string) buffer_entrada;
        if (ss.find("arranca")!= string::npos) ss="arranca";
        else if (ss.find("acelera")!= string::npos) ss="acelera";
        else if (ss.find("reduce")!= string::npos) ss="reduce";
        else if (ss.find("stop")!= string::npos) ss="stop";
        else if (ss.find("Camino Despejado")!= string::npos) ss="Camino Despejado";
        else if (ss.find("jado")!= string::npos) ss="Camino Despejado"; //We have noticed that Mega's sending speed makes the system 
									// lose some data due to our read-socket function. If you can avoid fast-consecutive sends. 
									// This problem will be solved in the next version.
        else if (ss.find("Obstaculo Detectado")!= string::npos) ss="Obstaculo Detectado";

        msg.data = ss.data();
        pub.publish(msg);
        ROS_INFO("HEMOS RECIBIDO UN DATO REALIMENTADO: %s ", ss.data());
    }


    }

}

// --->     PRIVATE FUNCTIONS      <---- //

int pub_feedback_2servidores_coche::iniciar_socket_feedback(void)
{
    int numbytes;  
        char buf[TAM_BUFFER_ENTRADA];
        char ip[]=IP;
        struct addrinfo hints, *servinfo;
        int rv;
        char s[INET6_ADDRSTRLEN];

        cout << "\n Abriendo socket hacia la IP:" << ip << endl;
        // Structure that will contain our connection information.
        memset(&hints, 0, sizeof hints);
        hints.ai_family = AF_UNSPEC; // communication protocol, ipv4 or ipv6
        hints.ai_socktype = SOCK_STREAM; // stream socket

        //save the information in servinfo.
        if ((rv = getaddrinfo(&ip[0], PUERTO_FEEDBACK, &hints, &servinfo)) != 0) {
          //if occurs some error, print it and close the program.
          fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
          return 1;
        }

        //check the state of the connection
        for(p = servinfo; p != NULL; p = p->ai_next) {
          
              //socket is created
              if ((sockfd = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) == -1) {
                //Managing the error.
                perror("client: socket");
                continue;
              }
              // try to establish the connection
              if (connect(sockfd, p->ai_addr, p->ai_addrlen) == -1) {
                
                close(sockfd);
                perror("client: connect");
                continue;
              }
          // stop the loop if we have connected
          break;
        }

        //check if the connections has been succesfull.
        if (p == NULL) {
          fprintf(stderr, "client: failed to connect\n");
          return 2;
        }

        freeaddrinfo(servinfo); // once the socket is created, we dont need information for its creation.

        int i =2000;
        setsockopt(sockfd,SOL_SOCKET,SO_RCVTIMEO,(const char *)&i,sizeof(i));
        return 0;
}

// --> Specific socket creation function       <-- //


void pub_feedback_2servidores_coche::error(char *msg)
{
    perror(msg);
    exit(0);
}
