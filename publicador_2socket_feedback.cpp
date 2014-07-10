
/*************
**** Este publicador se encarga de establecer la comunicación con el servidor de feedback en arduino.
**** Posteriormente ejecuta una decodificación en los parametros y los publica en el nodo de feedback
**** para realizar las operaciones reactivas precisas
*************/
#include <iostream>
#include <sstream>

#include "/home/samper/arduino_ros/src/Librerias_2Socket.h"  //Contiene la definición de las clases de cada uno de los nodos.

using namespace std;



#define BACKLOG 10     // how many pending connections queue will hold

extern bool problemas_comunicacion; //variable para contar el numero de veces que se pierde la conexión. A partir del valor definido como perdidas consecutivas se toman medidas de seguridad.
                     //Como se ha dicho antes, el programa no comprueba por sí mismo sino que confía en la comprobación que realiza el de control.


// ---->     MAIN DEL PUBLICADOR     <---- //

int main (int argc, char **argv){

        int count = 0;
        char parar= 'N'; // char para opcion de detener la comunicación 

        pub_feedback_2servidores_coche publicador_2;

        ros::init(argc, argv, "car_feedback_pub");
        ros::NodeHandle nh; 
        ros::Rate loop_rate(10); //podemos cambiar la frecuencia. sólo tener en cuenta el tiempo de timeout de la escucha en la funcion recibir_publicar(). 
        // En principio se han mantenido los mismos valores de escucha y loop que en la parte de control. (ver fichero .h)

        if(publicador_2.iniciar_publicador(nh)==true){


            while (ros::ok())
              {
                publicador_2.recibir_publicar();

                ros::spinOnce();

                loop_rate.sleep();

              } //fin while ros ok
        
    }


    return 0;
}


// ---->     FUNCIONES DE LA CLASE  <---- //

// ---->    FUNCIONES PUBLIC       <---- //

 pub_feedback_2servidores_coche::pub_feedback_2servidores_coche(){ //constructor de la clase

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
        close(sockfd); //cerramos el cliente y "desconectamos" el publicador.
        pub.shutdown();    
    } else {
        if(   n= select(sockfd+1, &readSet,NULL,NULL,&timeout) > 0){
        for( int i;i<TAM_BUFFER_ENTRADA;i++) buffer_entrada[i]='\0'; //hacemos un flush de la variable que almacena el buffer antes de leer. 
        recv(sockfd, buffer_entrada, TAM_BUFFER_ENTRADA,0);
        ss =(string) buffer_entrada;
        if (ss.find("arranca")!= string::npos) ss="arranca";
        else if (ss.find("acelera")!= string::npos) ss="acelera";
        else if (ss.find("reduce")!= string::npos) ss="reduce";
        else if (ss.find("stop")!= string::npos) ss="stop";
        else if (ss.find("Camino Despejado")!= string::npos) ss="Camino Despejado";
        else if (ss.find("jado")!= string::npos) ss="Camino Despejado"; //si los mensajes se envian muy rápido, por el retardo entre publicaciones se sobreescribe.
        else if (ss.find("Obstaculo Detectado")!= string::npos) ss="Obstaculo Detectado";

        msg.data = ss.data();
        pub.publish(msg);
        ROS_INFO("HEMOS RECIBIDO UN DATO REALIMENTADO: %s ", ss.data());
    }


    }

}

// --->     FUNCIONES PRIVATE      <---- //

int pub_feedback_2servidores_coche::iniciar_socket_feedback(void)
{
    int numbytes;  
        char buf[TAM_BUFFER_ENTRADA];
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
        if ((rv = getaddrinfo(&ip[0], PUERTO_FEEDBACK, &hints, &servinfo)) != 0) {
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

// --> FUNCIONES ESPECIFICAS PARA LA CREACION DEL SERVIDOR <-- //


void pub_feedback_2servidores_coche::error(char *msg)
{
    perror(msg);
    exit(0);
}
