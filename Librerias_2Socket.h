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

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <boost/ref.hpp>

#include <cstdlib>
#include <iostream>

#include <unistd.h>
#include <errno.h>
#include <string>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/wait.h>
#include <signal.h>

using namespace std;

bool problemas_comunicacion;

#define PUERTO_FEEDBACK "79"  // Puerto en el que se encuentra el servidor de feedback de arduino.
#define PUERTO_CONTROL "80"
#define IP "192.168.2.3"
#define TAM_BUFFER_ENTRADA 31
#define tiempo_en_us_espera_OK 1000000  //tiempo de espera para recibir la confirmacion de comando recibido por parte de arduino

// Por no masificar el numero de archivos a consultar para determinar el funcionamiento el programa se ha estructurado en un .h con las cabeceras y el .cpp con el main y definicion de funciones
// relativas a cada nodo.

class pub_feedback_coche {  //Esta clase aún no se ha implementado en el código ya que se ha comprobado que el funcionamiento es mejor con los 2 servidores en arduino.

public:

    pub_feedback_coche();
    bool iniciar_publicador(ros::NodeHandle n); //crea al publicador siempre y cuando el socket se haya creado adecuadamente
    void esperar_cliente(); //simplemente espera a recibir un cliente
    bool recibir_publicar(); //recibe y publica en el topic. Puesto que es solo de informacion y hay un doble socket, no se comprueban los errores de conexion en esta comunicacion (seria redundante). 

protected:
    ros::Publisher pub;
    char buffer_entrada[TAM_BUFFER_ENTRADA];


private:
    int iniciar_servidor_feedback(void);


    int sockfd;
    int  new_fd;  // listen on sock_fd, new connection on new_fd
    bool socket_creado;

    
    // del socket
    void sigchld_handler(int s);
    void* get_in_addr(struct sockaddr *);


    struct sockaddr_storage their_addr; // connector's address information
    socklen_t sin_size;
    struct sigaction sa;
    char s[INET6_ADDRSTRLEN];
    std::string ss;
    std_msgs::String msg;
 
};


class subscriptor_control_coche {

    public:

        subscriptor_control_coche ();
        void iniciar_subscriptor(ros::NodeHandle n);
        int iniciar_socket();
        void controlcocheCallback (const std_msgs::String::ConstPtr& msg);
        int* get_problemas() { return &problemas; };
    protected:
        ros::Subscriber sub;
        //estructura para el tiempo que espero la confirmacion
        struct timeval timeout;

    private:
        void error(char *msg); 
        int sockfd;
        struct addrinfo *p; 
        int problemas; //variable para contar el numero de veces que se pierde la conexión. A partir del valor definido como perdidas consecutivas se toman medidas de seguridad.
        bool socket_creado;
};

class pub_feedback_2servidores_coche {

public:

    pub_feedback_2servidores_coche();
    bool iniciar_publicador(ros::NodeHandle n); //crea al publicador siempre y cuando el socket se haya creado adecuadamente
    // void esperar_cliente(); //simplemente espera a recibir un cliente. Esta funcion se utilizaria en caso de estar el servidor alojado en el PC
    bool recibir_publicar(); //recibe y publica en el topic. Puesto que es solo de informacion y hay un doble socket, no se comprueban los errores de conexion en esta comunicacion (seria redundante). 

protected:
    ros::Publisher pub;
    char buffer_entrada[TAM_BUFFER_ENTRADA];
    struct timeval timeout;


private:
    int iniciar_socket_feedback(void);
    void error(char *msg);

    int sockfd;
    bool socket_creado;
    struct addrinfo *p; 
    std::string ss;
    std_msgs::String msg;
 
};

class sub_feedback_coche {

public: 

    sub_feedback_coche (ros::NodeHandle nh);
    void callbackFeedback(const std_msgs::String::ConstPtr& msg);
protected:

    ros::Publisher Feedback_A_Control;
    ros::Subscriber Sub_Feedback;    

};
