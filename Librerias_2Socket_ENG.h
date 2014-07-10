
#include "ros/ros.h"
#include "std_msgs/String.h"
/******************
**** 	The main purpose of this libary is grouping all the functions, classes and special variables trying to make easier
****    undertand the global process.
****	
******************/

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

#define PUERTO_FEEDBACK "79"  // Feedback's Arduino's Port.
#define PUERTO_CONTROL "80"   // Control's Arduino's Port.
#define IP "192.168.2.3"
#define TAM_BUFFER_ENTRADA 31  //size of our buffer.
#define tiempo_en_us_espera_OK 1000000  //Max time for accepting the OK from arduino after a sending.


class pub_feedback_coche {  //This class is still being developed. It tries to create the feedback server on the PC.
			    // However we have noticed that the proccess works better if both server are on Arduino.

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
        void iniciar_subscriptor(ros::NodeHandle n);  //starts the subscriber
        int iniciar_socket();	//starts the communication
        void controlcocheCallback (const std_msgs::String::ConstPtr& msg);  //subscriptor's related function
        int* get_problemas() { return &problemas; }; //export the variable called problemas
    protected:
        ros::Subscriber sub;
        //Confirmation waiting time structure
        struct timeval timeout;

    private:
        void error(char *msg); 
        int sockfd;
        struct addrinfo *p; 
        int problemas; //this variable keeps the count of the number of consecutive failed sendings.
        bool socket_creado;
};

class pub_feedback_2servidores_coche {

public:

    pub_feedback_2servidores_coche();
    bool iniciar_publicador(ros::NodeHandle n); //creates the publisher
   // void esperar_cliente(); //waits a client to connect. Is not developed
    bool recibir_publicar(); //recibes feedback info and publishes it. 

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
