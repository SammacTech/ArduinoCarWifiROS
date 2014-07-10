/****************
**** Este publicador se encarga de publicar los comandos
**** escritor desde la interfaz HMI en el topic de control
**** Topic de publicacion: car_control
**** Nombre del nodo en el sistema: car_control_pub
*****************/


#include "ros/ros.h"

#include "std_msgs/String.h"

#include <iostream>
#include <sstream>

 using namespace std;


int main(int argc, char **argv)
{
  /**
   * La función ros::init() es necesario ejectuarla en primer lugar si deseamos interactuar con el sistema ROS
   */

  ros::init(argc, argv, "car_control_pub");


  /**
   * Declaramos un nodehandle denominado nh, encargado de
   * gobernar la comunicación.
   */

  ros::NodeHandle nh;


  /**
   * Definimos un publicador de dicha comunicación así como el tipo de mensaje 
   * que envía.
   */

  ros::Publisher pub = nh.advertise<std_msgs::String>("car_control",1, true);
  ros::Rate loop_rate(10);


  /**
   *  Cuenta del número de mensajes enviados.
   */

  int count = 0;
  bool inicio=true; //booleano para determinar el mensaje de espera de subscriptores
  char parar= 'N'; // char para opcion de detener la comunicación 
  bool cya_subscribers=false;  // para detener cuando me quedo sin subscriptores

  while (ros::ok() && !cya_subscribers)
  {

    /**
     * cuerpo del mensaje que publicará pub en el topic escogido.
     */

    std_msgs::String msg;
    

    // Rellenamos el mensaje
    std::string ss;
    
    /*Enviamos los diferentes comandos*/
    cout <<" Opciones de control: \n\t---> arranca\n\t---> acelera \n\t---> reduce\n\t---> stop \n\t---> salir"<<endl;
    cout <<" Introduzca un comando: ";
    cin >> ss;
    if(ss.compare("salir")!=0){
    msg.data = ss.data(); 



    /**
     * Publicamos el mensaje en el topic.
     */

    while (pub.getNumSubscribers()<1){   // esperamos subscriptores, en caso de desaparecer a media ejecucion se da la opcion
					//  de terminar.Poner 2 si usamos el Scada de Arduino UNO, 2 si no lo estamos usando.
         if(inicio) {
                cout << "Esperando Subscriptores."<<endl;
                inicio =false;
          }
          cout << "El subscriptor ha perdido el socket de comunicacion.\n Se encuentra bloqueado buscandolo.\n Desea parar (S/N)"<<endl;
          cin >> parar;
          if (parar=='s' || parar=='S') {
            cya_subscribers=true;
            system("rosnode kill car_control_sub");
            system ("rosnode kill car_feedback_pub");
            pub.shutdown();  
            break;
          }
    }
    if( !cya_subscribers) pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();

    count++;
    if (count==3)
      count=0;
    }//fin if
    else {
      ROS_WARN(" INICIADO PROTOCOLO DE FINALIZACION" );
      ROS_WARN(" CERRANDO EL SISTEMA... ");
      system("rosnode kill car_control_sub");
      system ("rosnode kill car_feedback_pub");
      msg.data = "salir"; 
      pub.publish(msg);
      ros::spinOnce();
      loop_rate.sleep();
      ROS_WARN("FINALIZACION REALIZADA.");
      pub.shutdown(); 
      break;
    }
  } //fin while ros ok


  return 0;
}


