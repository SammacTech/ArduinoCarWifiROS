/****************
**** The aim of this publisher is publishing the commands
**** written in the HMI interface at the control topic.
**** The topic mentioned above is called car_control
**** The name of this publisher in the ROS system is car_control_pub
*****************/


#include "ros/ros.h"

#include "std_msgs/String.h"

#include <iostream>
#include <sstream>

 using namespace std;



int main(int argc, char **argv)
{

// In first place, we must give a name to the publisher and say which node will handle the node's communication.
  ros::init(argc, argv, "car_control_pub");

  ros::NodeHandle nh;


  /**
   * Create the publisher and specify the related topic.
   */

  ros::Publisher pub = nh.advertise<std_msgs::String>("car_control",1, true);
  ros::Rate loop_rate(10);


  /**
   *  count the number of msgs.
   */

  int count = 0;
  bool inicio=true; 
  char parar= 'N'; // lets the user decide between continue the program or end it if a problem is detected.
  bool cya_subscribers=false;  // tells the system if the subscribers are still active or not.

  while (ros::ok() && !cya_subscribers)
  {


    std_msgs::String msg;
    
    std::string ss;
    
    /*print the commands*/
    cout <<" Opciones de control: \n\t---> arranca\n\t---> acelera \n\t---> reduce\n\t---> stop \n\t---> salir"<<endl;
    cout <<" Introduzca un comando: ";
    cin >> ss;
    if(ss.compare("salir")!=0){
    msg.data = ss.data(); 



    /**
     * publish the message/command.
     */

    while (pub.getNumSubscribers()<2){  //wait all subscribers. Set to 1 if we are not using the Arduino Uno Scada. Set to 2 if we are using it.
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


