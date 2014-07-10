/*

This sketch complements the wireless arduino-based car
project. Its main purpose is emulating a 'SCADA System'.
In this way, it gives information about the system by turning
on and off several leds. In addition, this sketch shows 
how to use ROSserial Libraries.

Requisites:
*  Arduino Uno
*  ROS remotely Controlled Car System.
*  Rosserial Libraries.
*  ROS Master's Node  (Roscore).
*  1 Led RGB
*  1 Led rojo
*  1 Led Verde
*  4 buttons.
*  ROS's Nodes used by this sketch.
    *   ROS Control's Publisher.
    *   ROS Control's Subscriber.
    *   ROS Feedback's Subscriber.

Developers:
 *  Joaquin Macanas Valera
 *  Jose Luis Samper Escudero

*/
#include <ros.h>
#include <std_msgs/String.h>

#define RGB_R 3
#define RGB_V 5
#define RGB_A 6
#define ARRANQUE 2
#define OBSTACULO 4

#define ARRANCA 8
#define ACELERA 9
#define REDUCE 10
#define PARAR 11
#define TAM 7

int on_off;
int select;
int velocidades[]={180,200,220,255};
int colores[TAM][3]={{0,128,255},{255,128,0},{255,0,0},{102,0,204}};

ros::NodeHandle na;

std_msgs::String msg_control_arduino;

void Feedback (const std_msgs::String& Feedback_msg){
 String ss;
 ss=Feedback_msg.data;

 if (ss.equals("Obstaculo Detectado")) {
   digitalWrite(OBSTACULO, HIGH);
   na.loginfo("  OBSTACULO ENCONTRADO");
 }
 else if(ss.equals("Camino Despejado")){
   digitalWrite(OBSTACULO,LOW);
     na.loginfo("  CAMINO DESPEJADO");
 } 
}

void Control (const std_msgs::String & Control_msg){
  String ss=Control_msg.data;
  
  if( ss.equals("arranca") && !on_off){

    na.loginfo("VEHICULO EN MOVIMIENTO");    
    on_off=true;
    digitalWrite(ARRANQUE, HIGH);
    encender_RGB(colores[select][0],colores[select][1],colores[select][2]);

  }else if (ss.equals("acelera") && select <TAM && on_off) {

    na.loginfo("VEHICULO ACELERANDO");    
    if(select<TAM-1) select++;
    else na.loginfo("VAS A TODA MECHA");
    encender_RGB(colores[select][0],colores[select][1],colores[select][2]);
  
  }else if (ss.equals("reduce") && select>=0 && on_off){
 
    na.loginfo("REDUCCION DE VELOCIDAD APLICADA");   
    if(select>0) select--;
    else na.loginfo("SOLO TE FALTA PARAR EL COCHE");
    encender_RGB(colores[select][0],colores[select][1],colores[select][2]);
    
  }else if ((ss.equals("stop")||ss.equals("salir")) && on_off){
 
    na.loginfo("VEHICULO DETENIDO");    
    on_off=false;
    digitalWrite(ARRANQUE, LOW);
    encender_RGB(96,96,96);
    
  }

}

ros::Publisher arduino_control("car_control", & msg_control_arduino);
ros::Subscriber<std_msgs::String> arduino_feedback("car_feedback", Feedback);
ros::Subscriber<std_msgs::String> arduino_sub_control("car_control", Control);

void setup() {
  

  pinMode(RGB_R,OUTPUT);
  pinMode(RGB_V,OUTPUT);
  pinMode(RGB_A,OUTPUT);
  pinMode(OBSTACULO,OUTPUT);
  pinMode(ARRANQUE, OUTPUT);
  
  pinMode(ARRANCA, INPUT);
  pinMode(ACELERA, INPUT);
  pinMode(REDUCE,  INPUT);
  pinMode(PARAR,   INPUT);
  
  digitalWrite( RGB_R, 96);
  digitalWrite( RGB_V, 96);
  digitalWrite( RGB_A, 96);
  digitalWrite( OBSTACULO, LOW);
  digitalWrite( ARRANQUE, LOW);
  
  int select=0;
  na.initNode();
  na.advertise (arduino_control );
  na.subscribe (arduino_feedback);
  na.subscribe (arduino_sub_control);
  
}

void loop (){
  
  if ( digitalRead( ARRANCA) && !on_off) {
    na.loginfo("COMANDO ENVIADO: ARRANCA");
    msg_control_arduino.data="arranca";
    arduino_control.publish(&msg_control_arduino);
    while(digitalRead( ARRANCA ));
    delay(1000);
  }else if(digitalRead(ACELERA) && on_off && select < TAM) {
    
    na.loginfo("COMANDO ENVIADO: ACELERA");    
    msg_control_arduino.data="acelera";    
    arduino_control.publish(&msg_control_arduino);
    while(digitalRead(ACELERA));
    delay(1000);    
  }else if(digitalRead(REDUCE) && on_off && select >= 0) {
 
    na.loginfo("COMANDO ENVIADO: REDUCE");    
    msg_control_arduino.data="reduce";
    arduino_control.publish(&msg_control_arduino);
    while(digitalRead( REDUCE ));
    delay(1000);    
  }else if(digitalRead(PARAR) && on_off) {

    na.loginfo("COMANDO ENVIADO: PARAR");    
    msg_control_arduino.data="parar";
    arduino_control.publish(&msg_control_arduino);
    while(digitalRead( PARAR ));
    delay(1000);    
  }
  
  
  
  na.spinOnce();
  
}

void encender_RGB( int rojo, int verde, int azul){

  analogWrite(RGB_R,255-rojo);
  analogWrite(RGB_V,255-verde);
  analogWrite(RGB_A,255-azul);  
}
