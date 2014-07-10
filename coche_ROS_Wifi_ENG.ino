/*
This program establishes a wireless communication between an 
arduino-controlled car and a ROS system.

Requisites:

*   ROSsystem
*   Arduino Mega
*   Arduino Wifi Shield
*   Wifi local net.

Functional description:

The communication is created by using 2 servers on arduino: one manages the control
and the other holds the feedback. In this way, the control server receives
the orders from the ROS System, meanwhile, the feedback server sends 
the sensorial data to it.

The movement algorithm is extremely simple, the car will move forwards
when the ROS system commands it . When an obstacle is detected, the car
will move backwards and then will rotate until the obstacle is not detected.
If the duration of obstacle detection is higher than the time the car needs
to turn around 360º , the car will move backwards again and will repeat
the process. Car's rotation direction is alternative.

Developers:

  *   Joaquin Macanas Valera
  *   Jose Luis Samper Escudero

*/
#include <SPI.h>
#include <WiFi.h>
#include <Servo.h>

#define TAM 30
#define distancia_seguridad 300
#define TIEMPO_GIRAR_90 1600 //time, in ms, the car needs for spinning 90º
#define TIEMPO_PAUSA_TRAS_GIRO 100 //delay time after turning.
#define puertoROS 79      // Ros feedback port
#define PinServo 44
#define TIEMPO_MARCHA_ATRAS 800 //amount of time the car will be moving backwards
#define INICIOSERVO 90 //Servo's position that lets the car moving forwards.
#define num_velocidades 4


//-->  Variables del Wifi shield  <-- //

char ssid[] = "sammac";      // your network SSID (name) 
char pass[] = "sammac";   // your network password
int keyIndex = 0;                 // your network key Index number (needed only for WEP)
int status = WL_IDLE_STATUS;
WiFiServer server(80);
WiFiServer FeedbackServer( puertoROS );
//-->  Variables Intermedias <--- //

boolean cliente;
int datos;
char buffer[TAM];
String comando;
char buffersalida[TAM];
boolean vuelvo_de_giro=false;
boolean baliza_inicio=true;
boolean obstaculo;
boolean estado_obstaculo_anterior;
boolean marcha_atras_=false;

//-->  Variables del coche   <---//
 
int num_obstaculos=0; 
int actMot[4]={A1, A2, A3, A4};
int motD =5, motI = 6;
int motDval = 0, motIval = 0;
int sensorPin = A0, sensorValue = 0;
int LED = 8;
boolean on_off = false;
boolean ultimoGiro = false; //false=left | true=right
boolean avanzado = false; //¿Have the rotation stopped?
int velocidad[]={180,200,220,255};
int select=0; //Speed selector. Values [0,1,2]
Servo myservo;


void setup() {
  //Initialize serial and wait for port to open:  
  Serial.begin(9600);
  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present"); 
    // don't continue:
    while(true);
  } 

  // attempt to connect to Wifi network:
  while ( status != WL_CONNECTED) { 
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:    
    status = WiFi.begin(ssid);

    // wait 10 seconds for connection:
    delay(10000);
  } 
  server.begin();
  FeedbackServer.begin();
  // you're connected now, so print out the status:
  printWifiStatus();

  // Conectamos con el nodo de ROS al que enviar el estado del robot.

   Serial.println(" Servidores Inciados. Recuerde: \n\t Servidor de control en el puerto 80. ");
   Serial.print ("\t Servidor de feedback en el puerto ");
   Serial.println(puertoROS);


  //--> Output motor pinsr <--//

  pinMode(actMot[0], OUTPUT); //Right Motor 
  pinMode(actMot[1], OUTPUT); //Right Motor 
  pinMode(actMot[2], OUTPUT); //Left Motor 
  pinMode(actMot[4], OUTPUT); //Left Motor 
  pinMode(motD, OUTPUT);
  pinMode(motI, OUTPUT);
  pinMode(LED, OUTPUT);

  //Motor initilization
  digitalWrite(motD, LOW);
  digitalWrite(motI, LOW);
  digitalWrite(actMot[0],LOW);
  digitalWrite(actMot[1],LOW);
  digitalWrite(actMot[2],LOW);
  digitalWrite(actMot[3],LOW);

  // Secure start up.
  comando = "stop";
  obstaculo=false;
  estado_obstaculo_anterior=true;
  myservo.attach(PinServo);
  myservo.write(INICIOSERVO);
}


void loop() {

  // listen for incoming clients
  WiFiClient client = server.available();
  WiFiClient Feedback=FeedbackServer.available();
  cliente=true;
  if (client && Feedback) {
    //when a clients connects for first time, print it. 
    if(cliente==true) Serial.println("new client");

    while (client.connected() && Feedback.connected()) {
      // while the clients are connected


      datos=0;
      //if there are available data in the buffer we read it
      while (client.available()) {
        if(datos==0) client.write('1');
        buffer[datos]=client.read();
        datos++; 

      }
      //if we have read some data, print it and decode it
      if(datos >0){
        buffer[datos]='\0';

        comando=(String) buffer; 

        Feedback.write(" Buffer Leido.");
        Feedback.print (buffer);



        // Decodification code.

        if (comando.equals("stop")){
          on_off = false;
          digitalWrite(actMot[1],LOW);
          digitalWrite(actMot[3],LOW);
        }
        if (comando.equals("arranca")){
          on_off = true;
          Serial.println("arranco");      
          digitalWrite(actMot[1],HIGH);
          digitalWrite(actMot[3],HIGH);
        }
        if (comando.equals("acelera") && select< num_velocidades - 1){
          select++;
          Serial.println("velocidad: ");
          Serial.println(velocidad[select]);
        }
        if (comando.equals("reduce") && select>0){
          select--;
          Serial.println("velocidad: ");
          Serial.println(velocidad[select]);
        }

      } // end of decodification.
      
      // move the robot
      a_funcionar();
      // feedback
      
      if (on_off){
        if(obstaculo && obstaculo!=estado_obstaculo_anterior) {
          Serial.println("Obstaculo Detectado");
          Feedback.write("Obstaculo Detectado");
          estado_obstaculo_anterior=obstaculo;
        }
        else if( !obstaculo && obstaculo!=estado_obstaculo_anterior){
          Serial.println("Camino Despejado");

          Feedback.write("Camino Despejado");
          estado_obstaculo_anterior=obstaculo;
        }
      }

    } // client disconnected
    cliente=false;
    // stop the car if we disconnect
    on_off = false;
    digitalWrite(actMot[1],LOW);
    digitalWrite(actMot[3],LOW);
    // give the web browser time to receive the data
    delay(1);

    // close the connection:
    client.stop();
    Serial.println("Cliente Desconectado");
  }
}


void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}


void mover(){
  Serial.print("Envio a mover en mover: ");
  Serial.println(motDval);
  Serial.println(motIval);
  Serial.print(" actMot 0: ");
  Serial.println(digitalRead(actMot[0]));
  Serial.print(" actMot 1: ");
  Serial.println(digitalRead(actMot[1]));
  Serial.print(" actMot 2: ");
  Serial.println(digitalRead(actMot[2]));
  Serial.print(" actMot 3: ");
  Serial.println(digitalRead(actMot[3]));  
  analogWrite(motD,motDval);
  analogWrite(motI,motIval);
}

void a_funcionar(){
  String cadena;
  // if we are connected to ROS, read the proximity sensor
  if (on_off){
    //if car movement is enabled, read the sensor
    sensorValue = analogRead(sensorPin);
    Serial.println(sensorValue);
    // if this value is inside our secure zone, turn around.
    if (sensorValue > distancia_seguridad ){
 
      obstaculo=true;


      digitalWrite(LED, HIGH);        
      avanzado=false; 
      num_obstaculos++;
      if( num_obstaculos > 2) marcha_atras_=false;
      if( !marcha_atras_) {
        marcha_atras(); //moce backwards if this is the first time we rotate in this direction or we have spin 360 º.
        marcha_atras_=true;
        num_obstaculos=0;
      }

      // double if for choosing the alternative rotation. First if left-rotation, second right-rotation
      if (ultimoGiro && !avanzado){

        digitalWrite(actMot[0],LOW);
        digitalWrite(actMot[1],LOW);

        myservo.write(55);
        motDval=velocidad[select];
        motIval=velocidad[select];        
        mover();
        delay(TIEMPO_GIRAR_90);

        //After rotating, stop the motors (avoid inertial issues)

        motDval = 0;
        motIval = 0;          
        mover();
        digitalWrite(actMot[0],LOW);
        digitalWrite(actMot[1],HIGH);

        ultimoGiro=true;
        vuelvo_de_giro=true;
        delay(TIEMPO_PAUSA_TRAS_GIRO); //delay time            

      }
      if (!ultimoGiro && !avanzado){

        digitalWrite(actMot[2],LOW);
        digitalWrite(actMot[3],LOW);
        myservo.write(125);
        motDval=velocidad[select];
        motIval=velocidad[select];         
        mover();
        delay(TIEMPO_GIRAR_90);
                   
        motDval = 0;
        motIval = 0;          
        mover();

        digitalWrite(actMot[2],LOW);
        digitalWrite(actMot[3],HIGH);
        ultimoGiro=false;
        vuelvo_de_giro=true;
        delay(TIEMPO_PAUSA_TRAS_GIRO);
      }
    }
    // move forwards if we havent detected any obstacle
    else{
      myservo.write(INICIOSERVO);

      num_obstaculos=0;
      obstaculo=false;
      marcha_atras_=false;
      digitalWrite(LED, LOW);
      motDval=velocidad[select];
      motIval=velocidad[select];


      mover();
      avanzado=true;
      if(vuelvo_de_giro){
        ultimoGiro=!ultimoGiro;
        vuelvo_de_giro=false;
      }

    }
  } 
  //stop the motors
  else {
    if(baliza_inicio){
      motDval = 0;
      motIval = 0;          
      mover();
      baliza_inicio=false;
    }
  }
}


void marcha_atras(){

myservo.write(INICIOSERVO);
digitalWrite(actMot[1],LOW);
digitalWrite(actMot[3],LOW);

digitalWrite(actMot[0],HIGH);
digitalWrite(actMot[2],HIGH);

 motDval=velocidad[select];
 motIval=velocidad[select];         
 mover();
 delay(TIEMPO_MARCHA_ATRAS);
 
digitalWrite(actMot[0],LOW);
digitalWrite(actMot[2],LOW);
digitalWrite(actMot[1],HIGH);
digitalWrite(actMot[3],HIGH);
}
