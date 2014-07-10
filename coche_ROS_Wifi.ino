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
/*
Este programa permite comunicar, de forma inalambrica un coche controlado por arduino con
un sistema ROS,. 

Requisitos:

*   ROSsystem
*   Arduino Mega
*   Arduino Wifi Shield
*   Red wifi.

Descripcion de funcionamiento:

Para la comunicacion ROS se establecen dos servidores en arduino; uno de control y otro 
de realimentacion. Por el primero, recibiremos las ordenes de control y por el segundo
el coche enviara los datos realimentados.

El algoritmo de movimiento es muy sencillo, el coche avanzara cuando el sistema ROS lo 
ordene. En el momento en el que encuentre un obstaculo, dara marcha atras y girara
hasta que deje de detectar obstaculo. Si el obstaculo se detecta durante un tiempo
superior al necesario para girar 360º vuelve a dar marcha atras y repite el proceso.
Los giros son alternos. 

Desarrolladores:

  *   Joaquin Macanas Valera
  *   Jose Luis Samper Escudero
 
 */

#include <SPI.h>
#include <WiFi.h>
#include <Servo.h>

#define TAM 30
#define distancia_seguridad 300
#define TIEMPO_GIRAR_90 1600 //tiempo en ms que tarda el coche en girar 90º
#define TIEMPO_PAUSA_TRAS_GIRO 100 //tiempo en ms de pausa tras terminar el giro
#define puertoROS 79      // puerto abierto por el sistema ROS
#define PinServo 44
#define TIEMPO_MARCHA_ATRAS 800
#define INICIOSERVO 90 //Posicion para la que el servo hace que el coche vaya recto.
#define num_velocidades 4


//-->  Variables del Wifi shield  <-- //

char ssid[] = "sammac";      // your network SSID (name) 
char pass[] = "sammac";   // your network password
int keyIndex = 0;                 // your network key Index number (needed only for WEP)
int status = WL_IDLE_STATUS;
WiFiServer server(80);
WiFiServer FeedbackServer( puertoROS );
// IPAddress servidorROS(); // introducir IP separada por comas:  servidorROS(192,168,1,1)
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
boolean ultimoGiro = false; //false=izquierda | true=derecha
boolean avanzado = false; //¿Se ha avanzado tras el ultimo giro al encontrar un obstaculo?
int velocidad[]={180,200,220,255};
int select=0; //Selector de velocidad del array anterior. Valores [0,1,2]
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


  //--> Definicion de los pines de salida del motor <--//

  pinMode(actMot[0], OUTPUT); //Motor derecho
  pinMode(actMot[1], OUTPUT); //Motor derecho
  pinMode(actMot[2], OUTPUT); //Motor izquierdo
  pinMode(actMot[4], OUTPUT); //Motor izquierdo
  pinMode(motD, OUTPUT);
  pinMode(motI, OUTPUT);
  pinMode(LED, OUTPUT);

  //Inicializacion de los motores a 0
  digitalWrite(motD, LOW);
  digitalWrite(motI, LOW);
  digitalWrite(actMot[0],LOW);
  digitalWrite(actMot[1],LOW);
  digitalWrite(actMot[2],LOW);
  digitalWrite(actMot[3],LOW);

  // iniciamos con stop para que no arranque. Seguridad Redundante.
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
    //La primera vez que entra indica que hay un cliente nuevo. 
    if(cliente==true) Serial.println("new client");

    while (client.connected() && Feedback.connected()) {
      // mientras tengamos el cliente conectado.
      // no hemos leido ningun dato


      datos=0;
      //si hay datos disponibles para leer, los leemos y contamos.
      while (client.available()) {
        if(datos==0) client.write('1');
        buffer[datos]=client.read();
        datos++; 

      }
      //en caso de haber leido datos, decodificamos estos
      if(datos >0){
        buffer[datos]='\0';

        comando=(String) buffer; 

        Feedback.write(" Buffer Leido.");
        Feedback.print (buffer);



        // Decodificacion orden recibida

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
        //Vemos si se quiere acelerar o reducir
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

      } // fin de if datos leidos.

      a_funcionar();
      // comprobaciones paa el envio de feedback
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

    } // fin del while cliente conectado
    cliente=false;
    // paramos el coche si perdemos la conexion
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
  // Mientras se encuentre el robot conectado a ROS comprobamos el sensor de proximidad.
  if (on_off){
    //si el coche esta en movimiento, leemos el sensor de proximidad
    sensorValue = analogRead(sensorPin);
    Serial.println(sensorValue);
    // Si supera la distancia minima de seguridad generamos el aviso.
    if (sensorValue > distancia_seguridad ){
 
      obstaculo=true;


      digitalWrite(LED, HIGH);        
      avanzado=false; 
      num_obstaculos++;
      if( num_obstaculos > 2) marcha_atras_=false;
      if( !marcha_atras_) {
        marcha_atras();
        marcha_atras_=true;
        num_obstaculos=0;
      }

      //Si el ultimo giro ha sido a derechas y tenemos otro obstaculo volvemos a girar a derechas
      if (ultimoGiro && !avanzado){

        digitalWrite(actMot[0],LOW);
        digitalWrite(actMot[1],LOW);

        myservo.write(55);
        motDval=velocidad[select];
        motIval=velocidad[select];        
        mover();
        delay(TIEMPO_GIRAR_90);

        //Tras el giro paramos el motor esta que en el siguiente ciclo el sensor vea si tiene algun obstaculo delante

        motDval = 0;
        motIval = 0;          
        mover();
        // myservo.write(90);
        digitalWrite(actMot[0],LOW);
        digitalWrite(actMot[1],HIGH);

        ultimoGiro=true;
        vuelvo_de_giro=true;
        delay(TIEMPO_PAUSA_TRAS_GIRO); //breve pausa para detener el vehiculo tras giro            

      }
      //Si el ultimo giro ha sido a izquierdas y tenemos otro obstaculo volvemos a girar a izqueirdas
      if (!ultimoGiro && !avanzado){

        digitalWrite(actMot[2],LOW);
        digitalWrite(actMot[3],LOW);
        myservo.write(125);
        motDval=velocidad[select];
        motIval=velocidad[select];         
        mover();
        delay(TIEMPO_GIRAR_90);
        //Tras el giro paramos el motor esta que en el siguiente ciclo el sensor vea si tiene algun obstaculo delante             
        motDval = 0;
        motIval = 0;          
        mover();

        digitalWrite(actMot[2],LOW);
        digitalWrite(actMot[3],HIGH);
        ultimoGiro=false;
        vuelvo_de_giro=true;
        delay(TIEMPO_PAUSA_TRAS_GIRO); //breve pausa para detener el vehiculo tras giro
      }
    }
    //Si no hay obstaculo avanzamos en linea recta
    else{
      myservo.write(INICIOSERVO);

      num_obstaculos=0;
      obstaculo=false;
      marcha_atras_=false;
      digitalWrite(LED, LOW);
      motDval=velocidad[select];
      motIval=velocidad[select];
      //  Serial.print("Envio a mover: ");
      //  Serial.println(motDval);
      //  Serial.println(motIval);

      mover();
      avanzado=true;
      if(vuelvo_de_giro){
        ultimoGiro=!ultimoGiro;
        vuelvo_de_giro=false;
      }

    }
  } 
  //Hemos mandado el comando de parada
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
