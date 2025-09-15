

#include <MotorDriver.h>

#include "FirebaseESP8266.h"
#include <ESP8266WiFi.h>
#include <ArduinoJson.h>

// Set these to run example.
#define FIREBASE_HOST "robotreloj1.firebaseio.com"
#define FIREBASE_AUTH "Td6LPbgp9InIMZ0gSio94Q9yKn77z2dFGXdKxqxg"
#define WIFI_SSID "Alfaro B"
#define WIFI_PASSWORD "24512100A"

MotorDriver m;

#define MotorA 4 
#define MotorX 2

float EMA = 0;
float EMX = 0;

 String str = "0.00,0.00";


 

//2. Define FirebaseESP8266 data object for data sending and receiving
FirebaseData firebaseData;


// tiempos 
unsigned long tiempoanterior = 0;
unsigned long tiempocomunicacion = 300;


uint16_t paso = 0;
uint16_t pasoAnterior = 0; // variables que cuentas para verificar si la comunicacion esta bien 

unsigned long tiempoMovimientoanterior =0; //Para evitar grandes movimientos
unsigned long tiempoMovimiento= 150; //siempre debe se ser mayor al tiempo de Comunicacion

     
void loop() {

    unsigned long tiempo = millis();
  if (tiempo - tiempoanterior >= tiempocomunicacion) {
      tiempoanterior = tiempo;

      pasoAnterior = paso;
     
      //6. Try to get int data from Firebase
      //The get function returns bool for the status of operation
      //firebaseData requires for receiving the data
     
      if(Firebase.get(firebaseData,"/Actuadores/string"))
      
      {
        
        
        paso += 1;
        //Success
        //Serial.print("Get int data success, int = ");
        str = String(firebaseData.stringData());
        
        Serial.println(str);

        float Valor =0;         
        int inicioValor =0 ;
        int countValor=0;
        
        for (int i = 0; i < str.length(); i++) {
          
          if (str.substring(i, i+1) == ",") { // si hay una coma   
            
            Valor = str.substring(inicioValor, i).toFloat();     
            
            if (countValor==0){
               EMA = Valor; 
               
            }
            if (countValor==1){
               EMX = Valor; 
               
            }


            inicioValor = i+1;                   
            countValor += 1;


 
          }
          
        }


      
         
      }

      else{
        //Failed?, get the error reason from firebaseData
    
        Serial.print("Error in getString, ");
        Serial.println(firebaseData.errorReason());
       // Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
        ESP.reset();

      }

//// Enviar el status 



      String Enviar = String(EMA) + "," + String(EMX) ;
      if(Firebase.setString(firebaseData, "/Sensores/string" ,Enviar))
  {
    //Success
     Serial.println("Set string data success");
          Serial.print(Enviar);
  
  }
  else{
    //Failed?, get the error reason from firebaseData

    Serial.print("Error in setString, ");
    Serial.println(firebaseData.errorReason());
  }

  }

     if (  paso == pasoAnterior )// verificcar si se dejaron de recibir mensajes          
   {
    
     unsigned long tiempoActual = millis();
     // verificar si se cumplio el tiempo maxdiomo de movimiento 
     if (tiempoActual - tiempoMovimientoanterior > tiempoMovimiento) {
      tiempoMovimientoanterior = tiempoActual;
                        
       EMA=0;
       EMX=0;

         
    }
   }  

     
     
//     Serial.println("EMA");
// mas poder
     
     
      EMA = EMA*255; 
      
       
     


     
//     SeriaX.println("EMX");
      
      
      EMX = EMX*255;

     

     
     MoverMotorX();
     MoverMotorA(); 

}

void MoverMotorX() {

  if (EMX < 0) {
   m.motor(MotorX,BACKWARD,-EMX);
  }

  if (EMX > 0) {
   m.motor(MotorX,FORWARD,EMX);
  
  }
  if (EMX == 0) {
  m.motor(MotorX, RELEASE,EMX);
 
  }
  delay(10);
}


void MoverMotorA() {

  if (EMA < 0) {
   m.motor(MotorA,BACKWARD,-EMA);
  }

  if (EMA > 0) {
   m.motor(MotorA,FORWARD,EMA);
  
  }
  if (EMA == 0) {
  m.motor(MotorA, RELEASE,EMA);
 
  }
    delay(10);
}



void setup() {

  Serial.begin(115200);
 
  // connect to wifi.
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("connecting");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
   Serial.println();
  Serial.print("connected: ");
  Serial.println(WiFi.localIP());
  
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  //. Enable auto reconnect the WiFi when connection lost
  Firebase.reconnectWiFi(true);

}
