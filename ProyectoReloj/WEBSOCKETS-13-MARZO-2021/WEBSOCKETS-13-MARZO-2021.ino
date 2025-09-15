#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>  
//#include <WiFi.h>
#include <WebSocketsServer.h>


/*1=ON  2=ON  3=OFF 4=OFF 5=OFF 6=OFF 7=OFF 8=OFF (ATmega2560<->ESP8266)
1=OFF 2=OFF 3=ON  4=ON  5=OFF 6=OFF 7=OFF 8=OFF (USB <->ATmega2560)
1=OFF 2=OFF 3=OFF 4=OFF 5=ON  6=ON  7=ON  8=OFF (USB<->ESP8266 (Update firmware or sketch))
1=OFF 2=OFF 3=OFF 4=OFF 5=ON  6=ON  7=OFF 8=OFF (USB<->ESP8266 (communication))// se deve de alternar el 7 A ON o OF para ver los datos del esp x el puerto serial 
1=OFF 2=OFF 3=OFF 4=OFF 5=OFF 6=OFF 7=OFF 8=OFF (All independent)

Table DIP switch Special Solution:
1=ON 2=ON 3=ON 4=ON 5=OFF 6=OFF 7=OFF 8=OFF (USB <-> ATmega2560<-> ESP8266)
USB converter CH340G connect to RX0/TX0 of ATmega2560
ESP8266 connect to RX3/TX3 of ATmega25
*/

// Constants
String ssid = "MySSID";
String password = "MyPassword";
String Datos="";
String inputString="";

String Imprimir = "" ; 

// Globals
WebSocketsServer webSocket = WebSocketsServer(80);
uint8_t NUM = 0;


void setup() {
    //WiFiManager
    //Local intialization. Once its business is done, there is no need to keep it around
    WiFiManager wifiManager;
    //reset saved settings
    //wifiManager.resetSettings();
    
    //set custom ip for portal
    //wifiManager.setAPStaticIPConfig(IPAddress(10,0,1,1), IPAddress(10,0,1,1), IPAddress(255,255,255,0));

    //fetches ssid and pass from eeprom and tries to connect
    //if it does not connect it starts an access point with the specified name
    //here  "AutoConnectAP"
    //and goes into a blocking loop awaiting configuration
    wifiManager.autoConnect("AutoConnectAP");
    //or use this for auto generated name ESP + ChipID
    //wifiManager.autoConnect();
  // Print our IP address
  
  // Start WebSocket server and assign callback
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);

   // Start Serial port
  Serial.begin(115200);
  
  Serial.println("Conectado");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}
 void RecibirDatos()
{
    if (Serial.available()) {       
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
         Datos= inputString;
         inputString ="";
         // enviar por websocket
        webSocket.sendTXT(NUM,Datos);
        //Serial.print(Datos); Se envian pero no se imprimen
        Datos="";
    }
    
 
    }
}



// Called when receiving any WebSocket message
void onWebSocketEvent(uint8_t num,
                      WStype_t type,
                      uint8_t * payload,
                      size_t length) {
             

// primero enviar el string de datos 

  // Figure out the type of WebSocket event
  switch(type) {
 
    // Client has disconnected
    case WStype_DISCONNECTED:
     
      Serial.printf("[%u] Disconnected!\n", num);
      
      break;
 
    // New client has connected
    case WStype_CONNECTED:
     { 
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connection from ", num);
        //Serial.println(ip.toString());
     }
      break;

    //Sillega un mensaje de texto
    case WStype_TEXT:
    {
         // poner podo el payload (List)en un string 
       Imprimir = "" ; 
       //Serial.print(length);
       for (int i = 0; i < length ; i++)
       {
          Imprimir += (char)payload[i];
          Serial.write((char)payload[i]);
         }
      //enviar al arduino

  
    }
         
         break;
 
    // For everything else: do nothing
    case WStype_BIN:
    case WStype_ERROR:
    case WStype_FRAGMENT_TEXT_START:
    case WStype_FRAGMENT_BIN_START:
    case WStype_FRAGMENT:
    case WStype_FRAGMENT_FIN:

   
    default:
      break;
  
}
                      }

 
void loop() {
  // Look for and handle WebSocket data
  
  webSocket.loop();
  //Serial.println("d");
  
  
  RecibirDatos();

}
