#include <AFMotor.h>
#include <ArduinoJson.h>
/////////Pines de los motores
//////....................//////////////
AF_DCMotor MotorA(4);
AF_DCMotor MotorX(2);
AF_DCMotor MotorB(3);
//AF_DCMotor MotorBombaEfluente (1);
//Servo ServoAngulo;

float EMA = 0;
float EMX = 0;

const int PinLimMinX = 30;  // the number of the limit pin
const int PinLimMaxX = 23;  // the number of the limit pin

float LimMinXState = 0;
float LimMaxXState = 0;

float VOL = 0;
float EMB = 0;

float Caudal = 30;  //ml segundo

// tiempos de impresion
unsigned long tiempoflujoanterior = 0;
unsigned long tiempoflujo = 1000;

// tiempos de impresion
unsigned long tiempoanterior2 = 0;
unsigned long tiempoimpresion2 = 200;

// Salida -- Aqui se realiza la configuracion JSon que Rige Todo el robot
const size_t capacity = 1024;
DynamicJsonDocument docSalida(capacity);
const char* json = "{\"Actuadores\":{\"EMA\":[0],\"EMX\":[0],\"EMB\":[0],\"VOL\":[0]}}";


void loop() {



  LeerJson();

  MoverMotorX();
  MoverMotorA();
  MoverMotorB();
  Regar();

  EnviarJson();
}

void LeerJson() {


  DynamicJsonDocument docEntrada(1024);

  if (Serial.available()) {  // se le da priordad a la coneccion por puerto serial

    DeserializationError error = deserializeJson(docEntrada, Serial);
    delay(10);
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.c_str());
      return;
    }
    //(lOS ACTUADORES SE ENVÍAN DE CERO A uno )

    EMX = docEntrada["Actuadores"]["EMX"][0];
    EMX *= 255;  // se ejecuta en 250 pero se guarda y enviade cero a 1

    EMA = docEntrada["Actuadores"]["EMA"][0];  // posicion del servo angulo
    EMA *= 255;                                // se ejecuta en 250 pero se guarda y enviade cero a 1

    EMB = docEntrada["Actuadores"]["EMB"][0];  // posicion del servo angulo
    EMB *= 255;                                // se ejecuta en 250 pero se guarda y enviade cero a 1

    VOL = docEntrada["Actuadores"]["VOL"][0];
  }
}






void EnviarJson() {

  unsigned long tiempo2 = millis();
  if (tiempo2 - tiempoanterior2 >= tiempoimpresion2) {

    tiempoanterior2 = tiempo2;

    docSalida["Actuadores"]["EMA"][0] = EMA / 255;
    docSalida["Actuadores"]["EMX"][0] = EMX / 255;
    docSalida["Actuadores"]["VOL"][0] = VOL;  //MILILITROS


    serializeJson(docSalida, Serial);
    Serial.println();

    //Serial.print(LimMaxXState);
    //Serial.print(LimMinXState);
  }
}


void Regar() {


  unsigned long tiempoactual = millis();
  if (tiempoactual - tiempoflujoanterior >= tiempoflujo) {

    tiempoflujoanterior = tiempoactual;

    if (VOL > 0) {
      // cada segundo reducir el volumen la vcantidad de caudal en relacion con
      // Porque el caudal es en mL/s

      EMB = 70;

      //Curva Calibracion
      VOL -= Caudal;
    }
    if (VOL <= 0) {
      // cada segundo reducir el volumen la vcantidad de caudal en relacion con
      // Porque el caudal es en mL/s

      EMB = 0;
      VOL = 0;
    }
  }
}


void MoverMotorX() {

  LimMinXState = digitalRead(PinLimMinX);
  LimMaxXState = digitalRead(PinLimMaxX);


  if (LimMinXState == LOW) {

    if (EMX < 0) {
      MotorX.setSpeed(-EMX);
      MotorX.run(BACKWARD);
      delay(10);
    }
  } else {
    MotorX.run(RELEASE);
  }
  if (LimMaxXState == HIGH) {

    if (EMX > 0) {
      MotorX.setSpeed(EMX);
      MotorX.run(FORWARD);
      delay(10);
    }
  } else {
    MotorX.run(RELEASE);
  }
  if (EMX == 0) {
    MotorX.run(RELEASE);
  }
}


void MoverMotorA() {

  if (EMA < 0) {
    MotorA.setSpeed(-EMA * 0.7);
    MotorA.run(BACKWARD);
    delay(2);
  }

  if (EMA > 0) {
    MotorA.setSpeed(EMA * 0.7);
    MotorA.run(FORWARD);
    delay(2);
  }
  if (EMA == 0) {
    MotorA.run(RELEASE);
  }
}

void MoverMotorB() {

  if (EMB < 0) {
    MotorB.setSpeed(-EMB);
    MotorB.run(BACKWARD);
    delay(2);
  }

  if (EMB > 0) {
    MotorB.setSpeed(EMB);
    MotorB.run(FORWARD);
    delay(2);
  }
  if (EMB == 0) {
    MotorB.run(RELEASE);
  }
}






void setup() {

  Serial.begin(9600);

  // Crear archivo Json
  deserializeJson(docSalida, json);
  //X
  MotorA.setSpeed(255);
  MotorA.run(RELEASE);

  //Y
  MotorX.setSpeed(255);
  MotorX.run(RELEASE);

  //Bomba
  MotorB.setSpeed(255);
  MotorB.run(RELEASE);


  pinMode(PinLimMinX, INPUT);
  pinMode(PinLimMaxX, INPUT);
}




///////////////////////////////////////
