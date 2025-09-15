import time
import os
import threading
import serial


import websocket


class Conexion:

    def __init__(self, wifi=False,baudios=115200,puerto=3,online=False,IP = "ws://192.168.0.200"):

        self.TiempoAnterior = 0
        self.wifi=wifi
        self.online=online
        self.textoAenviar = ""
        self.textoAenviarAnterior=""
        self.textoEntrada=""
        self.serial_object = None
        self.Baudios = baudios
        self.Puerto = puerto


        # Wifi
        self.ws = websocket.WebSocket()
        self.IP = IP


    def IniciarConexion (self):


        if self.wifi == False:

            self.serial_object = serial.Serial('COM'+str(self.Puerto), self.Baudios)
            print("conectado", self.serial_object)


        else:

           if self.online :

                TextoInicial= "0.0, 0.0, 0.0"
                self.firebaseApp.put('/Actuadores', "string", str(TextoInicial))
                self.firebaseApp.put('/Sensores', "string", str(TextoInicial))

                result = self.firebaseApp.get("/", "Actuadores")
                print("Consulta", result)
                if result != None:
                    print("Conectado", result)
           else:
               try:
                  print("Conectandose A la ip ", self.IP)
                  self.ws.connect(self.IP)
                  print("Conectado A la ip ",self.IP)
                  self.ws.close()

               except Exception as e:
                   print("Conectando a Websocket ", e)

        if self.wifi == True and self.online == False :
            tw = threading.Thread(target=self.socket_loop)
            # t13.daemon = True
            tw.start()

        else:
            t13 = threading.Thread(target=self.EnviarDatos)
            #t13.daemon = True
            t13.start()

            t1 = threading.Thread(target=self.RecibirDatos)
           # t1.daemon = True
            t1.start()



    def socket_loop (self):

        while (1):
            try :

                #if self.textoAenviar != self.textoAenviarAnterior :
                    self.ws.connect(self.IP)
                    self.ws.send(str(self.textoAenviar))
                    self.textoEntrada = str(self.ws.recv())
                    self.ws.close()
 #                   print(self.textoEntrada)

                    time.sleep(0.2)

                #self.textoAenviarAnterior = self.textoAenviar
            except Exception as e :
                print("Error en el socket loop ",e)

    def EnviarDatos(self):


        while (1):

           try:
                if self.wifi == True: ## conexion por firebase
                  #  print("probando")
                    ## enviar datos
                    self.firebaseApp.put('/Actuadores', "string", str(self.textoAenviar))
                    #print("TEXTO ENVIAR ",self.textoAenviar)

                else:

                    #print("textoEnviar", self.textoAenviar)
                    self.serial_object.write(str(self.textoAenviar).encode())

           except Exception as e :

               print ("Error enviando datos",e)


    def RecibirDatos(self):
        while (1):

            try:

                if self.wifi == False:


                    self.textoEntrada = str(self.serial_object.readline().decode('utf-8'))
                  #  print("textoentrada",self.textoEntrada)
                    # Leer una linea por el puerto serial

                else:


                    # print("textoEnviar", textoAenviar)
                    self.textoEntrada = str(self.firebaseApp.get("/","Sensores/string"))  ## La info entra en forma de Stringh "0.0,0.0"   EMA,EMX



            except Exception as e:

                print("Error Recibiendo datos",e)
               # self.textoEntrada = ""



    # def Comunicarse(self):
    #
    #     while(True):
    #
    #         print("loop")
    #
    #         if self.wifi == True:
    #
    #             ## enviar datos
    #             print("Enviar datos")
    #             self.firebaseApp.put('/Actuadores', "string", str(self.textoAenviar))
    #            ## recibir datos
    #
    #             self.textoEntrada = str(self.firebaseApp.get("/","Sensores/string"))  ## La info entra en forma de Stringh "0.0,0.0"   EMA,EMX
    #         else:
    #             try:
    #
    #                 # print("textoEnviar", textoAenviar)
    #                 self.serial_object.write(str(self.textoAenviar).encode())
    #
    #                 print(str(self.textoAenviar))
    #
    #                 self.textoEntrada = str(
    #                     self.serial_object.readline().decode('utf-8'))  # Leer una linea por el puerto serial
    #                 print(self.textoEntrada)
    #
    #             except Exception as e:
    #
    #                 print(e, " Enviando y recibiendo datos desde el puerto serial ")
    #
    #         ## despues de enviar el texto se borra la info
    #         self.TextoAenviar ="" ##solo se envia una ves
    #
    #         print("Comunicandose")
    #
    #         ## Ahora se recibe el texto y se devuelve a la variable de texto recibido
    #

    def Enviar (self,texto):

        self.textoAenviar=texto

    def Recibir(self):
        return self.textoEntrada

