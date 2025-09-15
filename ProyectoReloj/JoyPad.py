import pygame
import threading
import time

class Joypad:

    def __init__(self):

        #pygame.init()
        pygame.init()
        self.joystick = pygame.joystick.Joystick(0)


        self.nombre = 'No hay ningun control conectado'
        self.axisYizquierdo = 0  ## huskee, da numeros de a -1 a 1
        self.axisYderecho = 0

        self.axisYizquierdo = 0 ## huskee, da numeros de a -1 a 1
        self.axisYderecho = 0
        self.axisXizquierdo = 0  ## huskee, da numeros de a -1 a 1
        self.axisXderecho = 0
        #
        self.hatY = 0
        self.hatX = 0
        #
        self.boton1 = 0
        self.boton2 = 0
        self.boton3 = 0
        self.boton4 = 0
        self.boton5 = 0
        self.boton6 = 0
        self.boton7 = 0

        t1 = threading.Thread(target=self.Get)
        t1.daemon = True
        t1.start()

    def Get(self):

        while (1):
            # #
            for event in pygame.event.get():  # User did something.

                if event.type == pygame.JOYBUTTONDOWN:

                    #print("Joystick button pressed.")
                   pass


                elif event.type == pygame.JOYBUTTONUP:
                   pass# print("Joystick button released.")


            # Get the name from the OS for the controller/joystick.
            self.joystick.init()
            try :
                self.nombre = self.joystick.get_name()
               # print(self.nombre)
                #
                self.axisYizquierdo = self.joystick.get_axis(0)  ## huskee, da numeros de a -1 a 1

                self.axisYderecho = self.joystick.get_axis(2)
                self.axisXizquierdo = self.joystick.get_axis(1)  ## huskee, da numeros de a -1 a 1
                self.axisXderecho = self.joystick.get_axis(3)
                #print("X", self.axisXizquierdo)
                self.hatX = self.joystick.get_hat(0)[0]


                self.hatY = self.joystick.get_hat(0)[1]
                #print("Y", self.hatY)


                self.boton1 =self.joystick.get_button(0)
                self.boton2 = self.joystick.get_button(1)
                self.boton3 = self.joystick.get_button(2)
                self.boton4 = self.joystick.get_button(3)
                self.boton5 = self.joystick.get_button(4)
                self.boton6 = self.joystick.get_button(5)
                self.boton7 = self.joystick.get_button(6)

            except Exception as e :
                print("lellendo los botones ",e)

            time.sleep(0.01)
            #print(self.axisYizquierdo,self.axisYderecho)





