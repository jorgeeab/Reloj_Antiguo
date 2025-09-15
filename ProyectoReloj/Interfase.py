import pandas as pd
import tkinter as tk
from tkinter import *
import PIL.Image, PIL.ImageTk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import matplotlib.animation as animation
from JoyPad import Joypad
#import ProtocoloACKTR
import ProtocoloIrA

import json
import os
import time
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

class InterfaseVisual:
    def __init__(self, window, window_title):

        self.window = window
        self.window.title(window_title)
        self.window.geometry("1100x650")
        self.window ["bg"] = "white"

        self.ok = False


        self.j = Joypad()


        # Create a canvas that can fit the above video source size
        self.canvas = tk.Canvas(window, width=500, height=500)
        self.canvas.place(x=0,y=0)


        # ##Figuras de los gráficos
        # self.fS = Figure(figsize=(10, 4), dpi=60 )
        # self.aS = self.fS.add_subplot(1, 1, 1)
        #
        # self.fA = Figure(figsize=(10, 4), dpi=60)
        # self.aA = self.fA.add_subplot(1, 1, 1)
        #
        # # self.fO = Figure(figsize=(10, 3), dpi= 60)
        # # self.aO = self.fO.add_subplot(1,1,1)
        #
        # self.fR = Figure(figsize=(9, 3), dpi=60)
        # self.aR = self.fR.add_subplot(1, 1, 1)
        #
        # self.canvasS = FigureCanvasTkAgg(self.fS, master=self.window)
        # self.canvasS.get_tk_widget().place(x=500,y=0)
        #
        # self.canvasA = FigureCanvasTkAgg(self.fA, master=self.window)
        # self.canvasA.get_tk_widget().place(x=500,y=240)
        #
        # # self.canvasO = FigureCanvasTkAgg(self.fO, master=self.window)
        # # self.canvasO.get_tk_widget().place(x=0,y=0)
        #
        # self.canvasR = FigureCanvasTkAgg(self.fR, master=self.window)
        # self.canvasR.get_tk_widget().place(x=0,y=470)
        #
        # self.canvasEp = tk.Canvas(master=self.window, width=600, height=1000)
        # self.canvasEp.place(x=1000, y=390)

        self.btn_quit = tk.Button(window, text='Apagar', command=self.Apagar)
        self.btn_quit.place(x=900,y=500)

        self.labelX = tk.Label(text="X=").place(x=550,y=500)
        self.EntradaX = tk.Entry(width=5)
        self.EntradaX.place(x=580,y=500)

        self.labelY = tk.Label(text="Y=").place(x=610,y=500)
        self.EntradaY = tk.Entry(width=5)
        self.EntradaY.place(x=640,y=500)

        self.labelVOL = tk.Label(text="C=").place(x=670,y=500)
        self.EntradaVOL = tk.Entry(width=5)
        self.EntradaVOL.place(x=700,y=500)

        self.EntradaNombreProtocolo = tk.Entry(width=20)
        self.EntradaNombreProtocolo.place(x=550,y=550)

        self.BotonguardarProtocolo = tk.Button(text='Guardar', command=self.BotonGuardarProtocolo).place(x=760,y=550)
        self.Botonaprender = tk.Button(text='Aprender', command=self.BotonAprender).place(x=500,y=600)

        self.Protocolo = ProtocoloIrA.Protocolo('Protocolo-Con')
        self.robot = self.Protocolo.robot

       #  self.ListadeParametros = self.Protocolo.modelo.get_parameter_list()
       # # print("Lista de parametros", self.ListadeParametros)
       #  contadorparametros = 1
       #  for Parametro in self.ListadeParametros:
       #      contadorparametros += 1
       #      self.canvasEp.create_text(200, 10*contadorparametros,text=Parametro)


        # self.AnimacionActuadores = animation.FuncAnimation(self.fA,self.animarActuadores, interval=600)
        # self.AnimacionSensores = animation.FuncAnimation(self.fS, self.animarSensores, interval=600)
        # self.AnimacionRecompensa = animation.FuncAnimation(self.fR, self.animarRecompensa, interval=600)

        #self.AnimacionObjetos = animation.FuncAnimation(self.fO, self.animarObjetos, interval=200)

        self.loadConfig()

        self.delayupdate = 200
        self.update()


        #self.delaycontrol = 400
        #self.Control()

        self.window.mainloop()

    def saveConfig(self):

        self.Configuracion = {'Protocolo': self.Protocolo.NombreProtocolo ,
                               'X': self.EntradaX.get(),'Y': self.EntradaY.get(),'VOL': self.EntradaVOL.get()}

        print(self.Configuracion)

        if not os.path.isdir("Config"):
           os.makedirs("Config")

        with open("Config/configuracion.json", 'w') as f:
            json.dump(self.Configuracion, f)


    def loadConfig (self):

        if  os.path.isdir('Config'):  ## Si aun no hay una carpeta con el nombre de este protocolo entonces se crea una

                with open('Config/configuracion.json', 'r') as f:
                    self.Configuracion = json.load(f)

                #Nombre =self.Configuracion['Protocolo']
                #self.Protocolo.ImportarProtocolo(Nombre)
                self.EntradaNombreProtocolo.delete(0,END)
                self.EntradaNombreProtocolo.insert(0,self.Configuracion['Protocolo'])


                self.EntradaX.delete(0,END)
                self.EntradaX.insert(0,self.Configuracion['X'])

                self.EntradaY.delete(0,END)
                self.EntradaY.insert(0,self.Configuracion['Y'])

                self.EntradaVOL.delete(0,END)
                self.EntradaVOL.insert(0,self.Configuracion['VOL'])


        ##....Guardar la configuracion del protocolo .. se guardan tambien en archivo json

    def Apagar(self):

        self.saveConfig()
        quit()


    def Control(self):


        EMX = 0
        EMY = 0
        EBE = 0
        EMA = 0

        try:

            # self.robot.Renderizar = True
            EMX = round(self.j.axisYizquierdo, 2)
            # print("EMX",EMX)
            EMY = round(self.j.axisXizquierdo, 2)
            EBE = round(self.j.boton3)  ###### Poner control
            EMA = round(self.j.axisYderecho, 2)

            regar = self.j.boton1
            reset = self.j.boton4

            if regar > 0: self.robot.Regar(70.0)

            if reset > 0: self.robot.reset()

            # if self.robot.ModoVirtual:
            accion = np.array([EMX, EMA]).astype(np.float32)

            # print(accion)
            self.robot.step(accion)



        except Exception as e:
            print("Tomando los controles", e)

    def BotonAprender(self):

        self.Protocolo.Aprender(50000,True,2000)

    def BotonGuardarProtocolo(self):

        Nombre = self.EntradaNombreProtocolo.get()
        self.Protocolo.GuardarProtocolo(Nombre)  ## mejor que se pueda guardar el mismo Protocolo ahora con otro nombre


    def BotonNuevoProtocolo(self):
      #  Nombre = self.EntradaNombreProtocolo.get()
        self.Protocolo.NuevoProtocolo(importar_existente=False)

#    def BotonImportarProtocolo(self):

       # Nombre = self.EntradaNombreProtocolo.get()
        #self.Protocolo.NuevoProtocolo(nimportar_existente=True)

    def BotonCambiarComandos(self):

        X=0
        Y=0
        VOL = 0

        try:
            X = self.EntradaX.get()
            Y = self.EntradaY.get()
            VOL = self.EntradaVOL.get()

        except Exception as e:
            print(e)
        self.robot.XRequerido =X
        self.robot.YRequerido =Y
        self.robot.VOLRequerido =VOL

    # def animarPrediccion(self, i):
    #
    #     try:
    #
    #         try:
    #             self.aP.clear()
    #             self.aP.plot(self.Protocolo.dataframePrediccion)
    #             self.aP.set_title('Predicciones')
    #
    #         except:
    #             pass
    #
    #
    #     except:
    #         pass

#    " def animarActuadores(self, i):
#
#         try:
#
#             self.aA.clear()
#             DF = self.robot.env_method("Actuadores")[0]
#
#             for column in DF:
#               self.aA.plot(DF[column], label=column )
#             self.aA.legend()
#             self.aA.set_ylabel('valor')
#             self.aA.set_xlabel('tiempo')
#             self.aA.set_title('Estado Actual de los Atuadores')
#             time.sleep(0.00001)
#         except:
#             pass
# "
    # def animarObjetos(self, i):
    #
    #     try:
    #         self.aO.clear()
    #         DF = self.robot.env_method("Objetos")[0]
    #         for column in DF:
    #           self.aO.plot(DF[column], label=column )
    #         self.aO.legend()
    #
    #     except Exception as e:
    #         pass

    # def animarSensores(self, i):
    #
    #     try:
    #         self.aS.clear()
    #         DF = self.robot.env_method("Sensores")[0]
    #         for column in DF:
    #           self.aS.plot(DF[column], label=column )
    #         self.aS.legend()
    #         self.aS.set_ylabel('valor')
    #         self.aS.set_xlabel('tiempo')
    #         self.aS.set_title('Entradas , Lectura de sensores')
    #         time.sleep(0.00001)
    #     except:
    #         pass
    #
    # def animarRecompensa(self, i):
    #
    #     try:
    #         self.aR.clear()
    #         DF = self.robot.env_method("Recompensa")[0]
    #
    #         for column in DF:
    #           self.aR.plot(DF[column], label=column )
    #         self.aR.legend()
    #
    #         self.aR.set_ylabel('Recompensa')
    #         self.aR.set_xlabel('tiempo')
    #         self.aR.set_title("")
    #         time.sleep(0.00001)
    #
    #     except:
    #         pass

    def update(self):
        # obtener la imagen con la detección
        try:
            time.sleep(0.0001)

            frame = self.robot.Imagen()

            # pasarla al canvas y actualizar intrfase
            self.photo = PIL.ImageTk.PhotoImage(image=PIL.Image.fromarray(frame))
            self.canvas.delete("all")
            self.canvas.create_image(0, 0, image=self.photo, anchor=tk.NW)

            self.window.after(self.delayupdate, self.update)

        except Exception as e:
           print("Updating",e)


def main():
    # Create a window and pass it to the Application object
    InterfaseVisual(tk.Tk(), 'Interfase AC')

main()
