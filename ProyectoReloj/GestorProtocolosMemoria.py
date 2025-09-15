

#import math

import numpy as np
import pandas as pd
# from stable_baselines.common.policies import MlpPolicy#, MlpLstmPolicy, MlpLnLstmPolicy
# from stable_baselines.common import make_vec_env
from stable_baselines3 import HER,SAC #,PPO2,ACER,SAC

#from stable_baselines3.common.callbacks import StopTrainingOnRewardThreshold
#from stable_baselines3.common.callbacks import CheckpointCallback,EvalCallback,CallbackList
from CallbacksModificado import CheckpointCallback,EvalCallback,CallbackList,evaluate_policy,ProgressBarManager
#from stable_baselines.common.evaluation import evaluate_policy
#from stable_baselines3.common.cmd_util import  DummyVecEnv

import time
import datetime
#from stable_baselines.gail import generate_expert_traj
#from stable_baselines.gail import ExpertDataset

#from stable_baselines3.common.vec_env import VecEnv
#from stable_baselines import results_plotter
#from stable_baselines.results_plotter import load_results, ts2xy
import os
#import matplotlib.pyplot as plt

#from tqdm import tqdm
#import gym

#import websocket

import cv2
#import time
import threading
#import json
import optuna


#from RelojEnv3Fb import RelojSimHierbaEnv
#from RelojEnvSimple import RelojSimHierbaEnv
from RelojEnvMemoria3 import RelojSimHierbaEnv
#from RelojEnvMemoriaVision import RelojSimHierbaEnv
#from RelojEnvMemoriaCompleto import RelojSimHierbaEnv
from JoyPad import Joypad
from Stitch import Stitch


class Protocolo:

    def __init__(self, Nombre='Sin Nombre'):

        self.modelo = None

        self.robot = RelojSimHierbaEnv()

        self.NombreProtocolo = Nombre


        self.CarpetaProtocolo = str.format('AnalizarProtocolos/{}', self.NombreProtocolo)
        print("Carpeta protocolo", self.CarpetaProtocolo)

        self.NuevoProtocolo(True)

        self.Ensayo = "Ensayos/EnsayoPrueba/EnsayoPrueba.xlsx"
        self.NombreEnsayo = "EnsayoPrueba"
       #self.ImportarEnsayo(NombreEnsayo=self.NombreEnsayo)

        self.ControlManual = True
        self.j = Joypad()
        print("joypad iniciado")
        ## establecer las condiciones iniciales del robot
        #
        self.GenerandoMapa = False
        self.robot.reset()
        print("ROBOT Iniciado")
        t2 = threading.Thread(target=self.InterfaseLoop)
        t2.daemon = False
        t2.start()

        #generar mapa continuamente
        #
        # t2 = threading.Thread(target=self.GenerarMapa)
        # t2.daemon = False
        # t2.start()
        #


    def GenerarMapa(self):

        #  imagen = cv2.imread("ESPACIO2.jpg", cv2.IMREAD_COLOR)
        # print("threat")
        # frame = self.robot.Imagen()

        # try:
        while(1):

            if self.GenerandoMapa:

                SeccionesCompletas=False



                if SeccionesCompletas == False:



                    for punto in self.robot.PuntosMapa():
                        CarpetaSeccion = "Ensayos/{}/Mapa/Secciones/{},{}".format(self.NombreEnsayo, punto[0], punto[1])
                        if os.path.isdir(CarpetaSeccion)== False:
                                os.makedirs(CarpetaSeccion)

                    #Verificar los puntos que han sido tomados
                    margen = self.robot.MargenMapa * self.robot.PixelesPorCentimetro
                    anchocamara = self.robot.Mappeador.AnchoFoto
                    altocamara = self.robot.Mappeador.AltoFoto
                    MargenMapa = self.robot.MargenMapa
                    Anchomapa = MargenMapa*2+self.robot.MaxX*2
                    MapaCompleto = np.zeros( (int(Anchomapa * self.robot.PixelesPorCentimetro), int(Anchomapa * self.robot.PixelesPorCentimetro), 3), np.uint8) * 1
                    ImagenNegra= np.zeros((altocamara,anchocamara , 3 ),  np.uint8) * 1

                    FranjaHorizontal= np.zeros(( altocamara,int(Anchomapa * self.robot.PixelesPorCentimetro), 3 ),  np.uint8) * 1
                    FranjaUnida = ImagenNegra
                    DistanciaXPuntos = self.robot.Mappeador.DistanciaXPuntos
                    DistanciaYPuntos = self.robot.Mappeador.DistanciaYPuntos
                    PosXFranjaUnida = 0

                    FranjasHorizontales =[]

                    unir = False  # cuando es positivo se usa el algoritmo de stitch

                    Puntos = self.robot.PuntosMapa()

                    for y in range(DistanciaYPuntos, int(Anchomapa), DistanciaYPuntos):  # (cm)

                        FranjaHorizontal = np.zeros(
                            (altocamara, int(Anchomapa * self.robot.PixelesPorCentimetro), 3), np.uint8) * 1
                        print("Franja horizontal shape",FranjaHorizontal.shape)
                        for x in range(DistanciaXPuntos, int(Anchomapa), DistanciaXPuntos):  # (cm)

                            CarpetaSeccion = "Ensayos/{}/Mapa/Secciones/{},{}".format(self.NombreEnsayo, x, y)
                            if os.path.isdir(CarpetaSeccion):
                                posx = x * self.robot.PixelesPorCentimetro
                                posy = y * self.robot.PixelesPorCentimetro

                                if len (os.listdir(CarpetaSeccion)) == 0 : # si la carpeta  está  vacia

#                                   FranjaHorizontal [  : ,int(margen+posx - anchocamara/2 ): int(margen+posx + anchocamara/2)  ] = ImagenNegra

                                   unir = False

                                else: # si hay imagenes
                                    #seleccionar la ulima imagen


#                                   #C = sorted(filter(os.path.isfile, os.listdir(CarpetaSeccion)),key=os.path.getmtime)
                                    ListaOrdenada=os.listdir(CarpetaSeccion)


                                    ArchivoImagen = CarpetaSeccion + "/"+str(ListaOrdenada[len(ListaOrdenada)-1])
                                    print("ArchivoImagen",ArchivoImagen)
                                    Imagen = cv2.imread(ArchivoImagen)

                                    if unir :
                                        try:
                                            FranjaUnida= Stitch().UnirImagenes(FranjaUnida,Imagen)

                                            ## si hay un siguiente  punto
                                            if x+DistanciaXPuntos < Anchomapa:
                                                CarpetaSeccionSeccionSiguiente = "Ensayos/{}/Mapa/Secciones/{},{}".format(self.NombreEnsayo, x+DistanciaXPuntos ,
                                                                                                           y)

                                                if len(os.listdir(CarpetaSeccionSeccionSiguiente)) == 0:  # si la carpeta  está  vacia

                                                    FranjaHorizontal[:, int(margen + PosXFranjaUnida - anchocamara / 2): int(
                                                                margen + PosXFranjaUnida + anchocamara / 2)] = FranjaUnida

                                                else:

                                                    FranjaHorizontal[:,int(margen + PosXFranjaUnida - anchocamara / 2): int(
                                                        margen + PosXFranjaUnida + anchocamara / 2)] = FranjaUnida
                                                    #FranjasHorizontales.append(FranjaHorizontal)
                                        except Exception as e :
                                            print ("Uniendo imagen",e)


                                    else:#if unir = false

                                        FranjaUnida = Imagen

                                        PosXFranjaUnida = posx

                                        FranjaHorizontal[:, int(margen + PosXFranjaUnida - anchocamara / 2): int(margen + PosXFranjaUnida + anchocamara / 2)] = FranjaUnida
                                        FranjasHorizontales.append(FranjaHorizontal)


                                    unir= True
                            ## hacer una imagen negra igual al mapa commpleto
                            #FondoNegro = np.zeros( (int(Anchomapa * self.robot.PixelesPorCentimetro), int(Anchomapa * self.robot.PixelesPorCentimetro), 3), np.uint8) * 1
                            ## pegar la frqanja en el centro del fondoNegro

                            #Girar La franja y hacerla vertical

                            # scale = 1
                            # hroi, wroi = FranjaHorizontal.shape[:2]
                            #
                            # center = (wroi / 2, hroi / 2)
                            #
                            # angulo = -90
                            #
                            # # Rotar la imagen
                            # M = cv2.getRotationMatrix2D(center, angulo, scale)
                            # FranjaVertical = cv2.warpAffine(FranjaHorizontal, M, (wroi, hroi))

                            #Girar l franja y hacerla vertical

                            FranjaVertical = cv2.rotate(FranjaHorizontal, cv2.ROTATE_90_COUNTERCLOCKWISE)
                            cv2.imshow("FranjaVertical", FranjaVertical )
                            cv2.waitKey(1)

                            MapaCompleto[:, int(margen + y - altocamara / 2): int(margen +y + altocamara / 2)] = FranjaVertical
                            cv2.imshow("Mapa",MapaCompleto)
                            cv2.waitKey(1)






    def InterfaseLoop(self):

        #  imagen = cv2.imread("ESPACIO2.jpg", cv2.IMREAD_COLOR)
        # print("threat")
        # frame = self.robot.Imagen()
        #
        # try:
            while (1):
                # try:

                rojo = (0, 0, 255)
                verde = (0, 255, 0)
                azul = (255, 0, 0)
                blanco = (255, 255, 255)
                gris = (122, 122, 122)
                negro = (0, 0, 0)
                amarillo = (0, 255, 255)
                Letra = cv2.FONT_HERSHEY_DUPLEX



                #Imagen de la camara
                AnchoImagenCamara=350
                AltoImagenCamara=350

                ImagenCamara = np.zeros(( AltoImagenCamara, AnchoImagenCamara, 3 ),  np.uint8)# crear el rectangulo en el que va la camara
                                         #alto,ancho
                Camara = self.robot.Imagen()
                AltoCamara, AnchoCamara = Camara.shape[0:2]
                ##suponiendo que la camara es mas Ancha que alta
                ancho=AnchoImagenCamara ## ancho fijo
                alto=int(ancho*AltoCamara/AnchoCamara)
                Camara= cv2.resize(Camara, (ancho, alto))
                ImagenCamara[:alto,:ancho] = Camara
                                                      # ancho,alto
                ImagenCamaraVirtual = np.zeros((AltoImagenCamara, AnchoImagenCamara, 3),
                                        np.uint8)  # crear el rectangulo en el que va la camara

                CamaraVirtual = self.robot.simulacioncamara
                AltoCamaraVirtual, AnchoCamaraVirtual = CamaraVirtual.shape[0:2]
                ##suponiendo que la camara es mas Ancha que alta
                ancho = AnchoImagenCamara  ## ancho fijo
                alto = int(ancho * AltoCamaraVirtual / AnchoCamaraVirtual)
                CamaraVirtual = cv2.resize(CamaraVirtual, (ancho, alto))
                ImagenCamaraVirtual[:alto, :ancho] = CamaraVirtual


                ImagenMapaVirtual = self.robot.ImagenMapaVirtual()
                ImagenMapaVirtual= cv2.resize(ImagenMapaVirtual, (1000, 1000))

                ImagenMapa = self.robot.ImagenMapa()
                ImagenMapa = cv2.resize(ImagenMapa, (350, 350))


                ImagenCamaraVirtal = cv2.resize(ImagenMapa, (350, 350))



                ImagenMapaLimitado = self.robot.ImagenMapaLimitado()
                #print(ImagenMapaLimitado)
                ImagenMapaLimitado = cv2.resize(ImagenMapaLimitado, (350, 350))

                ## Sobreponer mapa real en virtual quitando el blanco


                ## Extraer Las Dimensiones Reales del mapa
                MaxX = self.robot.MaxX
                MinX = self.robot.MinX

                #(cm)
                margen = self.robot.MargenMapa
                anchomapa = MaxX*2 + 2* margen
                altomapa = MaxX*2  + 2* margen


                #(pixeles) los de la imagen virtual
                anchoimagen=1000
                altoimagen=1000

                Escala = anchoimagen/anchomapa


                ## Curear cuadro del mapa en la frame
                wPerimetro = int(MaxX*2 * Escala)
                hPerimetro = int(MaxX*2 * Escala)

                px = int(margen*Escala)
                py = int(margen*Escala)

                # print("Version",cv2.version.opencv_version)

                cv2.rectangle(ImagenMapaVirtual, pt1=(px, py), pt2=((px + wPerimetro), (py + hPerimetro)),
                              color=blanco, thickness=1, lineType=cv2.LINE_4)
                #
                Plantas = self.robot.plantas()
                # Hierbas = self.robot.hierbas()

                Posicion = self.robot.PosicionReal()  # (A,X,Y)

                Angulo,Radio,PosXDetectada, PosYDetectada, PosZDetectada = self.robot.PosicionDetectada()  # (X,Y,Z) Posicion detectada en forma de cuadricula

                # PuntosMapeo = self.robot.PuntosMapa()

                PorcentajeMapa, PorcentajePlantas, PorcentajeHierbas = self.robot.Porcentajes()

                Error = self.robot.error()

                # area de influencia

                Xminimo, Xmaximo, Yminimo, Ymaximo = self.robot.area()  # ( AnguloMinimo,  AnguloMaximo, Xminimo, Xmaximo )

                PuntosMapeo = self.robot.PuntosMapa()  ## Una lista con los puntos de mapeo en cm

                PMx, PMy, PMd = self.robot.PuntoCercano()  # (PMx,PMy,PMd)

                figuras = self.robot.figuras()  # ((Fx1,Fy1,Fa1),(Fx2,Fy2,Fa2))
                # print("figuras",figuras)

                F1x, F1y, F1a, F1d = figuras[0][0], figuras[0][1], figuras[0][2], figuras[0][3]
                F2x, F2y, F2a, F2d = figuras[1][0], figuras[1][1], figuras[1][2], figuras[1][3]

                EMA, EMX, EMY, EMAD, EMXD, EMYD = self.robot.actuadores()

                VelA, VelX, VelZ = self.robot.velocidades()

                # Objetivos = self.robot.Objetivos()  # (A,X,Y)

                Recompensa = self.robot.Recompensa()  # float Recompensa
                #  print("Recompensa",Recompensa)

                Centromapa = (int((MaxX+margen)*Escala), int((MaxX+margen)*Escala))

                ## primer elipse para marcar limite superior
                radius = int(MaxX * Escala)  # cm * Aumento

                # print("radio",radius)

                axes = (radius, radius)
                angle = 0
                startAngle = 0
                endAngle = 360
                # When thickness == -1 -> Fill shape
                thickness = 1

                cv2.ellipse(img=ImagenMapaVirtual, startAngle=startAngle, endAngle=endAngle, color=verde, thickness=thickness,
                            angle=angle, axes=axes, center=Centromapa)

                ## marca limite Inferoior
                radius = int(MinX * Escala)
                axes = (radius, radius)
                cv2.ellipse(img=ImagenMapaVirtual, startAngle=startAngle, endAngle=endAngle, color=verde, thickness=thickness,
                            angle=angle,
                            axes=axes, center=Centromapa)

                #  puntos

                # puntpo posicion
                radius = int(Posicion[1] * Escala)
                if radius > 0:
                    axes = (radius, radius)
                else:
                    axes = (0, 0)
                startAngle = 180 + int(Posicion[0])
                endAngle = startAngle + 10
                thickness = 3

                cv2.ellipse(img=ImagenMapaVirtual, startAngle=startAngle, endAngle=endAngle, color=blanco, thickness=thickness,
                            angle=angle,
                            axes=axes, center=Centromapa)

                # punto Plantas
                for planta in Plantas:

                    PuntoP = (px +int( planta[0] * Escala),
                              py + int( planta[1] * Escala))
                    # print("F1a",F1a)
                    cv2.circle(ImagenMapaVirtual, PuntoP, 4, verde,1, )

                #
                # # punto Hierbas
                # for hierba in Hierbas:
                #
                #
                #     ## DIbujar mapa de la hubicacion detectada
                #
                #     PuntoH = (int(px + hierba[0]  * PixelesPorCentimetro * escala), int(py + hierba[1] * PixelesPorCentimetro * escala))
                #     # print("F1a",F1a)
                #     cv2.circle(frame, PuntoH,5, rojo, 2, )
                #
                # ## Dibujar un Cuadro para los limites marcados

                pxLimite = px + int(Xminimo * Escala)
                pyLimite = py + int(Yminimo * Escala)
                ## Curear cuadro del mapa en la frame
                wPerimetroLimite = int((Xmaximo - Xminimo) * Escala)
                hPerimetroLimite = int((Ymaximo - Yminimo) * Escala)

                # print("Version",cv2.version.opencv_version)

                cv2.rectangle(ImagenMapaVirtual, pt1=(pxLimite, pyLimite),
                              pt2=((pxLimite + wPerimetroLimite), (pyLimite + hPerimetroLimite)),
                              color=blanco, thickness=1, lineType=cv2.LINE_4)

                ## Dibujar los puntos de Mapeo
                for punto in PuntosMapeo:

                    xpunto = px + int(punto[0] * Escala)
                    ypunto = py + int(punto[1] * Escala)

                    valor = punto[2]
                    if valor == 0:
                        cv2.circle(ImagenMapaVirtual, (xpunto, ypunto),2, azul, 1, )
                    else:
                        cv2.circle(ImagenMapaVirtual, (xpunto, ypunto), 2, blanco,1, )

                ##Dibujar el punto de la posicion

                pxd = px + int(PosXDetectada* Escala )
                pyd = py + int(PosYDetectada* Escala )

                cv2.circle(ImagenMapaVirtual, (pxd,pyd ), 3, rojo, 2, )

                ## Cortar el mapa

                ImagenMapaCortado =  np.zeros(( 100, 100, 3 ),  np.uint8) * 1
                try:
                    ImagenMapaCortado =ImagenMapaVirtual[ pyd-50 : pyd+50, pxd-50: pxd+50 ]

                except:pass
                ImagenMapaAumentado = cv2.resize(ImagenMapaCortado, (350, 350))


                # ## Ahora Resaltar el punto mas Cercano
                # xpunto = px + int(PMx * PixelesPorCentimetro * escala)
                # ypunto = py + int(PMy * PixelesPorCentimetro * escala)
                # cv2.circle(frame, (xpunto, ypunto), 3, blanco, 2, )
                #

                # Informacion para imprimir en la pantalla

                ImagenDatos = np.zeros(( 700, 75, 3 ),  np.uint8) * 1

                info = [
                    # ("A", Posicion[0]),
                    # ("X", Posicion[1]),
                    # ("Y", Posicion[2]),

                    ("Angulo", Angulo),
                    ("Radio", Radio),
                    ("XD", PosXDetectada),
                    ("YD", PosYDetectada),
                    ("ZD", PosZDetectada),

                    ("PM", PorcentajeMapa),
                    ("DP", PMd),

                    ("PP", PorcentajePlantas),
                    ("F1d", F1d),

                    ("PP", PorcentajeHierbas),
                    ("F1d", F2d),

                    ("Error", Error),

                    ("Xmin", Xminimo),
                    ("Xmax", Xmaximo),
                    ("Ymin", Yminimo),
                    ("Ymax", Ymaximo),

                    ("R:", Recompensa),

                    ("EMA:", EMA),
                    ("EMAD:", EMAD),
                    ("EMX:", EMX),
                    ("EMXD:", EMXD),
                    ("EMY:", EMY),
                    ("EMYD:", EMYD),

                    ("VelA:", VelA),
                    ("VelX:", VelX),
                    ("VelZ:", VelZ)

                ]
                # loop over the info tuples and draw them on our frame
                for (i, (k, v)) in enumerate(info):
                    text = "{}: {}".format(k, round(v, 6))
                    cv2.putText(ImagenDatos, text, (10, ((i * 20) + 20)),
                                Letra, 0.3, blanco, 1)

                # # CUADRO DE OBSERVACION
                #      ##Rectangulo del Perimetro
                #      xPerimetro = 0
                #      yPerimetro = 0
                #
                #      wPerimetro = int(width*0.3)
                #     # print("WPerimetro" , wPerimetro )
                #      hPerimetro = int(height*0.3)
                #  #    print("HPerimetro", hPerimetro)
                #
                #     # print("Version",cv2.version.opencv_version)
                #
                #      cv2.rectangle(frame,pt1=(xPerimetro,yPerimetro),pt2=((xPerimetro + wPerimetro), (yPerimetro + hPerimetro)),color=rojo,thickness=1,lineType=cv2.LINE_4)
                #
                #      PuntoFigura1 = int(F1x*0.3), int(F1y*0.3)
                #      # print("F1a",F1a)
                #      cv2.circle(frame, (PuntoFigura1), int(math.sqrt( F1a/(width*height)*(hPerimetro*wPerimetro))), verde, 2, )
                #      # print("FIG1 x= {}, y= {} ".format(PuntoFigura1[0],PuntoFigura1[1]))
                #
                #      PuntoFigura2 = int(F2x*0.3), int(F2y*0.3)
                #      cv2.circle(frame, (PuntoFigura2), int(math.sqrt( F2a/(width*height)*(hPerimetro*wPerimetro))), rojo, 2, )
                #
                #      # PuntoMapa = int(pmx * AumentoImagenCamara), int(pmy * AumentoImagenCamara)
                #      # cv2.circle(frame, (PuntoMapa), int(math.sqrt(F2a)), rojo, 2, )

                #  frame = cv2.resize(frame, None, fx=0.8, fy=0.8, interpolation=cv2.INTER_CUBIC)

                # cv2.imshow("ImMapal", ImagenMapaLimitado)
                # print("immapaLim.type",ImagenMapaLimitado.shape)
                # cv2.imshow("ImMapa", ImagenMapa)
                # print("immapa.type", ImagenMapa.shape)

                ImagenMapaVirtual = cv2.resize(ImagenMapaVirtual, (350, 350))

                #Pantalla Dividida En cuatro Partes
                pantallaIzquierda = cv2.vconcat([ImagenMapaVirtual,ImagenCamara])

                #pantallaDerecha = cv2.vconcat([ImagenMapaLimitado,ImagenMapa])
                pantallaDerecha = cv2.vconcat([ImagenMapaAumentado,ImagenCamaraVirtual])

                Pantalla = cv2.hconcat([ImagenDatos,pantallaIzquierda,pantallaDerecha])



                cv2.imshow("Ensayo: {}, Protocolo: {}".format(self.NombreEnsayo,self.NombreProtocolo), Pantalla)

                #cv2.imshow("Mapa_{}".format(self.NombreProtocolo), mapa)

                self.Control()

                cv2.waitKey(1)
        #
        # except Exception as e :
        #     print("InterfaseLoop",e)

    def Control(self):

        EMX = 0
        EMY = 0
        EBE = 0
        EMA = 0

        if (self.ControlManual):

            try:

                # self.robot.Renderizar = True
                EMX = round(self.j.axisYizquierdo, 2)
                # print("EMX",EMX)
                EMY = round(self.j.axisXizquierdo, 2)
                EBE = round(self.j.boton3)  ###### Poner control
                EMA = round(self.j.axisYderecho, 2)

                regar = self.j.boton1
                reset = self.j.boton4

                if regar > 0: self.robot.Regar(200.0)

                elif reset > 0: self.robot.reset()


                # if self.robot.ModoVirtual:
                else:
                    accion = np.array([EMX, EMA]).astype(np.float32)

                    #print(accion)
                    self.robot.step(accion)



            except Exception as e:
                print("Tomando los controles", e)



    def objective(self, trial):

        self.ControlManual = False

        mejor_recompensa = -1000
        try:



            #  HER SAC
            lr = trial.suggest_loguniform("lr", 1e-5, 1e-2)
            gamma = trial.suggest_loguniform("gamma", 0.93, 0.97)

            batch_size = trial.suggest_int("batchsize",10, 1024)
            #batch_size = trial.suggest_categorical("batch_size", [32 , 64 , 256, 512, 1024])

            #buffer_size = trial.suggest_int("buffer_size", 1e2, 1e4)
            buffer_size = int(1e6)

             # Int parameter
            num_layers = trial.suggest_int('num_layers', 1, 4)


            layers = []
            for layer in range(num_layers):
              n_units = int(trial.suggest_loguniform('n_units_L{}'.format(layer), 4, 1024))
              layers.append(n_units)

            Nombremodelo = "LR{}-G{}-Ly{}-Bat{}-Buf{}".format(round(lr, 5), round(gamma, 5), layers,batch_size,buffer_size)

            self.CarpetaProtocolo =str.format('AnalizarProtocolos/{}/ModelosAnalizados/{}', self.NombreProtocolo,Nombremodelo)


            print(self.CarpetaProtocolo)

            self.NuevoProtocolo(lr = lr, gamma = gamma, batch_size = batch_size,layers=layers, buffer_size = buffer_size)

            mejormodelo=self.modelo



            for step in range(4):

                self.modelo = mejormodelo
                self.Aprender(1e4, evaluar=False)

                # Report intermediate objective value.

                recompensas_evaluaciones = []

                for i in range (2):

                    mean_reward, std_reward = evaluate_policy(self.modelo, self.robot, n_eval_episodes=6)
                    recompensas_evaluaciones.append(mean_reward)

                prom_recompensas = np.mean(recompensas_evaluaciones)
                desviacion_recompensas = np.std(recompensas_evaluaciones)

                intermediate_value = prom_recompensas - desviacion_recompensas

                if step < 1 :
                  print("STEP",step)
                  mejor_recompensa = intermediate_value
                else:
                  if intermediate_value > mejor_recompensa  :
                    mejor_recompensa = intermediate_value
                    mejormodelo=self.modelo

                print("promedio_recompensas",prom_recompensas)
                print("desviacion_recompensas", desviacion_recompensas)

                print("Recompensa estandar:  ", intermediate_value)
                print("mejor_recompensa",mejor_recompensa)


                trial.report(intermediate_value, step)



                # Handle pruning based on the intermediate value.
                if trial.should_prune():
                    raise optuna.TrialPruned()


            self.GuardarProtocolo(mejormodelo)

        except Exception as e:
            print("Entrenando", e)

        return mejor_recompensa

    def Entrenar (self,Pasos=4,len_Pasos=1e4,guardar=False):

      mejormodelo=self.modelo

      for step in range(Pasos):

        self.modelo = mejormodelo
        self.Aprender(int(len_Pasos), evaluar=False)

        # Report intermediate objective value.

        recompensas_evaluaciones = []

        for i in range (2):

            mean_reward, std_reward = evaluate_policy(self.modelo, self.robot, n_eval_episodes=6)
            recompensas_evaluaciones.append(mean_reward)

        prom_recompensas = np.mean(recompensas_evaluaciones)
        desviacion_recompensas = np.std(recompensas_evaluaciones)

        intermediate_value = prom_recompensas - desviacion_recompensas

        if step < 1 :
          print("STEP",step)
          mejor_recompensa = intermediate_value
        else:
          if intermediate_value > mejor_recompensa  :
            mejor_recompensa = intermediate_value
            mejormodelo=self.modelo
            if guardar:
              self.GuardarProtocolo(mejormodelo)

        print("promedio_recompensas",prom_recompensas)
        print("desviacion_recompensas", desviacion_recompensas)

        print("Recompensa estandar:  ", intermediate_value)
        print("mejor_recompensa",mejor_recompensa)




    def Aprender(self,Pasos,evaluar,FrecuenciaEvaluacion=int(1e4)):

        self.robot.ObjetivosRandom(True)
        self.robot.PlantasRandom(True)
        self.robot.HierbasRandom(True)
        self.robot.evaluando(False)

        with ProgressBarManager(
                int(Pasos)) as plot_callback:  # this the garanties that the tqdm progress bar closes correctly

            if evaluar:
                # checkpoint_callback = CheckpointCallback(save_freq=10000, save_path=self.CarpetaProtocolo)

                eval_env = self.robot
                #callback_on_best = StopTrainingOnRewardThreshold(reward_threshold=-50, verbose=1)
                eval_callback = EvalCallback(eval_env,
                                             best_model_save_path='{}/mejor_modelo'.format(self.CarpetaProtocolo),
                                             log_path='{}/resultados'.format(self.CarpetaProtocolo), eval_freq=FrecuenciaEvaluacion,n_eval_episodes=3)

                # callback = CallbackList([eval_callback,checkpoint_callback,plot_callback])
                self.modelo.learn(total_timesteps=int(Pasos), callback=[eval_callback, plot_callback])  #

            else:
                self.modelo.learn(total_timesteps=int(Pasos), callback=[plot_callback])  #

        #self.GuardarProtocolo()


    def Evaluar(self):

        try:

            mean_reward, std_rewardevaluate_policy = evaluate_policy(self.modelo,self.robot,20,)
            print("Resultado de la Evaluacion ")
            print("Mean reward: ", mean_reward)
            print("desviacion: ",std_rewardevaluate_policy)

        except Exception as e:
            print("Evaluando ", e)



    def IrA (self,X,Y, intentos =10 ,pasosLimite=5000):

        self.robot.ObjetivosRandom(False)
        self.robot.PlantasRandom(False)
        self.robot.Evaluar=False
      #  self.robot.HierbasRandom(False)

        self.robot.CambiarRequerimientos(X,Y)

        episode_rewards, episode_lengths = [], []
        for i in range(intentos):

           # done =self.robot._get_done()
           # print("rew",self.robot._get_reward())
            listo = False
            ## primero revisar


            if self.robot._get_reward() > -0.1:
             listo = True


            episode_length = 0
            episode_reward = 0.0
            obs=self.robot._get_obs()

            while not listo:

                action, state = self.modelo.predict(obs)

               #print("ep len",episode_length)
                obs, reward, done, _info = self.robot.step(action)
                episode_reward += reward
                episode_length += 1
                if episode_length > pasosLimite or  self.robot._get_reward() > -0.1 :
                    listo = True
            episode_rewards.append(episode_reward)
            episode_lengths.append(episode_length)

            print("mean recompensa", np.mean(episode_rewards))
            print("std", np.std(episode_rewards))


    def Mappear(self, XMinima ,XMaxima,YMinima ,YMaxima):

        self.robot.ObjetivosRandom(False)
        self.robot.PlantasRandom(False)
        self.robot.HierbasRandom(False)

        Xcentro = int(((XMaxima - XMinima) / 2) + XMinima)
        Ycentro = int(((YMaxima - YMinima) / 2) + YMinima)

        self.robot.LimitarMapa(XMinima,XMaxima,YMinima,YMaxima)

        self.IrA(Xcentro, Ycentro, intentos=2, pasosLimite=10000)

        PuntosMapeo = self.robot.PuntosMapa()
        for punto in PuntosMapeo:

            print("llendo a puno {} {}".format(punto[0],punto[1]))
            self.IrA(X=int(punto[0]),Y=int(punto[1]),intentos=2,pasosLimite=10000)
            #self.robot.AgregarAMapa(guardar=False,carpeta=carpeta_pieza_mapa,returnImagen=True)
            ## si es el Primer punto

            if punto == PuntosMapeo[0]:

                carpeta_pieza_mapa = "Ensayos/{}/Mapa/Secciones/{},{}".format(self.NombreEnsayo, Xcentro, Ycentro)
                archivo_pieza_mapa = ""
                if not os.path.isdir(carpeta_pieza_mapa):
                    os.mkdir(carpeta_pieza_mapa)
                if len(os.listdir(
                        carpeta_pieza_mapa)) > 0:  ## se carga la ultima imagen de la lista por que se pónen en orden
                    archivo_pieza_mapa = "{}/{}.png".format(carpeta_pieza_mapa, os.listdir(carpeta_pieza_mapa)[-1])
                else:
                    archivo_pieza_mapa = "{}/{}.png".format(carpeta_pieza_mapa, "_")

                #self.robot.agregandoimagenes(True)
                self.robot.NuevaPieza(XMinima, XMaxima, YMinima, YMaxima, archivo_pieza_mapa)

            else:

                self.robot.AgregarAPieza()  ## agrega la imagen que esta vendo el robot volteada la pieza que se esta creando actualmente

            ## si es el ultimo punto
            if punto == PuntosMapeo[len(PuntosMapeo)-1]:
                self.robot.AgregarAPieza(UltimaImagen=True)  ## agrega la imagen que esta vendo el robot volteada la pieza que se esta creando actualmente

        #cv2.imwrite(img=ImagenFoto, filename=carpeta_seccion_mapa)
        # TiempoActual = datetime.datetime.now().date()
        #
        # print("Tiempo Actual", str(TiempoActual))
        #

        # carpeta_mapa = "Ensayos/{}/Mapa/MapaCompleto {}.png".format(self.NombreEnsayo, str(TiempoActual))
        # self.robot.GuardarMapa(Carpeta=carpeta_pieza_mapa)



    def Optimizar(self, n_modelos):

        self.study = optuna.create_study(pruner=optuna.pruners.MedianPruner(),
                                         direction="maximize",
                                         storage='sqlite:///{}_BD.db'.format(self.CarpetaProtocolo),
                                         study_name=self.NombreProtocolo, load_if_exists=True)
        self.study.optimize(self.objective, n_trials=n_modelos)

        print("Number of finished trials: {}".format(len(self.study.trials)))

        print("Best trial:")
        trial = self.study.best_trial

        print("  Value: {}".format(trial.value))

        print("  Params: ")
        for key, value in trial.params.items():
            print("    {}: {}".format(key, value))

        optuna.visualization.plot_optimization_history(self.study)

        optuna.visualization.plot_optimization_history(self.study)

        optuna.visualization.plot_contour(self.study, params=['n_estimators', 'max_depth'])

    def VisualizarAnalisis(self):


        study = optuna.create_study(pruner=optuna.pruners.MedianPruner(),
                                    direction="maximize",
                                    storage='sqlite:///{}_BD.db'.format(self.CarpetaProtocolo),
                                    study_name=self.NombreProtocolo, load_if_exists=True)

        optuna.visualization.plot_optimization_history(study)

        optuna.visualization.plot_optimization_history(study)

#        optuna.visualization.plot_contour(study, params=['n_estimators', 'max_depth'])

    def GuardarProtocolo(self,Modelo):  # esta funcion guarda este ptotocolo en una carpeta con su nombre

        if not os.path.isdir(self.CarpetaProtocolo):
            os.makedirs(self.CarpetaProtocolo)

        ## Guardar el modelo
        Modelo.save("{}/{}.zip".format(self.CarpetaProtocolo,self.NombreProtocolo))

        print("Modelo Guardado..............................")

        ##LR0.00952-G0.93478-Ly[256, 512, 512, 512]-Bat256-Buf6658

    def NuevoProtocolo(self,importar_existente=True ,lr=0.00004, gamma =0.936, batch_size = 802,layers=[8], buffer_size = 2900):


        if importar_existente:
          print("revisando por modelos preexistentes")
          if os.path.isfile("{}/mejor_modelo/best_model.zip".format(self.CarpetaProtocolo)):

              self.modelo = HER.load("{}/mejor_modelo/best_model.zip".format(self.CarpetaProtocolo), self.robot)

              print("un modelo preexistente se importo")

          else:

              print("nose encontro modelo preexistente, creando un nuevo protocolo")

              max_episode_length = int(self.robot.PasosMaximos)

              self.modelo = HER(policy='MlpPolicy', env=self.robot, model_class=SAC, n_sampled_goal=4,
                        online_sampling=False,
                        goal_selection_strategy='future',
                        verbose=1, buffer_size=buffer_size,  # int(1e6)
                        learning_rate=lr,  # 1e-3
                        gamma=gamma, batch_size=batch_size,  # gama 0.95
                        max_episode_length=max_episode_length,
                        policy_kwargs=dict(net_arch=layers))
        else:
          print("Creando nuevo modelo")
          max_episode_length = int(self.robot.PasosMaximos)
          self.modelo = HER(policy='MlpPolicy', env=self.robot, model_class=SAC, n_sampled_goal=4,
                        online_sampling=False,
                        goal_selection_strategy='future',
                        verbose=1, buffer_size=buffer_size,  # int(1e6)
                        learning_rate=lr,  # 1e-3
                        gamma=gamma, batch_size=batch_size,  # gama 0.95
                        max_episode_length=max_episode_length,
                        policy_kwargs=dict(net_arch=layers))


      #
        # except Exception as e:
        #     print(" CreandoProtocolo ", e)


    def ImportarEnsayo(self, NombreEnsayo):

        # try:

            self.NombreEnsayo = NombreEnsayo

            Carpeta_Mapa = "Ensayos/{}/Mapa".format(self.NombreEnsayo)
            os.makedirs(Carpeta_Mapa, exist_ok=True)

            Carpeta_Plantas= "Ensayos/{}/Plantas".format(self.NombreEnsayo)
            os.makedirs(Carpeta_Plantas,exist_ok=True)

            Carpeta_Imagenes_Plantas = "Ensayos/{}/Imagenes_Plantas".format(self.NombreEnsayo)
            os.makedirs(Carpeta_Imagenes_Plantas, exist_ok=True)

            ImagenesPlantas = []
            Plantas = []
            Marcas=[] ## hay un cuadro de ensayo para cada tipo de planta o cada excel en Carpeta Plantas


            for plantas in os.listdir(Carpeta_Plantas):

                  dir_excel = "{}/{}".format(Carpeta_Plantas,plantas)
                 # print("dirxcel", dir_excel)
                  dfplantas = pd.read_excel(dir_excel)
                  #print("dfPlanta", dfplantas)

                  #quitar el .xsl PARA CREAR LA CARPETA DE IMAGENES
                  plantas = plantas.split(".")[0]
                  #print("plantas_sin_xsl")
                  carpeta_imagenes_plantas = "{}/Imagenes_{}".format(Carpeta_Imagenes_Plantas,plantas)
                  os.makedirs(carpeta_imagenes_plantas,exist_ok=True)


                  #Extraer las plantas y las Imagenes de Las Plantas
                  for n,planta in enumerate (dfplantas["planta"]): ##

                      try:
                          planta = int(planta)
                      except:
                          pass

                      if isinstance(planta, int):
                            nombreimagen = ""
                   #         print("planta",planta)

                            ## revisar cual fue la ultima fotografia de la planta
                           ## ver  si ya hay una carpeta para el nombre de esa planta
                            carpeta_imagen  = "{}/{}".format(carpeta_imagenes_plantas,plantas)
                            os.makedirs(carpeta_imagen, exist_ok=True)

                            # if len(os.listdir(carpeta_imagen)) > 0: ## se carga la ultima imagen de la lista por que se pónen en orden
                            #     nombreimagen = "{}/{}".format(carpeta_imagen,os.listdir(carpeta_imagen)[-1])
                            # else:

                            nombreimagen ="Ensayos/SinImagen.png"
                            ## si no existe la carpeta de la planta entonces crearla

                            Xplanta = dfplantas["x_planta"][n]
                            Yplanta = dfplantas["y_planta"][n]

                            area = 30

                            Plantas.append([Xplanta, Yplanta,area])
                            ImagenesPlantas.append(nombreimagen)
                            ## el nombre de la imagen del mapa

                  #print("Listas_Plantas",Plantas)
                  #print("Imagnes_Plantas", ImagenesPlantas)


                  for n, marca in enumerate(dfplantas["marca"]):  ##

                   #   print("marca", marca)

                      ## revisar cual fue la ultima fotografia de la planta
                      ## ver  si ya hay una carpeta para el nombre de esa planta
                      carpeta_marca = "{}/{}".format(carpeta_imagenes_plantas, plantas)
                      os.makedirs(carpeta_marca, exist_ok=True)


                      Xmarca = dfplantas["x_marca"][n]

                      Ymarca = dfplantas["y_marca"][n]
                      try:
                          marca = int(Xmarca)
                      except:
                          pass

                      if isinstance(marca, int):
                        Marcas.append([Xmarca, Ymarca])
                          ## el nombre de la imagen del mapa

                  #print("Listas_Marcas", Marcas)



            self.robot.Evaluar=False
            self.robot.plantasrandom=False
            self.robot.ImportarMapa(Plantas,ImagenesPlantas,Carpeta_Mapa,Marcas)


        # except Exception as e:
        #     print("importando ensayo", e)

    def EjecutarEnsayo(self):

        # try:

            # Carpeta_Mapa = "Ensayos/{}/Mapa".format(self.NombreEnsayo)
            # os.makedirs(Carpeta_Mapa, exist_ok=True)
            Carpeta_Plantas = "Ensayos/{}/Plantas".format(self.NombreEnsayo)
            os.makedirs(Carpeta_Plantas, exist_ok=True)
            Carpeta_Imagenes_Plantas = "Ensayos/{}/Imagenes_Plantas".format(self.NombreEnsayo)
            os.makedirs(Carpeta_Imagenes_Plantas, exist_ok=True)

            self.ControlManual=False

            for plantas in os.listdir(Carpeta_Plantas):


                dir_excel = "{}/{}".format(Carpeta_Plantas, plantas)
                print("dirPlanta", dir_excel)
                dfplantas = pd.read_excel(dir_excel)
                print("dfPlanta", dfplantas)

                plantas = plantas.split(".")[0]

                carpeta_imagenes_plantas = "{}/Imagenes_{}".format(Carpeta_Imagenes_Plantas, plantas)
                os.makedirs(carpeta_imagenes_plantas, exist_ok=True)

                for tarea in dfplantas["tarea"]:  ##Para cada tarea

                    try:
                        tarea = int(tarea)
                    except:
                        pass

                    if isinstance(tarea, int):

                        Estado = dfplantas["estado"][tarea - 1]
                        print("estado", Estado)

                        if Estado == 0:

                            TiempoTarea = pd.to_datetime(dfplantas["estado"][tarea - 1])

                            TiempoActual = datetime.datetime.now()
                            DiferenciaTiempo = TiempoTarea - TiempoActual

                            Vol = dfplantas["liquido"][tarea - 1]

                            if DiferenciaTiempo.days < 0:  ## por codigo ssi la diferencia de tiempo es negativa solo se ponen los dias negativos

                                for n,planta in enumerate (dfplantas["planta"]):

                                    ##revisar las hubicaciones de las plantas
                                    Xplanta = dfplantas["x_planta"][n]
                                    Yplanta = dfplantas["y_planta"][n]
                                    self.IrA(Xplanta, Yplanta, pasosLimite=10000)
                                    print("regarun volumen de", Vol)
                                    print("para la planta",n)
                                    print("cuando la cantidad de plantas es ",len(dfplantas["planta"]))
                                    self.robot.Regar(Vol)


                                    # Cambiar el estado de 0 a 1 para indicar que ya se cumplio la tarea
                                    # pero cuando se cumpla la tarea para todas las plantas
                                    if n == len(dfplantas["planta"])-1:

                                        print("ultima planta de la tarea")
                                       # guardar dataframe ensayo
                                        dfplantas.iloc[tarea - 1, dfplantas.columns.get_loc("estado")] = 1
                                        #
                                        # Dataframe = pd.DataFrame(dfplantas)

                                        # writer = pd.ExcelWriter(dir_excel, engine='xlsxwriter')
                                        # dfplantas.to_excel(writer,index=False)
                                        # writer.save()
                                        print("guardando a {}".format(dir_excel))
                                        print(dfplantas)
                                        dfplantas.to_excel(r'{}'.format(dir_excel), index=False, header=True)



            # for planta in self.nombresplantas:
            #     ##revisar la hubicacion de la planta
            #     DFplanta = self.DFEnsayo[planta]
            #     print("DFP",DFplanta)
            #
            #     for pos in enumerate (DFplanta["x"]):  ##Para cada tarea

        # except Exception as e:
        #     print("Ejecutandoensayo", e)

    # def GuardarEnsayo(self):


#Protocolo = Protocolo(Nombre="RobotReal250421")

Protocolo = Protocolo(Nombre="RobotVision")
#Protocolo.ImportarEnsayo("Ensayo2")

#Protocolo.EjecutarEnsayo()
#Protocolo.robot.Renderizar(True)
#Protocolo.GenerandoMapa=True

#Protocolo.ControlManual= False

#Protocolo.robot.Regar(140)
#Protocolo = Protocolo(Nombre="Completo")
#Protocolo.NuevoProtocolo(importar_existente=False,lr=0.0005,gamma=0.94,batch_size=512,layers=[256,256,256],buffer_size= int(1e6))
#Protocolo.IrA(90,30,5,100000)
#Protocolo.Entrenar(4,10000,True)
#Protocolo.Mappear(50,90,50,90)

#Protocolo.Optimizar(20)
#Protocolo.VisualizarAnalisis()
#3Protocolo.Evaluar()
#Protocolo.Aprender(50000,True,25000)Protocolo.Aprender(20000,True,5000)

#Protocolo.Aprender(20000,True,1000)
#Protocolo.Aprender(5000,True,500)

#Protocolo.Probar(n_eval_episodes=10,pasosLimite=10000)

#Accuracy: -173.25499361287396 Best hyperparameters: {'Layer1': 512, 'Layer2': 1024, 'Layer3': 1024, 'Layer4': 512, 'batch_size': 512, 'buffer_size': 3441, 'gamma': 0.9654472877569693, 'lr': 0.0016095161311604506