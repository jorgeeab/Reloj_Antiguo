# @title Texto de título predeterminado

import json
import gym
import collections
from collections import OrderedDict
from gym import spaces
import time
import pandas as pd

from YOLO.yolo3opencv2 import Vision
from ShapeColorDetection.ShapeDetection import Figura,DetectordeContornos
from Aruco.ArucoDetection import *

from ConexionRobot import Conexion


# from relojSim_gym.relojSim_env.envs.RobotSimReloj import Robot

#from defisheye import Defisheye

import threading

class RelojSimHierbaEnv(gym.GoalEnv):
    metadata = {'render.modes': ['console']}

    def __init__(self):

        super(RelojSimHierbaEnv, self).__init__()

        self.Discreto = False

        ## Mapa .. esta parte no e
        # ntra en los datos analizados
        self.MaxX = 110  # cm
        self.MinX = 20
        self.MaxY = 20  # cm
        self.MaxA = 360  # Angulo


        self.PixelesPorCentimetro = 28.3 #20

        #self.MargenMapa=

        self.ArcoMaximo = self.LongitudArco(radio=self.MaxX, angulo=self.MaxA)
     #   print("Arco MAXImo", self.ArcoMaximo)
        self.ErrorMaximo = 2

       # self.AumentoCamara = 0.3  # cm/cm   ## la relacion entre la  camara(cm) y la altura o "POSY" (cm)
        RelacionAltoAncho=480/640

        self.anchocamara = 640
        self.altocamara = 480

        self.AreaMaxima = self.altocamara * self.anchocamara

        self.Plantas = []
        self.CantidadPlantas = 15  # el numero de plantas generadas random
        self.ContadorPlantas = 0
        self.DistanciaPlantas = 28
        self.AreaMaximaPlanta = 16
        self.AreaMinimaPlanta = 0.5

        ##Preparar plantas de prueba
        self.PlantasPrueba = []
        areaplantasprueba = 10

       # print( range(self.DistanciaPlantas, self.MaxX*2 - self.DistanciaPlantas,
                            # self.DistanciaPlantas))

        for xplanta in range(self.DistanciaPlantas, self.MaxX*2 ,
                             self.DistanciaPlantas):  # (cm)
            for yplanta in range(self.DistanciaPlantas, self.MaxX*2 , self.DistanciaPlantas):

                a, radio, y = self.CuadroAReloj(xplanta, yplanta, 0, self.MaxX, self.MaxX)

                if radio < self.MaxX and radio > self.MinX:

                    planta = [xplanta,yplanta,areaplantasprueba]
                    self.PlantasPrueba.append(planta)
        #print("Plantasprueba: ",self.PlantasPrueba)

        self.Plantas = self.PlantasPrueba

        self.Hierbas = []
        self.CantidadHierbas = 10
        self.AreaMaximaHierba = 50
        areahierba = 50

        self.HierbasPrueba = []
        for i in range(self.CantidadHierbas):
            if i > 0:
                p = [self.MaxA * (self.CantidadHierbas / i), (self.MaxX - self.MinX) * (self.CantidadHierbas / i),
                     areahierba]
                self.HierbasPrueba.append(p)


        self.PlantasDetectadas = []
        self.HierbasDetectadas = []
        self.NumeroDePlantasTotal = 0
        self.NumeroDeHierbasTotal = 0
        self.NumeroDePlantas = 0
        self.NumeroDeHierbas = 0
        self.ConteoInicial = False







        ##Configuracion

        self.TiempoActual=datetime.datetime.now()

        self.TiempoPaso = 300 # milisegundos
        self.TiempoPasoAnterior = self.TiempoActual


        self.Atrazo = 300
        
        self.AtrazoMinimo= 300
        self.AtrazoMaximo = 1500  # milisegundos


                                  #La velocidad cambia con el porcentage de aceleracion 
                                  # puede ser negativa
        self.VelocidadA = 0.0  ## grados /milisegundos
        self.VelocidadAMaxima = 0.05
        self.VelocidadAMinima = -0.05


        self.VelocidadX = 0.0 ## cm/ milisegundos .. ejecucion cada intervalo de comunicacion
        self.VelocidadXMaxima = 0.002
        self.VelocidadXMinima = -0.002
      


        ## suponiendo que hacen falta centimetros
        self.VelocidadY = 0.0
        self.VelocidadYMaxima = 0.005
        self.VelocidadYMinima = -0.005

        self.AceleracionA = 0.3*self.VelocidadAMaxima   ## es la cantidad de  pasos x segundo que aumenta o disminiye  la velocidad 
        self.AceleracionX =0.3*self.VelocidadXMaxima 
        self.AceleracionY =0.3*self.VelocidadYMaxima


        self.TiempoVelocidadAnterior = self.TiempoActual
        self.TiempoVelocidad = 700  # milisegundos


        self.Paso = 0


        self.ListaMovimientos=[]

        for TP in range (30):
            movimiento =[0,0,0,0,self.TiempoActual]

            self.ListaMovimientos.append(movimiento)


        self.Detector = DetectordeContornos(l_h=40, u_h=180, l_s=0, u_s=255, l_v=60, u_v=255)

        self.Figura_1 = Figura(nombre="Planta", Figura='Circle',area_minima=1000,area_maxima =1000)

        self.Figura_2 = Figura(nombre="Hierba", Figura='Circle',area_minima=300,area_maxima =5000)


        #DetectorYolo
        self.DetectorPlantas =Vision()
        self.ObjectsList = []
        self.NumeroObjetos = 2



        # = max(cm)*(1/ VelmaxX)(p/cm)
        # = maximospasosenX(p)
        self.TiempoEjecucion =   1000*60*1.5 # ( miliseguntos)

        self.PasosMaximos = int( self.TiempoEjecucion / self.TiempoPaso)


        #print("Pasos Maximos", self.PasosMaximos)
        self.max_episode_steps = self.pasosmaximos() ## Variable para el eviroment gym structure

        self.NombresActuadores =[ "EMX", "EMA"]

        self.EMA = 0
        self.EMX = 0
        self.EMY = 0
        self.EBE = 0

        self.VOLRequerido =0 ##EL VOLUMEN REQUERIDO PARA REGAR



        #self.Acciones = np.array([self.EMX, self.EMY, self.EMA, self.EBE])
        self.Acciones = np.array([self.EMX, self.EMA])

        n_actions = len(self.Acciones)

        # self.reward_range = range(-1,1)
        self.PosA = 0  # angulo
        self.PosX = self.MinX  # cm
        self.PosY = 0  # cm/

        ##Detectado por el Robot
        self.PosAR = 0  # angulo
        self.PosXR = self.MinX  # cm
        self.PosYR = 0  # cm/

        self.PosVelA = self.PosA  # cm  Para marcR L posicion inicial de las variables antesque pase el
        self.PosVelX = self.PosX  # cm
        self.PosVelY = self.PosY  # angulo


        self.PosAAnterior = self.PosA  # cm
        self.PosXAnterior = self.PosX  # cm
        self.PosYAnterior = self.PosY  # angulo


        self.EMXDetectado=0
        self.EMYDetectado=0  ## Los Actuadores recividos desde el arduino,,
        self.EMADetectado=0  ##  ayudaría a determinar el tiempo de viaje de los datos

        self.VOLDetectado = 0 ## EL VOLUMEN DE AGUA RESTANTE QUE REGAR SEGUN EL ARDUINO

        self.AnguloDetectado = 0
        self.RadioDetectado = 0


        self.PosXDetectado = 0
        self.PosYDetectado = 0
        self.PosZDetectado = 0

        self.PosXDetectadoAnterior = self.PosXDetectado
        self.PosYDetectadoAnterior = self.PosYDetectado
        self.PosZDetectadoAnterior = self.PosZDetectado


        self.Error = 0  # %Error

        self.F1x = 0  # cm
        self.F1y = 0  # cm
        self.F1a = 0  # cm
        self.F1d = 0  # cm

        self.F2x = 0  # cm
        self.F2y = 0  # cm
        self.F2a = 0  # cm
        self.F2d =0  # cm

           # Punto de control identificado en la pantalla ( mascercano
           # Los puntos de control son los mismos marcadores aruco
           # un marcador aruco que ademas se relaciona directamente con una coordenda

        self.FMx=0
        self.FMy=0
        self.FMd=0

        self.PMx = 0
        self.PMy = 0
        self.PMd = 0

        self.Tx=0
        self.Ty=0

        self.hora = 0
        self.minuto = 0
        self.segundo = 0
        self.milisegundo = 0
        # objetivo

        # un area de accion que se debe limpiar (rango de PosX,PosY)

        self.XMinima = 0
        self.XMaxima = 0
        self.YMinima = 0
        self.YMaxima = 0

        self.XRequerido =(self.XMaxima-self.XMinima)/2
        self.YRequerido =(self.YMaxima-self.YMinima)/2

        self.AnguloRequerido, self.RadioRequerido , self.ZRequerido  = self.CuadroAReloj(self.XRequerido,self.YRequerido,0,self.MaxX,self.MaxX)


        self.PlantasActuales =[]
        self.HierbasActuales = [] # Las Hierbas detectadas cuando el porcentage de
        self.PuntosMapeo = []


        self.PorcentajeHierbas = 0  # porcentage hierbas de a1 ESTA RECOMPENSA NO ES MEDIDA EN EL MUNDO REAL
        self.PorcentajePlantas = 0
        self.PorcentajeMapa = 0



        self.Especies = [ "lechuga"]



        ## Este Margen se grega para dibujar la imagen de fondo
        self.MargenSimulacionMapa = self.altocamara*2 #(pixeles )

        self.AltoMapa=  int(    2*self.MargenSimulacionMapa  +self.MaxX*2*self.PixelesPorCentimetro) #(pixeles)
        self.AnchoMapa= int(    2*self.MargenSimulacionMapa  +self.MaxX*2*self.PixelesPorCentimetro)#(pixeles)

        # importar la imagen de fondo del mapa
        self.fondo = cv2.imread("C:/Users/jorge/Desktop/PycharmProjects/ProyectoReloj/Imagenes/tierra/fondos/fondotierra.jpg", cv2.IMREAD_COLOR)
        ## POner un margen al fondo y cortar a una longitud especifica .. Para que los marcadores coincidan
        # fondo = fondo[0:self.AltoMapa, 0:self.AnchoMapa]

        self.fondo = cv2.resize(self.fondo, (self.AnchoMapa, self.AltoMapa))

        self.CentroXSimulacionMapa = self.AnchoMapa/2 #(Pixeles)
        self.CentroYSimulacionMapa = self.AltoMapa/2 #(Pixeles)

        # fondo = cv2.imread("Imagenes/tierra/fondotierra.jpg", cv2.IMREAD_COLOR)
        # fondo = cv2.resize(fondo, (self.AnchoMapa, self.AltoMapa))

        self.ImagenesPlantas = []

        for especie in os.listdir("Imagenes/Plantas/"):
            for planta in os.listdir("Imagenes/Plantas/{}".format(especie)):
                imagen = cv2.imread("Imagenes/Plantas/{}/{}".format(especie, planta), cv2.IMREAD_COLOR)
                self.ImagenesPlantas.append(imagen)

        self.simulacionmapa =self.fondo
            #self.simulacionmapa[0:self.AltoMapa,0:self.AnchoMapa]


        # cortar para que quede del tamaño del huerto

        self.imagencamara = self.simulacionmapa
        self.Mapa = self.imagencamara

        #Aruco Marks
        self.DistanciaMarcas= 20 #(cm)
        self.LadoMarcas = 3#(cm)

                                               ##en cm
        self.Mappeador=MapeadorAruco(anchomapa=self.MaxX*2,altomapa=self.MaxX*2,altocamara=self.altocamara,anchocamara=self.anchocamara,
                                     PixelesPorCentimetro=self.PixelesPorCentimetro,DistanciaMarcas=self.DistanciaMarcas,LadoMarcas=self.LadoMarcas,
                                     MaxX=self.MaxX,MinX=self.MinX,MaxA=self.MaxA,MargenMapa=10)

        print("Calibrando la Camara")

        self.Mappeador.Calibrar()


        self.PM = 100 #Porcentaje deseado de Mapa
        self.DP = 0  # Distancia al siguiente punto de mapeo

        self.PP = 100 # El porcentage de las plantas detectdas inicialmenbte
        self.DF1 = 0  # Distancia A la figura de la Planta más Cercana

        self.PH = 100  # Porcentaje de lasHierbas Detectadas inicalmente
        self.DF2 = 0  # Distancia A La Fifura de La hierba Más Cercana

        self.E = 0 #4



        self.Evaluar = False
        self.ContadorEvaluaciones = 0

        self.NombresSensores = ["AnguloRequerido","RadioRequerido", "XRequerido", "YRequerido", "ZRequerido",
                                "AnguloDetectado","RadioDetectado", "PosXDetectado", "PosYDetectado", "PosZDetectado",
                                "Error",
                                # "F1x", "F1y", "F1a","F1d",
                                # "F2x", "F2y", "F2a","F2d",
                                # "FMx", "FMy","FMd",
                                # "PMx","PMy" ,"PMd",
                                # "Tx","Ty",  ## Puntos que seguimiento para aumentar la informacion sensada
                                "hora", "minuto", "segundo", "milisegundo",  ## para estandarizar tiempo
                                "XMinima", "XMaxima", "YMinima", "YMaxima",
                                ##Los cuatro puntos delimitadores de la zona  cuadrada en la que se esta trabajando
                                # "PorcentajeMapa", "PorcentajePlantas","PorcentajeHierbas",   ## punto de mapeo más cercano
                                "EMXDetectado", "EMYDetectado", "EMADetectado"
                                ]

        self.ObservacionSimple = np.array(

            [self.AnguloRequerido,self.RadioRequerido, self.XRequerido ,self.YRequerido,self.ZRequerido ,
             self.AnguloDetectado,self.RadioDetectado, self.PosXDetectado, self.PosYDetectado, self.PosZDetectado,
             self.Error,
             # self.F1x, self.F1y, self.F1a,self.F1d,
             # self.F2x, self.F2y, self.F2a,self.F2d,
             # self.FMx, self.FMy,self.FMd,
             # self.PMx, self.PMy , self.PMd,
             # self.Tx,self.Ty,  ## Puntos que seguimiento para aumentar la informacion sensada
             self.hora, self.minuto, self.segundo, self.milisegundo,## para estandarizar tiempo
             self.XMinima, self.XMaxima, self.YMinima, self.YMaxima, ##Los cuatro puntos delimitadores de la zona  cuadrada en la que se esta trabajando
             #self.PorcentajeMapa, self.PorcentajePlantas, self.PorcentajeHierbas,   ## punto de mapeo más cercano
             self.EMXDetectado,self.EMYDetectado,self.EMADetectado

             ])

        self.ObservacionesMaximas = np.array(

            [self.MaxA,self.MaxX, self.MaxX*2,self.MaxX*2,self.MaxY,
             self.MaxA,self.MaxX,self.MaxX*2,self.MaxX*2, self.MaxY,
             self.ErrorMaximo,
             # self.anchocamara, self.altocamara, self.AreaMaxima,self.anchocamara,
             # self.anchocamara, self.altocamara, self.AreaMaxima,self.anchocamara,
             # self.anchocamara, self.altocamara, self.anchocamara,
             # self.MaxX*2, self.MaxX*2, self.MaxX*2,
             # self.anchocamara,self.anchocamara,
             24, 60, 60, 1000,
             self.MaxX*2, self.MaxX*2, self.MaxX*2, self.MaxX*2,
             #100,100,100,## AMximo porcentaje
             1,1,1## Maxima Potencia de los Motores de 0 a 1
             ])

        self.EspacioMemoria = 5

        ObservacionesNormalizadas = np.divide(self.ObservacionSimple, self.ObservacionesMaximas)

        Memoria = np.array([ObservacionesNormalizadas for i in range(self.EspacioMemoria)])
        self.Memoria = collections.deque([i for i in Memoria]) ## deque de memoria
             
        self.Memoria.append(ObservacionesNormalizadas)
        self.Memoria.popleft()
        #print("Memoria",self.Memoria)

        n_Observaciones = len(ObservacionesNormalizadas) * self.EspacioMemoria
        # ARRAY DE oBSERVACION

        self.Observaciones = []
        lista_obs = []

        for linea in list(self.Memoria):
          for valor in linea:        
            lista_obs.append(valor)

        if len(lista_obs) == n_Observaciones:

            self.Observaciones = lista_obs
            print("Observaciones",self.Observaciones)

        # el po'rcentage de hieerbas quitadas
        ### Experimentar sin incluirlo      X          Y          Z
        #        self.ObjetivoMaximo = np.array([self.MaxX*2,self.MaxX*2,self.MaxY,   100, self.MaxX*3,   100,self.anchocamara,   100,self.anchocamara ,self.ErrorMaximo])
        self.ObjetivoMaximo = np.array(
            [self.MaxA,self.MaxX,self.MaxX * 2, self.MaxX * 2, self.MaxY, self.ErrorMaximo])

        # self.PM, self.DP,  self.PP, self.DF1,     self.PH,self.DF2,     self.E

        # incluyendo porm ahora
        # self.ObjetivoDeseado = np.array(
        #     [self.XRequerido ,self.YRequerido,self.ZRequerido ,
        #      self.PM, self.DP,self.PP, self.DF1,self.PH,self.DF2,self.E ])
        # self.ObjetivoDeseado =np.divide(self.ObjetivoDeseado , self.ObjetivoMaximo)
        self.ObjetivoDeseado = np.array(
            [self.AnguloRequerido,self.RadioRequerido,self.XRequerido, self.YRequerido, self.ZRequerido, self.E])
        self.ObjetivoDeseado = np.divide(self.ObjetivoDeseado, self.ObjetivoMaximo)

        self.PesosoObjetivos = np.array([0.2,0.2,0.2,0.2,0.0,0.0])  ## Para Moverse a un punto especifico
        #        self.PesosoObjetivos = np.array([0.0,0.0,0.0,0.3, 0.5, 0.0, 0.0, 0.0, 0.0, 0.2])## Para Mapear

        #        self.PesosoObjetivos = np.array([0.0,0.0,0.0,0.0, 0.0, 0.0, 0.2, 4.0, 0.0, 0.2])  ## Para Regar

        #        self.PesosoObjetivos = np.array([0.0,0.0,0.0,0.2, 0.2, 0.0, 0.2, 4.0, 0.0, 0.2])  ## Para Deshierbar

        self.ObjetivoAlcanzado = np.array(
            [self.AnguloDetectado,self.RadioDetectado,self.PosXDetectado, self.PosYDetectado, self.PosZDetectado,
             self.Error])  # el valor Actual de los Objetivos




        # self.F2y,          self.F2a,     self.hora, self.minuto, self.segundo, self.milisegundo,self.AnguloMinimo, etc..
        self.RangoRecompensaFinal = -0.1

        n_objetivos = len(self.ObjetivoDeseado)



        self.observation_space = spaces.Dict({
            'observation': spaces.Box(low=0, high=1, shape=(n_Observaciones,), dtype=np.float32),
            'achieved_goal': spaces.Box(low=0, high=1, shape=(n_objetivos,), dtype=np.float32),
            'desired_goal': spaces.Box(low=0, high=1, shape=(n_objetivos,), dtype=np.float32)

        })

        # self.observation_space = spaces.Box(low=0, high=1, shape=(n_Observaciones,), dtype=np.float32)

        self.action_space = spaces.Box(-1, 1, shape=(n_actions,),
                                       dtype=np.float32)  # The observation will be the coordinate of the agent

        self.PasoAnterior = 0

        ## Variables para guardar diagnosticos

        self.PasosMaximosDiagnostico = 1000
        self.PasosDiagnostico = 0
        self.ArrayDiagnosticoActuadores = []
        self.ArrayDiagnosticoSensores = []

        self.EnvMode = False


        self.objetivosrandom = True  # cuando se activan los comandos random cada vez que se resetee se cambian random los Objetivos jajaja
        self.hierbasrandom = True
        self.plantasrandom = True
        self.reestablecerposiciones = False
        self.reiniciarmapa = True
        self.Renderizando = False  ## coloca una pausa en cada paso
        self.Manual = False
        self.DiagnosticarParametros = True


        self.ModoVirtual = True

        self.SinImagen=True


        self.wifi = True
        self.online =False

        ## Conexion con robot

        if self.ModoVirtual == False:

            # propiedades de la conexion
            ## Conexion con robot
            self.ConexionRobot = Conexion(wifi=self.wifi,online= self.online,puerto=10,baudios=9600)
            self.ConexionRobot.IniciarConexion()


            self.Camara = cv2.VideoCapture(2)



        else :

            self.GenerarMapa()

            # colores
        self.rojo = (0, 0, 255)
        self.verde = (0, 255, 0)
        self.azul = (255, 0, 0)
        self.blanco = (255, 255, 255)
        self.amarillo = (0, 255, 255)

    def DiagnosticoParametros(self):## genera unas

      ##
        # try:
            self.PasosDiagnostico += 1
            print(self.PasosDiagnostico)

        ## ir guardando los pasos  dados para luego meterlos en una red neurona
        #Primero se ejecutan cierto numero de pasos para establecer conexion
            if self.PasosDiagnostico > self.PasosMaximosDiagnostico:

                 #print("Guardando Diagnostico ")
                # Guardar el Diagnóstico como un archivo excel con pandas

                 Dfsensores = pd.DataFrame(self.ArrayDiagnosticoSensores,columns=self.NombresSensores)

                 Dfacciones = pd.DataFrame(self.ArrayDiagnosticoActuadores,columns=self.NombresActuadores)
                # print(Dfacciones)

                 #Verificar cuantos Diagnosticos hay

              #   print(Dfsensores)
                 try:
                    NDiagnistico = len(os.listdir("Diagnostico"))+1
                 except:
                     NDiagnistico =0

                 Dfsensores.to_excel(r"Diagnostico/DiagnosticoSensores{}.xlsx".format(NDiagnistico), index = False, header=True)
                 Dfacciones.to_excel(r"Diagnostico/DiagnosticoAcciones{}.xlsx".format(NDiagnistico), index = False, header=True)

                ## rEESTABLECER LOS CONTADORES

                 self.PasosDiagnostico = 0
                 self.ArrayDiagnosticoActuadores = []
                 self.ArrayDiagnosticoSensores = []

            else:

                accion = self._get_action()
                #print("accion",accion)
                obs = self.ObservacionSimple

              #  print("Observation",obs)
                self.ArrayDiagnosticoActuadores.append(accion)
                #print(".ArrayDiagnosticoActuadores",self.ArrayDiagnosticoActuadores)
                self.ArrayDiagnosticoSensores.append(obs)

        # except Exception as e :
        #      print("Recuperando Diagnostico ",e)
    def RecibirDatos(self):

        if self.wifi:


               try:
                    entrada = "0.0,0.0,0.0,0.0,0.0"

                    entrada = self.ConexionRobot.Recibir()

                    if len(entrada) < 1 :
                        entrada = "0.0,0.0,0.0,0.0,0.0"
                    else:
                        print("entrada : ", entrada)

                    for n, i in enumerate(entrada.split(',')):
                        val = float(i)  # using float because you don't only have integers
                        if n == 0:
                            self.EMADetectado = val
                        if n == 1:
                            self.EMXDetectado = val
                        if n == 2:
                            self.VOLDetectado = val
                        if n == 3:
                            self.PosXR = val   ## el valor a la posicion x suministrada por el robot
                           # print(self.PosX)
                        if n == 4:
                            self.PosAR = val   ## el valor a la posicion del angulo suministrada por el robot

               except Exception as e :

                 #  print("entrada : ", entrada)
                   print("errpr recibiendo datos ")



        else:

            entrada = self.ConexionRobot.Recibir()
            if len(entrada) < 1:
                entrada =  "{\"Actuadores\":{\"EMX\":[%s],\"EMA\":[%s],\"VOL\":[%s],\"PosX\":[%s],\"Angulo\":[%s]}}" % (0, 0, 0,0,0)

            self.Data = json.loads(entrada)
           # print(self.Data)
            self.Actuadores = pd.DataFrame(self.Data['Actuadores'])  # las variables que no controlo
            self.EMADetectado = self.Data['Actuadores']['EMA'][0]
            self.EMXDetectado = self.Data['Actuadores']['EMX'][0]
            self.VOLDetectado = self.Data['Actuadores']['VOL'][0]
            self.PosX = int( self.Data['Actuadores']['PosX'][0])
            self.PosA = self.Data['Actuadores']['Angulo'][0]



    def EjecutarAccion(self, EMX,EMY, EBE, EMA,VOL, Reset):

           if self.wifi:

                textoAenviar = str(round(EMA,2))+","+str(round(EMX,2))+","+str(round(VOL,2))+","
                print("texto enviar : ", textoAenviar)
                self.ConexionRobot.Enviar(texto=textoAenviar)

           else:

               textoAenviar = "{\"Actuadores\":{\"EMX\":[%s],\"EMA\":[%s],\"VOL\":[%s]}}" % (EMX, EMA,self.VOLRequerido)
               self.ConexionRobot.Enviar(texto=textoAenviar)


    def ObtenerFrame(self):

        try:
                r,Frame = self.Camara.read()
                return Frame

        except  Exception as e:
            print("vision", e)


    def Coseno(self, grados):
        return math.cos(math.radians(grados))

    def ArcoCoseno(self, radianes):
        return math.degrees(math.acos(radianes))


    def GenerarMapa(self):

        try:

            fondo = self.fondo

            contadorimagenes = 0

            for planta in self.Plantas:
                #print("Planta")

                try:

                    if contadorimagenes < len(self.ImagenesPlantas):

                        contadorimagenes += 1
                    else :
                        contadorimagenes = 0

                        ## Algoritmo para pegar la img2 sobre la img1
                        # Load two images
                        mata = self.ImagenesPlantas[contadorimagenes]

                        # I want to put logo on top-left corner, So I create a ROI
                        # print(ImagenesPlantas[contadorimagenes])

                        # print("planta[0]", planta[0])
                        # print("planta[1]", planta[1])
                        ## El centro de x y Y se ajustan al dibujar las matas

                        xplanta, yplanta, areaplanta = planta[0], planta[1], planta[2]  ## en (cm)

                        anchomata = int(math.sqrt(
                            areaplanta) * self.PixelesPorCentimetro)  ##encontrar el tamaño en pixeles de las plantas para pegarlas
                        altomata = int(math.sqrt(areaplanta) * self.PixelesPorCentimetro)

                        mata = cv2.resize(mata, (anchomata, altomata), interpolation=cv2.INTER_AREA)
                        # PasarA pixeles
                        xplanta = int(xplanta * self.PixelesPorCentimetro)
                        yplanta = int(yplanta * self.PixelesPorCentimetro)

                        ## Para dibujar la hub se pasa a pixeles y y se para el margen
                        fondo = self.DibujarPlanta(fondo, self.MargenSimulacionMapa, mata, xplanta, yplanta)

                except Exception as e:
                        print("No se agregó la planta  ", e)


            ##Pasar la imagen y el margen para que las hubicaciones se carguen correntamente tomando en cuenta el margen

            fondo = self.Mappeador.DibujarMarcas(fondo,self.MargenSimulacionMapa)

            self.simulacionmapa = fondo

        except Exception as e:
            print("GENERANDO MAPA ", e)

    def SimularMovimiento(self,EMX, EMA ,EMY, EBE):

        try:
            #Realizar La simulacion de los movimientos por el munto virtual


            ## almacenar el movimiento en la lista
            movimiento=[EMX,EMA,EMY,EBE,self.TiempoActual]
            self.ListaMovimientos=[movimiento]+self.ListaMovimientos
            self.ListaMovimientos.pop()
            #print(self.ListaMovimientos)

            ## revisar la lista tapra verificar si se deben ejecutar pasos
            ## que ya paso el tiempo que se le destino al Atrazo
            contadortiempo=0
            for movimiento in self.ListaMovimientos:
                 #print("aja",movimiento)

                 tiempomovimiento = movimiento[4]

                 dt = self.TiempoActual - tiempomovimiento
                 diferenciaenms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 100

                 if diferenciaenms > self.Atrazo:

                    EMX=movimiento[0]
                    EMA=movimiento[1]
                    EMY=movimiento[2]
                    EBE=movimiento[3]

                    self.EMXDetectado=EMX
                    self.EMADetectado=EMA
                    self.EMYDetectado=EMY
                    self.EBEDetectado=EBE  # moviendo angulo
                    
                    DifVelA = EMA*self.AceleracionA
                    if EMA >= 0.4 and self.VelocidadA+DifVelA <= self.VelocidadAMaxima:
                        self.VelocidadA += DifVelA
                    if EMA <= -0.4 and self.VelocidadA-DifVelA >= self.VelocidadAMinima:
                        self.VelocidadA += DifVelA
                    if  -0.4 < EMA < 0.4: # DesAcelerar
                       self.VelocidadA=0
                    NuevaPosA =  self.PosA + self.VelocidadA * self.TiempoPaso ## la velocidad por lo que duro el paso anterior para deteminar cuanto se movio
                    if NuevaPosA <= self.MaxA and NuevaPosA >=0:
                        self.PosA =  NuevaPosA 

                    
                    # moviendo X
                    DifVelX = EMX*self.AceleracionX

                    if EMX > 0.4 and self.VelocidadX+DifVelX <= self.VelocidadXMaxima:
                        self.VelocidadX += DifVelX
                    if EMX < -0.4 and self.VelocidadX-DifVelX >= self.VelocidadXMinima:
                        self.VelocidadX += DifVelX
                    if -0.4 < EMX < 0.4:  # DesAcelerar
                        self.VelocidadX =0


                    #print("Vel", self.VelocidadX)
                    NuevaPosX =  self.PosX + self.VelocidadX * self.TiempoPaso ## la velocidad por lo que duro el paso anterior para deteminar cuanto se movio
                    if NuevaPosX <= self.MaxX and NuevaPosX >=self.MinX:
                        self.PosX =  NuevaPosX

                    # moviendo Y

                    DifVelY = EMY*self.AceleracionY
                    if EMY > 0.4 and self.VelocidadY+DifVelY <= self.VelocidadYMaxima:
                        self.VelocidadY += DifVelY
                    if EMY < -0.4 and self.VelocidadY-DifVelY >= self.VelocidadYMinima:
                        self.VelocidadY += DifVelY
                    if -0.4 < EMY < 0.4:  # DesAcelerar
                      self.VelocidadY = 0
                    NuevaPosY =  self.PosY + self.VelocidadY* self.TiempoPaso ## la velocidad por lo que duro el paso anterior para deteminar cuanto se movio
                    if NuevaPosY <= self.MaxY and NuevaPosY >=0:
                        self.PosY =  NuevaPosY
                    break

        except Exception as e :
            print("Simualdo movimiento",e)




    def SimularCamara(self):

        camara = self.simulacionmapa

        try:

            xcamara,ycamara,z =0,0,0
            angulo=0


            xcamara ,ycamara ,z =self.RelojACuadro(self.PosA,self.PosX,0,self.MaxX,self.MaxX)



            if self.PosA >= 0 and self.PosA < 90:
                angulo = self.PosA - 90

            if self.PosA >= 90 and self.PosA < 180:
                angulo = self.PosA + 270


            if self.PosA >= 180 and self.PosA < 270:
                angulo = self.PosA - 90


            if self.PosA >= 270 and self.PosA < 860:
                angulo = self.PosA +270



            #Pasar en pixeles
            xcamara=xcamara*self.PixelesPorCentimetro
            ycamara=ycamara*self.PixelesPorCentimetro

            roi =self.simulacionmapa[ int(self.MargenSimulacionMapa+ycamara-(self.altocamara*1.5/2)) : int(self.MargenSimulacionMapa+ycamara + (self.altocamara*1.5/2) ) ,
                 int(self.MargenSimulacionMapa+ xcamara -(self.anchocamara*1.5/2)) : int(self.MargenSimulacionMapa+xcamara + (self.anchocamara*1.5/2)) ]
            scale = 1
            hroi,wroi = roi.shape[:2]
            center = (wroi/2,hroi/2)


            # Rotar la imagen
            M = cv2.getRotationMatrix2D(center, angulo, scale)
            roi = cv2.warpAffine(roi, M, (wroi, hroi))
            hroi, wroi = roi.shape[:2]


            camara= roi[int((hroi/2 - self.altocamara/2)):int((hroi/2 +self.altocamara/2)), int((wroi/2 -self.anchocamara/2)):int((wroi/2 +self.anchocamara/2))]

     #####  Quitar la istorcion por la camara
            # mtx,dist = self.Mappeador.Calibracion()
            # h, w = camara.shape[:2]
            # newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
            # # undistort
            # camara = cv2.undistort(camara, mtx, dist, None, newcameramtx)
            # # crop the image
            # x, y, w, h = roi
            # camara = camara[y:y + h, x:x + w]




        #OJO si la imagen se queda fija durante un segundo entonces se procesa la imagen
        # Para asegurarse de que el motor este quieto y tome bien la foto y además para
        # no dejar los motores moviendose por el tiempo que le toma procesar la imagen
        #if self.PosA == self.PosAAnterior and self.PosX == self.PosXAnterior :
        # agregar luego la condicion con posy


        except Exception as e:
            #print("Ocurrió un error simulando la camara ",e)

            camara =np.ones((int(self.altocamara), int(self.anchocamara ), 3),np.uint8) * 255
            pass

        return camara



         ##fUNCION PARA PASAR DE ANGULO A OS RECTANGULAR .. en cm

    def ProcesarSensores(self):

        if self.ModoVirtual:

            if self.VelocidadA < self.VelocidadAMaxima*0.2 or self.VelocidadA > self.VelocidadAMinima*0.2 :

                self.PosXDetectado, self.PosYDetectado ,self.PosZDetectado =self.RelojACuadro(self.PosA,self.PosX,self.PosY,self.MaxX,self.MaxX)

                self.AnguloDetectado = self.PosA
                self.RadioDetectado =self.PosX

        else:

            self.PosXDetectado, self.PosYDetectado, self.PosZDetectado = self.RelojACuadro(self.PosAR, self.PosXR,self.PosYR,self.MaxX,self.MaxX)
            self.AnguloDetectado = self.PosAR
            self.RadioDetectado = self.PosXR


            dt = self.TiempoActual - self.TiempoVelocidadAnterior
            diferenciaenms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 100

            if diferenciaenms > self.TiempoVelocidad:

                self.TiempoVelocidadAnterior = self.TiempoActual

                difAngulo = math.fabs(self.PosVelA - self.AnguloDetectado)

                self.PosVelA = self.AnguloDetectado

                self.VelocidadA = difAngulo / diferenciaenms
                if self.VelocidadA > self.VelocidadAMaxima:
                    self.VelocidadAMaxima = self.VelocidadA

                #  print("Velocidad AMayor", self.VelocidadAMaxima)

                difPosX = math.fabs(self.PosVelX - self.PosXDetectado)
                self.PosVelX = self.PosXDetectado
                self.VelocidadX = difPosX / diferenciaenms

                if self.VelocidadX > self.VelocidadXMaxima:
                    self.VelocidadXMaxima = self.VelocidadX
                # print("Velocidad XMayor", self.VelocidadXMaxima)

                difPosY = math.fabs(self.PosVelY - self.PosYDetectado)
                self.PosVelY = self.PosYDetectado
                self.VelocidadY = difPosY / diferenciaenms

                if self.VelocidadY > self.VelocidadYMaxima:
                    self.VelocidadYMaxima = self.VelocidadY
            #  print("Velocidad YMayor", self.VelocidadYMaxima)

        # self.FMx, self.FMy, \
        # self.FMd, self.Tx, self.Ty, self.PMx, self.PMy, self.PMd, self.PorcentajeMapa
        #



    def ProcesarImagen(self,camara):

        if self.VelocidadA < self.VelocidadAMaxima * 0.2 or self.VelocidadA > self.VelocidadAMinima * 0.2:

            self.PosAAnterior = self.PosA  # angulo
            self.PosXAnterior = self.PosX  # cm
            self.PosYAnterior = self.PosY  # cm


            self.PosXDetectadoAnterior = self.PosXDetectado  # cm
            self.PosYDetectadoAnterior = self.PosYDetectado  # cm
            self.PosZDetectadoAnterior = self.PosZDetectado  # cm


            try:


                camara, self.PosXDetectado, self.PosYDetectado, self.PosZDetectado,Angulo, self.FMx, self.FMy, \
                self.FMd, self.Tx, self.Ty, self.PMx, self.PMy, self.PMd, self.PorcentajeMapa = \
                    self.Mappeador.DetectarHubicacion(camara, PosXAnterior=self.PosXDetectado,
                                                      PosYAnterior=self.PosYDetectado, PosZAnterior=self.PosZDetectado,)
                ## Pasar las posiciones detectadas a angulos para poder secar la velocidad

                PosADetectado, PosXDetectado, PosYDetectado = self.CuadroAReloj(self.PosXDetectado,
                                                                                self.PosYDetectado,
                                                                                self.PosZDetectado,
                                                                                self.MaxX, self.MaxX)
                self.AnguloDetectado = PosADetectado
                self.RadioDetectado = PosXDetectado

                if self.ModoVirtual == False:


                    dt = self.TiempoActual - self.TiempoVelocidadAnterior
                    diferenciaenms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 100


                    if diferenciaenms > self.TiempoVelocidad:

                        self.TiempoVelocidadAnterior = self.TiempoActual

                        difAngulo = math.fabs(self.PosVelA - PosADetectado)

                        self.PosVelA = PosADetectado

                        self.VelocidadA = difAngulo / diferenciaenms
                        if self.VelocidadA > self.VelocidadAMaxima:
                            self.VelocidadAMaxima = self.VelocidadA
                      #  print("Velocidad AMayor", self.VelocidadAMaxima)


                        difPosX = math.fabs(self.PosVelX - PosXDetectado)
                        self.PosVelX = PosXDetectado
                        self.VelocidadX = difPosX / diferenciaenms

                        if self.VelocidadX > self.VelocidadXMaxima:
                            self.VelocidadXMaxima = self.VelocidadX
                       # print("Velocidad XMayor", self.VelocidadXMaxima)

                        difPosY = math.fabs(self.PosVelY - PosYDetectado)
                        self.PosVelY = PosYDetectado
                        self.VelocidadY = difPosY / diferenciaenms

                        if self.VelocidadY > self.VelocidadYMaxima:
                            self.VelocidadYMaxima = self.VelocidadY
                      #  print("Velocidad YMayor", self.VelocidadYMaxima)


            except Exception as e:

                    print("detrectando hubicacion", e)

            ##Rectangulo de LA MIRA

            self.wmira = int(self.anchocamara * 0.1)
            self.hmira = self.wmira
            self.xmira = int(self.anchocamara / 2 - self.wmira / 2)
            self.ymira = int(self.altocamara / 2 - self.hmira / 2)

            colormira = self.rojo

            cv2.rectangle(camara, (self.xmira, self.ymira), (self.xmira + self.wmira, self.ymira + self.hmira),
                          colormira, 1)


            if self.PosXDetectado != self.PosXDetectadoAnterior \
                    or self.PosYDetectado != self.PosYDetectadoAnterior  \
                    or self.PosZDetectado != self.PosZDetectadoAnterior:

                #  ##Para cada color
                contours = self.Detector.Detectar(camara)
                #  ## luego de cala color las figuras
                camara, self.F1x, self.F1y, self.F1a, self.F1d, NumeroDeFiguras = self.Figura_1.Detectar(camara,
                                                                                                         contours)
                camara, self.F2x, self.F2y, self.F2a, self.F2d, NumeroDeFiguras = self.Figura_2.Detectar(camara,
                                                                                                         contours)
                # mantener areas positivas
                if self.F1a < 0:
                    self.F1a = 0
                if self.F2a < 0:
                    self.F2a = 0

        return camara

    def RelojACuadro(self,PosA,PosX,PosY,CentroX,CentroY):

        xdetectado, ydetectado, zdetectado = 0,0,0

        if  PosA >= 0 and PosA < 90:
            xdetectado = int(CentroX - (PosX * self.Coseno(PosA)))
            ydetectado = int(CentroY - (PosX * self.Coseno(90 - PosA)))
            zdetectado = PosY

        if  PosA >= 90 and PosA < 180 :
            xdetectado = int(CentroX + PosX * self.Coseno(180 - PosA))
            ydetectado = int(CentroY - PosX * self.Coseno(PosA - 90))
            zdetectado = PosY

        if  PosA >= 180 and PosA < 360:

            xdetectado = int(CentroX + PosX * self.Coseno(180 + PosA))
            ydetectado = int(CentroY + PosX * self.Coseno(270-PosA))
            zdetectado = PosY

        if  PosA >= 360 and PosA < 0 :

            xdetectado = int(CentroX - (PosX * self.Coseno(360-PosA)))
            ydetectado = int(CentroY + (PosX * self.Coseno( PosA-270)))
            zdetectado = PosY


        return  xdetectado,ydetectado,zdetectado

    def CuadroAReloj(self, PosX, PosY, PosZ, CentroX, CentroY):

        adetectado, xdetectado, ydetectado = 0, 0, 0

        if PosX <= CentroX:


            if PosY  <= CentroX:

                # primero obtener posXDetectado Hipotenusa
                catetoA = CentroX - PosX
                catetoO = CentroY - PosY
                if (catetoA * catetoA) + (catetoO * catetoO) > 0:
                    xdetectado = math.sqrt((catetoA * catetoA) + (catetoO * catetoO))
                else:
                    xdetectado = 0
                if xdetectado > 0:
                    adetectado = self.ArcoCoseno(catetoA / xdetectado)
                else:
                    adetectado = 0

            else:

                catetoA = CentroX - PosX
                catetoO = PosY - CentroY
                if (catetoA * catetoA) + (catetoO * catetoO) > 0:
                    xdetectado = math.sqrt((catetoA * catetoA) + (catetoO * catetoO))
                else:
                    xdetectado = 0
                if xdetectado > 0:
                    adetectado = 360 - (self.ArcoCoseno(catetoA / xdetectado))
                else:
                    adetectado = 0

            ydetectado = PosZ


        if PosX > CentroX:

            if PosY <= CentroX:

                # primero obtener posXDetectado Hipotenusa
                catetoA = PosX - CentroX
                catetoO = CentroY - PosY
                if (catetoA * catetoA) + (catetoO * catetoO) > 0:
                    xdetectado = math.sqrt((catetoA * catetoA) + (catetoO * catetoO))
                else:
                    xdetectado = 0
                if xdetectado > 0:
                    adetectado = 180 -(self.ArcoCoseno(catetoA / xdetectado))
                else:
                    adetectado = 0

            else:

                catetoA = PosX - CentroX
                catetoO = PosY - CentroY
                if (catetoA * catetoA) + (catetoO * catetoO) > 0:
                    xdetectado = math.sqrt((catetoA * catetoA) + (catetoO * catetoO))
                else:
                    xdetectado = 0
                if xdetectado > 0:
                    adetectado = 270 - (90-(self.ArcoCoseno(catetoA / xdetectado)))
                else:
                    adetectado = 0

            ydetectado = PosZ


        # print("PosXDetectado", self.PosXDetectado)
        # print("PosAdetectada", self.PosADetectado)
        return adetectado, xdetectado, ydetectado

    def ObtenerObjetos(self):

        ##salida modificada ObjectsList = [[top, left, bottom, right, mid_v, mid_h, score, predicted_class]]
        # #..El ultimo () 7 es un string que luego se comprueba en
        ## AQUI PONER LA PARTE DE YOLO
        Lista = []
        Objetovacio = [0, 0, 0, 0, 0, 'Nada']
        Lista.append(Objetovacio)

        if len(self.ObjectsList) > 0:
            Lista = self.ObjectsList

        DFOBJETOS = pd.DataFrame()
        contadorObjetos = 0
        NombresObjetos = []
        Objeto = []

        while contadorObjetos < self.NumeroObjetos:  ## si el contador de objetos es menor que la cantidad de objetos requeridos

            if contadorObjetos < len(Lista):  # si el contador de objetos es menor que la longitud de la Lista
                Objeto = Lista[contadorObjetos]



            else:  # si el contador de objetos es mayor a la longitud d la lista ..se crea un nuevo objeto en blanco

                Objeto = [0, 0, 0, 0, 0, 'Nada']

            try:
                # print("OBJETO")
                # print(Objeto)
                # #
                Nombre = Objeto[5]
                NombresObjetos.append(Nombre)

                # print('nombre obj')
                # print(Nombre)

                DFObjeto = pd.DataFrame([Objeto[:5]], columns=['x{}'.format(contadorObjetos),
                                                               'y{}'.format(contadorObjetos),
                                                               'w{}'.format(contadorObjetos),
                                                               'h{}'.format(contadorObjetos),
                                                               'confianza{}'.format(contadorObjetos),
                                                               ])
                DFNombresObjetos = pd.DataFrame([Objeto[5]], columns=['Objeto{}'.format(contadorObjetos)])

                if contadorObjetos == 0:  ## si es el prier objeto que se cuenta se creaa el DFOBJETOS
                    DFOBJETOS = DFObjeto

                else:  ## si no entonces se Agrega
                    DFOBJETOS = pd.concat([DFOBJETOS, DFObjeto], sort=False, axis=1)

            except Exception as e:
                print('Creando dataframe a partir de la lista de objetos', e)

            contadorObjetos += 1
        #  print (DFOBJETOS)

        return DFOBJETOS, DFNombresObjetos

    def DibujarPlanta(self,Fondo,MargenMapa,mata, xplanta, yplanta):

        try:
            #print("DibujandoPlanta ")
            fondo = Fondo

            xplanta=xplanta+MargenMapa
            yplanta =yplanta + MargenMapa

            alto, ancho = mata.shape[:2]
            #  print("alto",alto)
            roi = fondo[ int(yplanta-alto/2) :int(yplanta + alto/2),int( xplanta-ancho/2) : int(xplanta + ancho/2)]  # la hubicacion de la planta
            # print("ancho ",ancho)

            l_h = 40
            l_s = 0
            l_v = 60

            u_h = 180
            u_s = 255
            u_v = 255

            lower_red = np.array([l_h, l_s, l_v])
            upper_red = np.array([u_h, u_s, u_v])

            # Detectando las formas
            mata = cv2.GaussianBlur(mata, (5, 5), 0)
            hsv = cv2.cvtColor(mata, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, lower_red, upper_red)
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.erode(mask, kernel)

            # contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            mask_inv = cv2.bitwise_not(mask)

            # Take only}} region of logo from logo image. dejar solo el color ..
            img2_fg = cv2.bitwise_and(mata, mata, mask=mask)

            # Now black-out the area of logo in ROI
            img1_bg = cv2.bitwise_and(roi, roi, mask=mask_inv)

            # Put logo in ROI and modify the main image
            dst = cv2.add(img1_bg, img2_fg)
            fondo[ int(yplanta-alto/2) :int(yplanta + alto/2),int( xplanta-ancho/2) : int(xplanta + ancho/2)] = dst # la hubicacion de la planta = dst

            return fondo


        except:
            # error
            #  print("Error al Dibujar")
            return Fondo



    def Imagen(self):

        alto1, ancho1 = self.imagencamara.shape[:2]

        img = cv2.resize(self.imagencamara,(800,int(800*(alto1/ancho1))),cv2.INTER_CUBIC)
        return img


    def ImagenMapa(self):

        self.Mapa = self.Mappeador.ImagenMapa()

        alto1, ancho1 = self.Mapa.shape[:2]

        img = cv2.resize(self.Mapa, (500, int(500 * (alto1 / ancho1))), cv2.INTER_CUBIC)

        return img

    def MapaSimulacion(self):
        # img = cv2.resize(self.simulacionmapa, (1000, 500))
        img = cv2.resize(self.simulacionmapa, None, fx=0.04, fy=0.04, interpolation=cv2.INTER_CUBIC)
        return img


    def Porcentajes(self):

        return self.PorcentajeMapa,self.PorcentajePlantas,self.PorcentajeHierbas

    def ObjetivosRandom(self, Bool):
        self.objetivosrandom = Bool
    def actuadores(self):
        return self.EMA,self.EMX,self.EMY,self.EMADetectado,self.EMXDetectado,self.EMYDetectado

    def HierbasRandom(self, Bool):
        self.hierbasrandom = Bool

    def PlantasRandom(self, Bool):
        self.plantasrandom = Bool
        self.ContadorPlantas = 0

    def evaluando(self,Bool):
        self.Evaluar = Bool

    def Observacion(self):  # devuelve dataframe con los sensores Y OOBBJJEETTOOSS
        return self.Observaciones

    def PosicionReal(self):
        return (self.PosA, self.PosX, self.PosY)

    def velocidades (self):
        return self.VelocidadA,self.VelocidadX,self.VelocidadY

    def PosicionDetectada(self):## en terminos de Cuadricula para la interfase
        return  self.AnguloDetectado,self.RadioDetectado, self.PosXDetectado, self.PosYDetectado, self.PosZDetectado

    def PuntosMapa(self):
        puntos = self.Mappeador.Puntos()
       # print("Puntos",puntos)
        return puntos


    def PuntoCercano(self):
        return self.PMx,self.PMy,self.PMd

    def area(self):
        return (self.XMinima, self.XMaxima, self.YMinima, self.YMaxima)

    def porcentaje(self):
        return self.PorcentajeHierbas

    def error(self):
        return self.Error

    def pasosmaximos(self):
        return self.PasosMaximos

    def Accion(self):  # devuelve dataframe con los sensores Y OOBBJJEETTOOSS
        return self.Acciones

    def Recompensa(self):
        return self._get_reward()

    def Objetivos(self):
        Objetivos = self.ObjetivoDeseado * self.ObjetivoMaximo
        return Objetivos

    def plantas(self):
        return self.Plantas

    def hierbas(self):
        return self.Hierbas

    def figuras(self):
        return [[self.F1x, self.F1y, self.F1a,self.F1d], [self.F2x, self.F2y, self.F2a,self.F2d]]

    def PrepararEvaluacion(self, n_evaluaciones=10):

        self.ContadorEvaluaciones = 0
        self.Hierbas = self.HierbasPrueba
        self.Plantas = self.PlantasPrueba
        self.Evaluar = True

    def PrepararEjecucion(self,XMinima=10,XMaxima=150,YMinima=10,YMaxima=50,reiniciar_mapeo=False):

        self.XMinima=XMinima
        self.XMaxima=XMaxima
        self.YMinima=YMinima
        self.YMaxima=YMaxima


        if reiniciar_mapeo :
            self.Mappeador.LimitarMapa(int(self.XMinima), int(self.XMaxima), int(self.YMinima), int(self.YMaxima))
        self.reset()
    def LongitudArco(self, radio, angulo):
        return 2 * math.pi * radio * (angulo / 360)

    def LongitudAngulo(self, radio, arco):
        ## evitar division por cero
        Angulo = 0
        if radio > 0:
            Angulo = (360 * arco) / (2 * math.pi * radio)

        return Angulo

    def CambiarObjetivos(self, ArrayObjetivos):  # devuelve dataframe con los sensores Y OOBBJJEETTOOSS
        self.ObjetivoDeseado = ArrayObjetivos

    def CambiarPlantas(self, ArrayPlantas):  # devuelve dataframe con los sensores Y OOBBJJEETTOOSS
        self.Plantas = ArrayPlantas

    def CambiarHierbas(self, ArrayHierbas):  # devuelve dataframe con los sensores Y OOBBJJEETTOOSS
        self.Hierbas = ArrayHierbas

    def Renderizar(self, Bool):
        self.Renderizando = Bool

    def _get_action(self):

        return   np.array([self.EMX, self.EMA])


    def _get_obs(self):

        try:

            """       
            Helper to create the observation.
            :return: (OrderedDict<int or ndarray>)
            """

            self.ObjetivoAlcanzado = np.array(
                [self.AnguloDetectado,self.RadioDetectado,self.PosXDetectado, self.PosYDetectado, self.PosZDetectado, self.Error])  # el valor Actual de los Objetivos


            # Normalizar el valor Actual de los Objetivos
            self.ObjetivoAlcanzado = np.divide(self.ObjetivoAlcanzado, self.ObjetivoMaximo)

            self.ObservacionSimple = np.array(

                [self.AnguloRequerido, self.RadioRequerido, self.XRequerido, self.YRequerido, self.ZRequerido,
                 self.AnguloDetectado, self.RadioDetectado, self.PosXDetectado, self.PosYDetectado, self.PosZDetectado,
                 self.Error,
                 # self.F1x, self.F1y, self.F1a,self.F1d,
                 # self.F2x, self.F2y, self.F2a,self.F2d,
                 # self.FMx, self.FMy,self.FMd,
                 # self.PMx, self.PMy , self.PMd,
                 # self.Tx,self.Ty,  ## Puntos que seguimiento para aumentar la informacion sensada
                 self.hora, self.minuto, self.segundo, self.milisegundo,  ## para estandarizar tiempo
                 self.XMinima, self.XMaxima, self.YMinima, self.YMaxima,
                 ##Los cuatro puntos delimitadores de la zona  cuadrada en la que se esta trabajando
                 # self.PorcentajeMapa, self.PorcentajePlantas, self.PorcentajeHierbas,   ## punto de mapeo más cercano
                 self.EMXDetectado, self.EMYDetectado, self.EMADetectado

                 ])
            # Normalizar las observaciones

            ObservacionesNormalizadas = np.divide(self.ObservacionSimple, self.ObservacionesMaximas)

            self.Memoria.append(ObservacionesNormalizadas)
            self.Memoria.popleft()

            n_Observaciones = len(ObservacionesNormalizadas) * self.EspacioMemoria
            # ARRAY DE oBSERVACION

            lista_obs = []
            for linea in list(self.Memoria):
              for valor in linea:        
                lista_obs.append(valor)

            if len(lista_obs) == n_Observaciones:
                self.Observaciones = lista_obs


            Observacion = OrderedDict([

                ('observation', self.Observaciones),
                ('achieved_goal', self.ObjetivoAlcanzado),
                ('desired_goal', self.ObjetivoDeseado)

            ])

           # print("Observacion",Observacion)

            return Observacion

        except Exception as e:
            print("Observando", e)

     #   print(" se tomaron las observaciones")

    def reset(self):
        """
        Important: the observation must be a numpy array
        :return: (np.array)
        """
        obs =None


        SinErrores = False
        while SinErrores == False:



            self.Paso = 0

                    # if self.hierbasrandom:
                    #     self.Hierbas = []
                    #     for i in range(self.CantidadHierbas):
                    #
                    #         posicionenrango=False
                    #
                    #         while posicionenrango==False: ## generar hubicaciones random hasta que se cumplan las condiciones
                    #
                    #             posxhierba = np.random.random() * self.MaxX*2
                    #             posyhierba = np.random.random() * self.MaxX
                    #             areahierba= np.random.random() * self.AreaMaximaHierba
                    #
                    #             a, radio, y = self.CuadroAReloj(posxhierba, posyhierba, 0, self.MaxX, self.MaxX)
                    #
                    #             if radio < self.MaxX and radio > self.MinX:
                    #
                    #                 Hierba = [posxhierba, posyhierba,areahierba ]
                    #                 self.Hierbas.append(Hierba)
                    #                 posicionenrango=True
                    #
            try:



               # print( "reiniciando")
               # print(self.plantasrandom)

                if self.plantasrandom:

                    if  self.ContadorPlantas == 0 or self.ContadorPlantas > len(self.Plantas)-1 :

                       # print("gENERANDO PLANTAS RANDOM")

                        self.ContadorPlantas = 0

                        self.Plantas = []

                        for i in range(self.CantidadPlantas):

                            posicionenrango = False

                            while posicionenrango == False:  ## generar hubicaciones random hasta que se cumplan las condiciones

                                posxplanta = np.random.random() * self.MaxX * 2
                                posyplanta = np.random.random() * self.MaxX *2
                                areaplanta = np.random.random() * self.AreaMaximaPlanta


                                a, radio, y = self.CuadroAReloj(posxplanta, posyplanta, 0, self.MaxX, self.MaxX)

                                if radio < self.MaxX and radio > self.MinX:

                                    Planta = [posxplanta, posyplanta, areaplanta]
                                    self.Plantas.append(Planta)
                                    posicionenrango = True
                                    # El contador de plantas hace que se vaya alternando enrtre cada planta para limpiar su alrededor

                    numeroplanta = int(np.random.random() * len(self.Plantas))
                    planta = self.Plantas[numeroplanta]
                    xplanta, yplanta = planta[0], planta[1]
                    margenlimite = 20

                    if xplanta - margenlimite > 0:  # self.MaA
                        self.XMinima = xplanta - margenlimite
                    else:
                        self.XMinima = 0

                    if xplanta + margenlimite < self.MaxX * 2:  # self.MaA
                        self.XMaxima = xplanta + margenlimite
                    else:
                        self.XMaxima = self.MaxX * 2

                    if yplanta - margenlimite > 0:  # self.Max
                        self.YMinima = yplanta - margenlimite
                    else:
                        self.YMinima = 0

                    if yplanta + margenlimite < self.MaxX * 2:  # self.Max
                        self.YMaxima = yplanta + margenlimite
                    else:
                        self.YMaxima = self.MaxX * 2

                    self.Mappeador.LimitarMapa(int(self.XMinima), int(self.XMaxima), int(self.YMinima),
                                               int(self.YMaxima))

                    self.ContadorPlantas += 1


                else:

                     self.Plantas =self.PlantasPrueba          ## si no se quiere que las plantas sean random

                if self.Evaluar:

                    self.Plantas = self.PlantasPrueba
                    if self.ContadorEvaluaciones < len(self.Plantas):

                        planta = self.Plantas[self.ContadorEvaluaciones]
                        xplanta,yplanta= planta[0],planta[1]

                        margenlimite = 20

                        if xplanta -margenlimite > 0:  # self.MaA
                            self.XMinima = xplanta - margenlimite
                        else:
                            self.XMinima = 0

                        if xplanta + margenlimite < self.MaxX * 2:  # self.MaA
                            self.XMaxima = xplanta + margenlimite
                        else:
                            self.XMaxima = self.MaxX * 2

                        if yplanta - margenlimite > 0:  # self.Max
                            self.YMinima = yplanta - margenlimite
                        else:
                            self.YMinima = 0

                        if yplanta + margenlimite < self.MaxX*2:  # self.Max
                            self.YMaxima = yplanta + margenlimite
                        else:
                            self.YMaxima = self.MaxX*2

                        #if self.ModoVirtual:
                        self.Mappeador.LimitarMapa(int(self.XMinima),int( self.XMaxima), int(self.YMinima),int( self.YMaxima))

                        self.ContadorEvaluaciones += 1

                    else:
                            self.ContadorEvaluaciones = 0
                            self.Plantas = self.PlantasPrueba
                            # self.Hierba = self.HierbasPrueba

                #print("XM",self.XMaxima)
                #print("Xm",self.XMinima)
                self.XRequerido = ((self.XMaxima - self.XMinima)/2 )+self.XMinima
                #print("XR",self.XRequerido)
                self.YRequerido = ((self.YMaxima - self.YMinima)/2) +self.YMinima
                #print("YR", self.YRequerido)
                self.AnguloRequerido, self.RadioRequerido, self.ZRequerido = self.CuadroAReloj(self.XRequerido, self.YRequerido, 0,
                                                                                 self.MaxX, self.MaxX)

                self.ObjetivoDeseado = np.array(
                    [self.AnguloRequerido,self.RadioRequerido,self.XRequerido, self.YRequerido, self.ZRequerido, self.E])
                self.ObjetivoDeseado = np.divide(self.ObjetivoDeseado, self.ObjetivoMaximo)

                # se reestablec el ambiente de simulacion

                if self.reestablecerposiciones:

                    self.PosX = self.MinX  # cm
                    self.PosY = 0  # cmc

                    # self.LimX=0
                    # self.LimY=0
                    self.PosA = 0  # angul
                    self.Error = 0  # %Error

                    self.F1x = 0  # cm
                    self.F1y = 0  # cm
                    self.F1a = 0  # cm

                    self.F2x = 0  # cm
                    self.F2y = 0  # cm
                    self.F2a = 0  # cm

                # self.hora = 0
                # self.minuto = 0
                # self.segundo = 0
                # self.milisegundo = 0

                if self.ModoVirtual:

                    self.GenerarMapa()

                obs = self._get_obs()


                if obs !=  None   :

                    SinErrores=True

            except Exception as e :
                print(" Reseteando ", e)


        return obs


    def CalcularPorcentajePlantas(self):
        ## Si El porcenteje del mapa es 100 se pueden calculkar el porcentage de plantas
        # Verificar El porcentaje de las plantas y hierbas

        if self.PorcentajeMapa == 100:

            # Se toma la imagen del mapa y se pasa por el detector de Plantas para saber cantaas hay
            ##Debe de cortarse primerotodo lo blanco
            contours = self.Detector.Detectar(self.Mapa)
            _, F1x, F1y, F1a, F1d, self.PlantasDetectadas = self.Figura_1.Detectar(self.Mapa, contours)
            _, F2x, F2y, F2a, F2d, self.Hierbasdetectadas = self.Figura_2.Detectar(self.Mapa, contours)
            ## una lista con las plantas detectadas,, Son las hubicaciones con respecto a
            # las dimensiones del mapa creado

            if self.ConteoInicial:  ## Se hace un conteo inicial para Extraer el numero total de plantas
                self.NumeroDePlantasTotal = len(self.PlantasDetectadas)
                self.NumeroDeHierbasTotal = len(self.HierbasDetectadas)

                self.ConteoInicial = False  ## Se ejecuta solo una vez

            else:

                self.NumeroDePlantas = len(self.PlantasDetectadas)
                self.NumeroDeHierbas = len(self.HierbasDetectadas)

                if self.NumeroDePlantasTotal > 0:
                    self.PorcentajePlantas = self.NumeroDePlantas / self.NumeroDePlantasTotal * 100
                else:
                    self.PorcentajePlantas = 0

                if self.NumeroDeHierbasTotal > 0:
                    self.PorcentajeHierbas = self.NumeroDeHierbas / self.NumeroDeHierbasTotal * 100
                else:
                    self.PorcentajeHierbas = 0

        else:
            self.ConteoInicial = True
            self.PorcentajePlantas = 0
            self.PorcentajeHierbas = 0

    def ObtenerImagen (self):
        camara = np.ones(
            (int(self.altocamara), int(self.anchocamara), 3),
            np.uint8)  # * 255

        try:

            if self.ModoVirtual == True:

                camara = self.SimularCamara()
            else:
                camara = self.ObtenerFrame()


        except Exception as e:
            print("Leyendo Imagen", e)

        return camara

    def DetectarError (self):

        # si el robot inenta moverse fuera del area de accion se va aumentando el error cada Segundo .

        if self.XMinima < self.PosXDetectado < self.XMaxima and self.YMinima < self.PosYDetectado < self.YMaxima:
            self.Error -= 0.5
        else:
            self.Error += 0.1

        if self.Error < 0:
            self.Error = 0
        # Limitar el Error al error Maxio
        if self.Error > self.ErrorMaximo:
            self.Error = self.ErrorMaximo




    def Regar (self,VOL):

       self.EjecutarAccion(EMA=0,EBE=0,EMX=0,EMY=0,VOL=VOL,Reset=0)
       self.RecibirDatos()

       while self.VOLDetectado > 0:
           print("regando")
           self.RecibirDatos()
           time.sleep(0.001)


    def paso(self, action):



        #try:

            if self.DiagnosticarParametros:
                self.DiagnosticoParametros()

            if self.ModoVirtual == True:

                ## se cuenta el tiempo con un tiempo de paso especifico
                self.TiempoAnterior = self.TiempoActual
                self.TiempoActual = self.TiempoActual + datetime.timedelta(milliseconds=self.TiempoPaso)
                self.hora = self.TiempoActual.hour
                self.minuto = self.TiempoActual.minute
                self.segundo = self.TiempoActual.second
                self.milisegundo = int(self.TiempoActual.microsecond / 1000)
                # print("s", self.segundo)

                ## Modificar el tiempo de espera para que cambie constantemente
                self.SimularMovimiento(EMX=self.EMX, EMY=self.EMY, EBE=self.EBE, EMA=self.EMA)



                if self.Renderizando:
                    time.sleep(self.TiempoPaso / 1000)



            else:

                ## Hora real

                self.hora = self.TiempoActual.hour
                self.minuto = self.TiempoActual.minute
                self.segundo = self.TiempoActual.second
                self.milisegundo = int(self.TiempoActual.microsecond / 1000)

                self.EjecutarAccion(EMX=self.EMX, EMY=self.EMY, EBE=self.EBE,  EMA=self.EMA,VOL=0, Reset=0)
                self.RecibirDatos()
                diferenciaenms =0

                while 0 <= diferenciaenms < self.TiempoPaso:
                    self.TiempoActual = datetime.datetime.now()

                    diferenciaenms = int(self.TiempoActual.second * 1000 + self.TiempoActual.microsecond / 1000) - int(
                        self.TiempoPasoAnterior.second * 1000 + self.TiempoPasoAnterior.microsecond / 1000)

                    time.sleep(0.001)
                self.TiempoPasoAnterior = self.TiempoActual


            self.Paso += 1
           # print(self.Paso)

            self.EMX = action[0]
            self.EMA= action[1]
            # self.EMY= action[2]
            # self.EBE = action[3]


            if self.SinImagen :

                self.ProcesarSensores()

            else:
                self.imagencamara= self.ObtenerImagen()
                self.imagencamara=self.ProcesarImagen(self.imagencamara)


            self.DetectarError()

           # self.CalcularPorcentajePlantas()




        # except Exception as e:
        #     print("Paso,", e)


            obs = self._get_obs()

            return obs


    def step(self, action):  ## accion es un numpy array de n salidas de largo
        ## lo de los pasos

        observacion, reward, done, info = None,None,None,None

        SinErrores = False
        while SinErrores == False:

            try :
                observacion = self.paso(action)

                reward = self._get_reward()

                # print(reward)
                done = False

                ## revisar si termino la ejecucion de un batch

                if reward > self.RangoRecompensaFinal or self.Paso > self.PasosMaximos:
                    done = True
                # Optionally we can pass additional info, we are not using that for now
                info = {}

                if observacion !=  None  and reward !=  None   and  done !=  None   and  info !=  None :

                    SinErrores=True


            except Exception as e:
                print("PasoErroneo intenTANDO DE NUEVO,", e)


        return observacion, reward, done, info

#
    # def compute_reward(self, achieved_goal: np.ndarray, desired_goal: np.ndarray, info: dict, p: float = 0.5) -> float:
    #
    #
    #     """
    #         Proximity to the goal is rewarded
    #         We use a weighted p-norm
    #     :param achieved_goal: the goal that was achieved
    #     :param desired_goal: the goal that was desired
    #     :param dict info: any supplementary information
    #     :param p: the Lp^p norm used in the reward. Use p<1 to have high kurtosis for rewards in [0, 1]
    #    :return: the corresponding reward
    #     """
    #     achieved_goal = np.divide(achieved_goal, self.ObjetivoMaximo)
    #     # print("achieved_goal",achieved_goal)
    #     desired_goal = np.divide(desired_goal, self.ObjetivoMaximo)
    #     # print("desired_goal", desired_goal)
    #     ## visualizando el calculo de la recompensa
    #     recompensas = np.abs(achieved_goal - desired_goal)
    #
    #     # print(" abs",recompensas)
    #     recompensas = -np.dot(self.PesosoObjetivos, recompensas)
    #     #  print("dot",recompensas)
    #
    #     return recompensas
    #
    # def _get_reward(self):
    #
    #     obs = self._get_obs()
    #     # reward = self.compute_reward(self.ObjetivoAlcanzado, self.ObjetivoDeseado, {})
    #     goalreward = self.compute_reward(obs['achieved_goal'], obs['desired_goal'], {})
    #
    #     return goalreward
    #

    def compute_reward(self, achieved_goal: np.ndarray, desired_goal: np.ndarray, info: dict, p: float = 0.5) -> float:
        """
        Proximity to the goal is rewarded
        We use a weighted p-norm
        :param achieved_goal: the goal that was achieved
        :param desired_goal: the goal that was desired
        :param dict info: any supplementary information
        :param p: the Lp^p norm used in the reward. Use p<1 to have high kurtosis for rewards in [0, 1]
        :return: the corresponding reward
        """
        # print("achieved_goal", achieved_goal)
        # print("desired_goal", desired_goal)
        # print("Pesos objetivos ",self.PesosoObjetivos)

        return -np.power(np.dot(np.abs(achieved_goal - desired_goal), self.PesosoObjetivos), p)

    def _get_reward(self) -> float:
        obs = self._get_obs()
        obs = obs if isinstance(obs, tuple) else (obs,)
        return sum(self.compute_reward(agent_obs['achieved_goal'], agent_obs['desired_goal'], {})
                     for agent_obs in obs)


    def render(self, mode='console'):
        pass

    def close(self):
        pass
