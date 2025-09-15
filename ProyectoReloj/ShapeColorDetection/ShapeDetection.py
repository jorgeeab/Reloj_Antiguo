import cv2
import numpy as np

class Figura:

    def __init__(self,nombre,Figura,area_minima,area_maxima):

        self.nombre = nombre
        self.Figura = Figura  ## unstring que determina la figura que se quiere detectar

        self.area_minima = area_minima
        self.area_maxima = area_maxima


        #Triangle
        #Rectangle
        #Circle

        self.font = cv2.FONT_HERSHEY_DUPLEX


    def Detectar(self,imagen,contours):

            try:

                FigurasDetectadas = []

                height, width, channels = imagen.shape
                rojo = (0, 0, 255)
                verde = (0, 255, 0)
                azul = (255, 0, 0)
                blanco = (255, 255, 255)
                amarillo = (0, 255, 255)
                negro = (0,0,0)

                X = 0
                Y= 0
                ##(en un principio voy a utilizar las coordenadas del primer punto del poligono detectado pero despues hay quhacerlo del centro )
                Area = 0 ## se utiliza el area para que el bicho pueda detectar la distancia
                Distancia= 0 # la distancias desde el centro de la imagen (   )

                Figura = "Nada"
                mayorarea=0

                for i ,cnt in enumerate(contours):

                    area = cv2.contourArea(cnt)
                    #print(area)
                    approx = cv2.approxPolyDP(cnt, 0.001 * cv2.arcLength(cnt, True), True)
                   # print("ravel",approx.ravel())
                   #  x = approx.ravel()[0]
                   #  y = approx.ravel()[1]

                    # compute the center of the contour
                    M = cv2.moments(cnt)
                    x = int(M["m10"] / M["m00"])
                    y = int(M["m01"] / M["m00"])


                    if  self.area_minima < area and area < self.area_maxima :

                        #cv2.drawContours(imagen, [approx], 0, amarillo, 1)
                        if len(approx) == 3:
                            Figura="Triangle"

                            #cv2.putText(imagen,Figura , (x, y), self.font, 2, verde)

                        elif len(approx) == 4:
                            Figura="Rectangle"

#                           cv2.putText(imagen, "Rectangle", (x, y), self.font, 1, verde)

                        elif 8 < len(approx) <200:
                            Figura = "Circle"
                            cv2.putText(imagen, "plant", (x, y), self.font, 1, verde)



                        if Figura == self.Figura and x*y != 0 :

                            dist = cv2.pointPolygonTest(cnt, (width/2, height/2), True)

                            figura = [x, y, area, dist]
                            FigurasDetectadas.append(figura)

                            if area > mayorarea:

                                mayorarea = area
                                X = x
                                Y = y
                                Area = mayorarea
                                Distancia = dist


                # cv2.putText(imagen, "{}".format(self.nombre) , (X, Y), self.font, 2,azul)
                # cv2.putText(imagen, "x{},y{}".format(X,Y), (X, Y),self.font, 1, negro)
               #cv2.putText(imagen, "d{}".format(Distancia) , (X, Y+105), self.font, 1, negro)

                        ## poe el momento solo puede haber un solo poligono igual de un solo color en
                        ##la pantalla

                ## NO devolver un pandas dataframe por que es mejor enumerar los nombres de las figuras

                return imagen,X,Y,Area,Distancia,FigurasDetectadas

            except Exception as e:
                print("Detectando Geometrias " ,e)


class DetectordeContornos:

    def __init__(self,l_h,u_h,l_s,u_s,l_v,u_v):

        self.lower_red = np.array([l_h, l_s, l_v])
        self.upper_red = np.array([u_h, u_s, u_v])

    def Detectar(self,imagen):

            try:

                # Detectando las formas
                hsv = cv2.cvtColor(imagen, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv, self.lower_red, self.upper_red)
                kernel = np.ones((15, 15), np.uint8)
                mask = cv2.erode(mask, kernel)
                kernel = np.ones((25, 25), np.uint8)
                mask = cv2.dilate(mask, kernel)


                ## filtro de contornos internos
                Contours = []
                contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                for i in range(len(contours)):
                    if hierarchy[0, i, 3] == -1:
                        Contours.append( contours[i])

                return Contours
            except Exception as e :
                print("DetectandoContornos ",e)
