


def GenerarMapa():
    try:


        # print("Generando Terreno Virtual")
        fondo = self.fondo
        # alto1, ancho1 = fondo.shape[:2]
        # (pixeles),(pixeles)
        # lista de plantas de diversas especies
        # para cada planta agregar plantas en el lugar de la planta y ponerlo sobre la imagen simukacion mapa


        contadorimagenes = 0


        for planta in self.Plantas:
            # print("Planta")

            try:

                if contadorimagenes < len(self.ImagenesPlantas):

                    contadorimagenes += 1
                else:
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

        fondo = self.Mappeador.DibujarMarcas(fondo, self.MargenSimulacionMapa)

        self.simulacionmapa = fondo

    except Exception as e:
        print("GENERANDO MAPA ", e)