# import math

import numpy as np
# import pandas as pd
# from stable_baselines.common.policies import MlpPolicy#, MlpLstmPolicy, MlpLnLstmPolicy
# from stable_baselines.common import make_vec_env
from stable_baselines3 import HER, SAC  # ,PPO2,ACER,SAC

# from stable_baselines3.common.callbacks import StopTrainingOnRewardThreshold
# from stable_baselines3.common.callbacks import CheckpointCallback,EvalCallback,CallbackList
from CallbacksModificado import CheckpointCallback, EvalCallback, CallbackList, evaluate_policy, ProgressBarManager
# from stable_baselines.common.evaluation import evaluate_policy
# from stable_baselines3.common.cmd_util import  DummyVecEnv

# from stable_baselines.gail import generate_expert_traj
# from stable_baselines.gail import ExpertDataset

# from stable_baselines3.common.vec_env import VecEnv
# from stable_baselines import results_plotter
# from stable_baselines.results_plotter import load_results, ts2xy
import os
# import matplotlib.pyplot as plt

# from tqdm import tqdm
# import gym

# import websocket

import cv2
# import time
import threading
# import json
import optuna

# from RelojEnv3Fb import RelojSimHierbaEnv
# from RelojEnvSimple import RelojSimHierbaEnv
from RelojEnvMemoria2 import RelojSimHierbaEnv
# from RelojEnvMemoriaCompleto import RelojSimHierbaEnv
from JoyPad import Joypad


class Protocolo:

    def __init__(self, Nombre='Sin Nombre'):

        self.modelo = None

        self.robot = RelojSimHierbaEnv()

        self.NombreProtocolo = Nombre

        self.CarpetaProtocolo = str.format('AnalizarProtocolos/{}', self.NombreProtocolo)
        print("Carpeta protocolo", self.CarpetaProtocolo)

        self.NuevoProtocolo(True)

        ## establecer las condiciones iniciales del robot
        #

        self.robot.reset()
        print("ROBOT Iniciado")


    def objective(self, trial):

        self.ControlManual = False

        mejor_recompensa = -1000
        try:

            #  HER SAC
            lr = trial.suggest_loguniform("lr", 1e-5, 1e-2)
            gamma = trial.suggest_loguniform("gamma", 0.93, 0.97)

            batch_size = trial.suggest_int("batchsize", 10, 1024)
            # batch_size = trial.suggest_categorical("batch_size", [32 , 64 , 256, 512, 1024])

            # buffer_size = trial.suggest_int("buffer_size", 1e2, 1e4)
            buffer_size = int(1e6)

            # Int parameter
            num_layers = trial.suggest_int('num_layers', 1, 4)

            layers = []
            for layer in range(num_layers):
                n_units = int(trial.suggest_loguniform('n_units_L{}'.format(layer), 4, 1024))
                layers.append(n_units)

            Nombremodelo = "LR{}-G{}-Ly{}-Bat{}-Buf{}".format(round(lr, 5), round(gamma, 5), layers, batch_size,
                                                              buffer_size)

            self.CarpetaProtocolo = str.format('AnalizarProtocolos/{}/ModelosAnalizados/{}', self.NombreProtocolo,
                                               Nombremodelo)

            print(self.CarpetaProtocolo)

            self.NuevoProtocolo(lr=lr, gamma=gamma, batch_size=batch_size, layers=layers, buffer_size=buffer_size)

            mejormodelo = self.modelo

            for step in range(4):

                self.modelo = mejormodelo
                self.Aprender(1e4, evaluar=False)

                # Report intermediate objective value.

                recompensas_evaluaciones = []

                for i in range(2):
                    mean_reward, std_reward = evaluate_policy(self.modelo, self.robot, n_eval_episodes=6)
                    recompensas_evaluaciones.append(mean_reward)

                prom_recompensas = np.mean(recompensas_evaluaciones)
                desviacion_recompensas = np.std(recompensas_evaluaciones)

                intermediate_value = prom_recompensas - desviacion_recompensas

                if step < 1:
                    print("STEP", step)
                    mejor_recompensa = intermediate_value
                else:
                    if intermediate_value > mejor_recompensa:
                        mejor_recompensa = intermediate_value
                        mejormodelo = self.modelo

                print("promedio_recompensas", prom_recompensas)
                print("desviacion_recompensas", desviacion_recompensas)

                print("Recompensa estandar:  ", intermediate_value)
                print("mejor_recompensa", mejor_recompensa)

                trial.report(intermediate_value, step)

                # Handle pruning based on the intermediate value.
                if trial.should_prune():
                    raise optuna.TrialPruned()

            self.GuardarProtocolo(mejormodelo)

        except Exception as e:
            print("Entrenando", e)

        return mejor_recompensa

    def Entrenar(self, Pasos=4, len_Pasos=1e4, guardar=False):

        mejormodelo = self.modelo

        for step in range(Pasos):

            self.modelo = mejormodelo
            self.Aprender(int(len_Pasos), evaluar=False)

            # Report intermediate objective value.

            recompensas_evaluaciones = []

            for i in range(2):
                mean_reward, std_reward = evaluate_policy(self.modelo, self.robot, n_eval_episodes=6)
                recompensas_evaluaciones.append(mean_reward)

            prom_recompensas = np.mean(recompensas_evaluaciones)
            desviacion_recompensas = np.std(recompensas_evaluaciones)

            intermediate_value = prom_recompensas - desviacion_recompensas

            if step < 1:
                print("STEP", step)
                mejor_recompensa = intermediate_value
            else:
                if intermediate_value > mejor_recompensa:
                    mejor_recompensa = intermediate_value
                    mejormodelo = self.modelo
                    if guardar:
                        self.GuardarProtocolo(mejormodelo)

            print("promedio_recompensas", prom_recompensas)
            print("desviacion_recompensas", desviacion_recompensas)

            print("Recompensa estandar:  ", intermediate_value)
            print("mejor_recompensa", mejor_recompensa)

    def Aprender(self, Pasos, evaluar, FrecuenciaEvaluacion=int(1e4)):

        self.robot.ObjetivosRandom(True)
        self.robot.PlantasRandom(True)
        self.robot.HierbasRandom(True)
        self.robot.evaluando(False)

        with ProgressBarManager(
                int(Pasos)) as plot_callback:  # this the garanties that the tqdm progress bar closes correctly

            if evaluar:
                # checkpoint_callback = CheckpointCallback(save_freq=10000, save_path=self.CarpetaProtocolo)

                eval_env = self.robot
                # callback_on_best = StopTrainingOnRewardThreshold(reward_threshold=-50, verbose=1)
                eval_callback = EvalCallback(eval_env,
                                             best_model_save_path='{}/mejor_modelo'.format(self.CarpetaProtocolo),
                                             log_path='{}/resultados'.format(self.CarpetaProtocolo),
                                             eval_freq=FrecuenciaEvaluacion, n_eval_episodes=3)

                # callback = CallbackList([eval_callback,checkpoint_callback,plot_callback])
                self.modelo.learn(total_timesteps=int(Pasos), callback=[eval_callback, plot_callback])  #

            else:
                self.modelo.learn(total_timesteps=int(Pasos), callback=[plot_callback])  #

        # self.GuardarProtocolo()

    def Evaluar(self):

        try:

            mean_reward, std_rewardevaluate_policy = evaluate_policy(self.modelo, self.robot, 20, )
            print("Resultado de la Evaluacion ")
            print("Mean reward: ", mean_reward)
            print("desviacion: ", std_rewardevaluate_policy)

        except Exception as e:
            print("Evaluando ", e)

    def IrA(self, X, Y, intentos=10, pasosLimite=5000):

        self.robot.ObjetivosRandom(False)
        self.robot.PlantasRandom(False)
        self.robot.HierbasRandom(False)

        XMinima = X - 20
        XMaxima = X + 20
        YMinima = Y - 20
        YMaxima = Y + 20

        self.robot.PrepararEjecucion(XMinima, XMaxima, YMinima, YMaxima, reiniciar_mapeo=False)

        episode_rewards, episode_lengths = [], []
        for i in range(intentos):
            obs = self.robot.reset()
            print("obs", obs)

            listo = False
            episode_reward = 0.0
            episode_length = 0

            while not listo:
                action, state = self.modelo.predict(obs)
                # print("Action",action)
                obs, reward, done, _info = self.robot.step(action)
                episode_reward += reward
                episode_length += 1
                if episode_length > pasosLimite or done == True:
                    listo = True
            episode_rewards.append(episode_reward)
            episode_lengths.append(episode_length)

        print("mean recompensa", np.mean(episode_rewards))
        print("std", np.std(episode_rewards))

    def Mappear(self, XMinima, XMaxima, YMinima, YMaxima):

        self.robot.ObjetivosRandom(False)
        self.robot.PlantasRandom(False)
        self.robot.HierbasRandom(False)
        Xcentro = int(((XMaxima - XMinima) / 2) + XMinima)
        Ycentro = int(((YMaxima - YMinima) / 2) + YMinima)

        self.robot.PrepararEjecucion(XMinima, XMaxima, YMinima, YMaxima, reiniciar_mapeo=True)
        self.IrA(Xcentro, Ycentro, intentos=2, pasosLimite=10000)

        PuntosMapeo = self.robot.PuntosMapa()

        for punto in PuntosMapeo:
            self.IrA(X=int(punto[0]), Y=int(punto[1]), intentos=2, pasosLimite=10000)

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

    def GuardarProtocolo(self, Modelo):  # esta funcion guarda este ptotocolo en una carpeta con su nombre

        if not os.path.isdir(self.CarpetaProtocolo):
            os.makedirs(self.CarpetaProtocolo)

        ## Guardar el modelo
        Modelo.save("{}/{}.zip".format(self.CarpetaProtocolo, self.NombreProtocolo))

        print("Modelo Guardado..............................")

        ##LR0.00952-G0.93478-Ly[256, 512, 512, 512]-Bat256-Buf6658

    def NuevoProtocolo(self, importar_existente=True, lr=0.00004, gamma=0.936, batch_size=802, layers=[8],
                       buffer_size=2900):

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


#Protocolo = Protocolo(Nombre="RobotReal18")

# Protocolo.robot.Regar(140)
# Protocolo = Protocolo(Nombre="Completo")


# Protocolo.NuevoProtocolo(importar_existente=False,lr=0.0005,gamma=0.94,batch_size=512,layers=[256,256,256],buffer_size= int(1e6))

# Protocolo.IrA(90,30,5,100000)
# Protocolo.Entrenar(4,10000,True)
# Protocolo.Mappear(30,60,70,110)
# Protocolo.Optimizar(20)
# Protocolo.VisualizarAnalisis()
# Protocolo.Evaluar()

# Protocolo.Aprender(50000,True,1000)

# Protocolo.Probar(n_eval_episodes=10,pasosLimite=10000)


# Accuracy: -173.25499361287396 Best hyperparameters: {'Layer1': 512, 'Layer2': 1024, 'Layer3': 1024, 'Layer4': 512, 'batch_size': 512, 'buffer_size': 3441, 'gamma': 0.9654472877569693, 'lr': 0.0016095161311604506