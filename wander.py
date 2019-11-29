# -*- coding: utf-8 -*-
##    Client of V-REP simulation server (remoteApi)
##    Copyright (C) 2015  Rafael Alceste Berri rafaelberri@gmail.com
##
##    This program is free software: you can redistribute it and/or modify
##    it under the terms of the GNU General Public License as published by
##    the Free Software Foundation, either version 3 of the License, or
##    (at your option) any later version.
##
##    This program is distributed in the hope that it will be useful,
##    but WITHOUT ANY WARRANTY; without even the implied warranty of
##    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
##    GNU General Public License for more details.
##
##    You should have received a copy of the GNU General Public License
##    along with this program.  If not, see <http://www.gnu.org/licenses/>.
##
##
##Habilite o server antes na simulação V-REP com o comando lua:
##simExtRemoteApiStart(portNumber) -- inicia servidor remoteAPI do V-REP


import vrep

import numpy as np
import math
import skfuzzy as fuzz
from skfuzzy import control as ctrl

# Cria as variáveis do problema
distanciaEsq = ctrl.Antecedent(np.arange(0, 25, 0.01), 'distanciaEsq')
distanciaDir = ctrl.Antecedent(np.arange(0, 25, 0.01), 'distanciaDir')
distanciaFre = ctrl.Antecedent(np.arange(0, 25, 0.01), 'distanciaFre')
velocidadeEsq = ctrl.Consequent(np.arange(0, 10, 0.5), 'velocidadeEsq')
velocidadeDir = ctrl.Consequent(np.arange(0, 10, 0.5), 'velocidadeDir')

# Cria as funções de pertinência usando tipos variados
distanciaEsq['perto'] = fuzz.trapmf(distanciaEsq.universe, [0, 0, 0.5, 2])
distanciaEsq['media'] = fuzz.trimf(distanciaEsq.universe, [0.5, 2, 10])
distanciaEsq['longe'] = fuzz.trapmf(distanciaEsq.universe, [2, 10, 25, 25])

distanciaDir['perto'] = fuzz.trapmf(distanciaDir.universe, [0, 0, 0.5, 2])
distanciaDir['media'] = fuzz.trimf(distanciaDir.universe, [0.5, 2, 10])
distanciaDir['longe'] = fuzz.trapmf(distanciaDir.universe, [2, 10, 25, 25])

distanciaFre['perto'] = fuzz.trapmf(distanciaFre.universe, [0, 0, 0.5, 2])
distanciaFre['media'] = fuzz.trimf(distanciaFre.universe, [0.5, 2, 10])
distanciaFre['longe'] = fuzz.trapmf(distanciaFre.universe, [2, 10, 25, 25])

velocidadeEsq['baixa'] = fuzz.trimf(velocidadeEsq.universe, [0, 0, 0.5])
velocidadeEsq['media'] = fuzz.trimf(velocidadeEsq.universe, [0.5, 5, 8])
velocidadeEsq['alta'] = fuzz.trapmf(velocidadeEsq.universe, [5, 8, 10, 10])

velocidadeDir['baixa'] = fuzz.trimf(velocidadeDir.universe, [0, 0, 0.5])
velocidadeDir['media'] = fuzz.trimf(velocidadeDir.universe, [0.5, 5, 8])
velocidadeDir['alta'] = fuzz.trapmf(velocidadeDir.universe, [5, 8, 10, 10])

"""### Criando as regras de decisão difusas"""

rule1 = ctrl.Rule(distanciaEsq['longe'], velocidadeEsq['alta'])
rule2 = ctrl.Rule(distanciaDir['longe'], velocidadeDir['alta'])
rule3 = ctrl.Rule(distanciaEsq['media'], velocidadeEsq['media'])
rule4 = ctrl.Rule(distanciaDir['media'], velocidadeDir['media'])
rule5 = ctrl.Rule(distanciaEsq['perto'], velocidadeEsq['baixa'])
rule6 = ctrl.Rule(distanciaDir['perto'], velocidadeDir['baixa'])

"""### Criando e simulando um controlador nebuloso"""

velocidade_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6])
velocidade_simulador = ctrl.ControlSystemSimulation(velocidade_ctrl)

#definicoes iniciais
serverIP = '127.0.0.1'
serverPort = 19999
leftMotorHandle = 0
vLeft = 0.
rightMotorHandle = 0
vRight = 0.
sensorHandle = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]


# variaveis de cena e movimentação do pioneer
noDetectionDist=0.5
maxDetectionDist=0.2
detect=[0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
braitenbergL=[-0.2,-0.4,-0.6,-0.8,-1,-1.2,-1.4,-1.6, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
braitenbergR=[-1.6,-1.4,-1.2,-1,-0.8,-0.6,-0.4,-0.2, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
v0=2

clientID = vrep.simxStart(serverIP,serverPort,True,True,2000,5)
if clientID <> -1:
    print ('Servidor conectado!')

    # inicialização dos motores
    erro, leftMotorHandle = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_oneshot_wait)
    if erro <> 0:
        print 'Handle do motor esquerdo nao encontrado!'
    else:
        print 'Conectado ao motor esquerdo!'

    erro, rightMotorHandle = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_oneshot_wait)
    if erro <> 0:
        print 'Handle do motor direito nao encontrado!'
    else:
        print 'Conectado ao motor direito!'

    #inicialização dos sensores (remoteApi)
    for i in range(16):
        erro, sensorHandle[i] = vrep.simxGetObjectHandle(clientID,"Pioneer_p3dx_ultrasonicSensor%d" % (i+1),vrep.simx_opmode_oneshot_wait)
        if erro <> 0:
            print "Handle do sensor Pioneer_p3dx_ultrasonicSensor%d nao encontrado!" % (i+1)
        else:
            print "Conectado ao sensor Pioneer_p3dx_ultrasonicSensor%d!" % (i+1)
            erro, state, coord, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(clientID, sensorHandle[i],vrep.simx_opmode_streaming)

    #desvio e velocidade do robo
    while vrep.simxGetConnectionId(clientID) != -1:
        for i in range(5):
            erro, state, coord, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(clientID, sensorHandle[i],vrep.simx_opmode_buffer)
            if erro == 0:
                dist[i] = math.sqrt(coord[0]**2 + coord[1]**2 + coord[2]**2)
                # if state > 0:
                #     detect[i] = 1-((dist-maxDetectionDist) / (noDetectionDist-maxDetectionDist))
                # else:
                #     detect[i] = 0
            else:
                dist[i] = 0

        distEsq = min(dist[0] dist[1])
        distDir = min(dist[2] dist[3])

        # Entrando com alguns valores para qualidade da distanciaEsq e do serviço
        velocidade_simulador.input['distanciaEsq'] = distEsq
        velocidade_simulador.input['distanciaDir'] = distDir

        # Computando o resultado
        velocidade_simulador.compute()        

        vLeft = velocidade_simulador.output['velocidadeEsq']
        vRight = velocidade_simulador.output['velocidadeDir']

        # atualiza velocidades dos motores
        erro = vrep.simxSetJointTargetVelocity(clientID, leftMotorHandle, vLeft, vrep.simx_opmode_streaming)
        erro = vrep.simxSetJointTargetVelocity(clientID, rightMotorHandle, vRight, vrep.simx_opmode_streaming)

    vrep.simxFinish(clientID) # fechando conexao com o servidor
    print 'Conexao fechada!'
else:
    print 'Problemas para conectar o servidor!'
