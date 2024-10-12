#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, ColorSensor
from pybricks.parameters import Port, Stop, Direction
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.iodevices import LUMPDevice

#inicializa o brick
robo_brick = EV3Brick()

#inicializa o sensor ultrassonico da lego
sensor_ultrassonico = UltrasonicSensor(Port.S2)

#inicializa o sensor da direita e da esquerda da lego
cor_direita = ColorSensor(Port.S4)
cor_esquerda = ColorSensor(Port.S3)

#inicializa os motores
motor_d = Motor(Port.D, Direction.CLOCKWISE) #horario
motor_e = Motor(Port.A, Direction.COUNTERCLOCKWISE) #antihorario

#motor da garra
#motor_garra = Motor(Port.B, Direction.COUNTERCLOCKWISE)

#sensor do vendra
sensor_vendra = LUMPDevice(Port.S1)

#diametro da roda em mm
diametro_roda = 44

#distancia entre as rodas em mm
distancia_entre_rodas = 320

#passa as informacoes do robo p a lib
base = DriveBase(
    motorDireito=motor_e,
    motorEsquerdo=motor_d,
    diametroRoda=diametro_roda,
    distanciaEntreAsRodas=distancia_entre_rodas
)

# sensores do vendra
# cor do sensor da esquerda do meio
def VerificarCorVEsquerda():
    return sensor_vendra.read(0)[1]

# cor do sensor da esquerda extrema
def VerificarCorVEsquerdaEX():
    return sensor_vendra.read(0)[0]

# cor do sensor da direita extrema
def VerificarCorVDireitaEX():
    return sensor_vendra.read(0)[3]

# cor do sensor da direita do meio
def VerificarCorVDireita():
    return sensor_vendra.read(0)[2]

# sensores da lego
# cor do sensor da esquerda
def VerificarCorEsquerdaLego():
    return cor_esquerda.rgb()

# cor do sensor da direita
def VerificarCorDoreitaLego():
    return cor_direita.rgb()