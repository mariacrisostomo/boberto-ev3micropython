#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, ColorSensor
from pybricks.parameters import Port, Stop, Direction
from pybricks.tools import wait
from pybricks.robotics import DriveBase
from pybricks.iodevices import LUMPDevice

# inicializa o brick
robo_brick = EV3Brick()

# inicializa o sensor ultrassonico da lego
sensor_ultrassonico = UltrasonicSensor(Port.S2)

# inicializa o sensor da direita e da esquerda da lego
cor_direita = ColorSensor(Port.S4)
cor_esquerda = ColorSensor(Port.S3)

# inicializa os motores da roda
motor_d = Motor(Port.D, Direction.CLOCKWISE) #horario
motor_e = Motor(Port.A, Direction.COUNTERCLOCKWISE) #antihorario

#motor da garra
#motor_garra = Motor(Port.B, Direction.COUNTERCLOCKWISE)

# sensor do vendra
sensor_vendra = LUMPDevice(Port.S1)

# diametro da roda em mm
diametro_roda = 44

# distancia entre as rodas em mm
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
def CorEsquerdaVendra():
    return sensor_vendra.read(0)[1]

# cor do sensor da esquerda extrema
def CorEsquerdaEXvendra():
    return sensor_vendra.read(0)[0]

# cor do sensor da direita extrema
def CorDireitaEXvendra():
    return sensor_vendra.read(0)[3]

# cor do sensor da direita do meio
def CorDireitaVendra():
    return sensor_vendra.read(0)[2]

# sensores da lego
# cor do sensor da esquerda
def CorEsquerdaLego():
    return cor_esquerda.rgb()

# cor do sensor da direita
def CorDireitaLego():
    return cor_direita.rgb()

# quantos % a - ou a + p ser preto ou branco vendra
# calibrar aqui
branco_vendra_esquerda = 65
preto_vendra_esquerda = 12

branco_vendra_direita = 65
preto_vendra_direita = 12

# pra calcular o erro na função seguirLinha
limite_direita = (branco_vendra_direita + preto_vendra_direita)
limite_esquerda = (branco_vendra_esquerda + preto_vendra_esquerda)

# o quanto deve virar p voltar
kp = 1

# o quao rapido p n desviar
kd = 0.5

erro_anterior = 0

# velocidade inicial da base (graus/s)
velocidade_base = 200

def seguirLinha():
    global kp
    global kd
    global erro_anterior
    global motor_d
    global motor_e
    global cor_direita
    global cor_esquerda
    
    # calcula o erro
    erro = CorEsquerdaVendra() - CorDireitaVendra()

    # calcula o ajuste
    ajuste = kp * erro + kd * (erro - erro_anterior)

    # aplica na velocidade os ajustes
    velocidade_motor_esquerdo = velocidade_base + ajuste
    velocidade_motor_direito = velocidade_base - ajuste

    # (re)define a velocidade dos motores (graus/s)
    motor_d.run(velocidade_motor_direito)
    motor_e.run(velocidade_motor_esquerdo)

    # armazena o erro atual para a proxima iteração
    erro_anterior = erro

    # espera um pouquinho pra nao sobrecarregar
    wait(10)

while True:
    seguirLinha()
    


    