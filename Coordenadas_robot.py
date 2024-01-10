
#Librerias
import numpy as np
from numpy import linalg
import cmath
import math
from scipy.spatial.transform import Rotation
from math import pi as pi
from math import cos as cos
from math import sin as sin
from math import atan2 as atan2
from math import acos as acos
from math import asin as asin
from math import sqrt as sqrt
#Parametros

pallet = [190,-500,-505]# en mm
pallet_array = np.array(pallet)
ejeY = False
rotx = False
mat=np.matrix

global d1, a2, a3, a7, d4, d5, d6,d, a, alph
d1 =  0.1273
a2 = -0.612
a3 = -0.5723
#a7 = 0.075
d4 =  0.163941
#d5 =  0.08535
d6 =  0.0922
tolerancia = 0.005

#d = mat([0.15185, 0, 0, 0.13105, 0.08535, 0.0921]) #ur3
#d = mat([0.15185, 0, 0, 0.1105, 0.08535, 0.0921]) #ur3
#d = mat([0.089159, 0, 0, 0.10915, 0.09465, 0.0823]) ur5
d = mat([0.1273, 0, 0, 0.163941, 0.1157, 0.0922])#ur10 mm
#a = mat([0 ,-0.24355 ,-0.2132 ,0 ,0 ,0]) #ur3
# a =mat([0 ,-0.425 ,-0.39225 ,0 ,0 ,0]) ur5
a = mat([0 ,-0.612 ,-0.5723 ,0 ,0 ,0])#ur10 mm
#alph = mat([math.pi/2, 0, 0, math.pi/2, -math.pi/2, 0 ])  #ur5
alph = mat([pi/2, 0, 0, pi/2, -pi/2, 0 ]) # ur10


#Listas
Pose=[]
Matrix=[]
TT=[]

# Generacion de partes
Partes = []
Cilindros = []

#Clases
class Plano:

    #Constructor

    def __init__(self, p1, p2, p3, pc, holi):
        if (holi):
            [self.A, self.B, self.C, self.D] = self.CalcularPlano(p1,p2,p3,pc)
        else:
            self.A = p1
            self.B = p2
            self.C = p3
            self.D = pc
    
    # Metodos

    def CalcularPlano(self,p1,p2,p3,pc):
        v1 = [p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]]
        v2 = [p3[0] - p1[0], p3[1] - p1[1], p3[2] - p1[2]]
        normal = [v1[1] * v2[2] - v1[2] * v2[1], v1[2] * v2[0] - v1[0] * v2[2], v1[0] * v2[1] - v1[1] * v2[0]]
        norm = math.sqrt( normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2])
        normal[0] /= norm
        normal[1] /= norm
        normal[2] /= norm
        D = -(normal[0] * p1[0] + normal[1] * p1[1] + normal[2] * p1[2])
        if normal[0] * pc[0] + normal[1] * pc[1] + normal[2] * pc[2] + D < 0:
            return normal[0],normal[1],normal[2],D
        else:
            return -normal[0],-normal[1],-normal[2],-D
    def distancia(self,p):
        return self.A * p[0] + self.B * p[1] + self.C * p[2] + self.D
class CajaRobot:

    # Constructor

    def __init__(self, esq1, esq2, q): # Las esquinas estan en sistema de coordenadas del gripper en metros
        esquinas0 = np.array([[esq1[0] / 100.0, esq1[1] / 100.0, esq1[2] / 100.0,1],
                             [esq1[0] / 100.0, esq2[1] / 100.0, esq1[2] / 100.0, 1],
                             [esq2[0] / 100.0, esq2[1] / 100.0, esq1[2] / 100.0, 1],
                             [esq2[0] / 100.0, esq1[1] / 100.0, esq1[2] / 100.0, 1],
                             [esq1[0] / 100.0, esq1[1] / 100.0, esq2[2] / 100.0, 1],
                             [esq1[0] / 100.0, esq2[1] / 100.0, esq2[2] / 100.0, 1],
                             [esq2[0] / 100.0, esq2[1] / 100.0, esq2[2] / 100.0, 1],
                             [esq2[0] / 100.0, esq1[1] / 100.0, esq2[2] / 100.0, 1]])

        self.esquinas = []
        '''
        print("Cajas originales")
        for e in esquinas0:
            print(e)
        
        print("M0T6")
        print (M0T6)
        esquinasT=np.transpose(np.dot(M0T6, np.transpose(esquinas0)))[:,0:3]
        '''
        
        theta1 = q[0]
        theta2 = q[1]
        theta3 = q[2]
        theta4 = q[3]
        theta5 = q[4]
        theta6 = q[5]

        th = np.matrix([[theta1], [theta2], [theta3], [theta4], [theta5], [theta6]])
        c = [0]
        M0T6 = HTrans(th,c)
        for i in range(8):
            punto = np.array([esquinas0[i,0], esquinas0[i,1], esquinas0[i,2], 1])
            resu = np.dot(M0T6, punto)
            self.esquinas.append([resu[0,0], resu[0,1], resu[0,2]])

        centro = [0,0,0]
        for e in self.esquinas:
            centro[0] += e[0]
            centro[1] += e[1]
            centro[2] += e[2]
        centro[0] /= 8
        centro[1] /= 8
        centro[2] /= 8
        self.planos = [Plano(self.esquinas[4],self.esquinas[0],self.esquinas[1],centro, True),
                       Plano(self.esquinas[5],self.esquinas[1],self.esquinas[2],centro, True),
                       Plano(self.esquinas[7],self.esquinas[2],self.esquinas[3],centro, True),
                       Plano(self.esquinas[4],self.esquinas[3],self.esquinas[0],centro, True),
                       Plano(self.esquinas[4],self.esquinas[5],self.esquinas[7],centro, True),
                       Plano(self.esquinas[0],self.esquinas[1],self.esquinas[2],centro, True)]   
class Parte:

    # Constructor

    def __init__(self, c, tipo):
        if tipo == 'cilindro':
            self.tipo = 'cilindro'
            self.puntos, self.planos = self.generar_planos_tangentes_cilindro(c)
            self.planos.append(Plano(c.vector[0],c.vector[1],c.vector[2], -np.dot(np.array(c.vector),np.array(c.p1)), False))
            self.planos.append(Plano(-c.vector[0],-c.vector[1],-c.vector[2], -np.dot(-np.array(c.vector),np.array(c.p2)), False))
        elif tipo == 'caja':
            self.tipo = 'caja'
            self.puntos = c.esquinas
            self.planos = c.planos


#Funciones
#Matriz de Transformacion
def Matriz__T(thetaa,a,d,alpha):
    T= np.array ([[np.cos(thetaa), -np.cos(alpha)*np.sin(thetaa), np.sin(alpha)*np.sin(thetaa), a*np.cos(thetaa)], 
               [np.sin(thetaa), np.cos(alpha)*np.cos(thetaa), -np.sin(alpha)*np.cos(thetaa), a*np.sin(thetaa)],
               [0, np.sin(alpha), np.cos(alpha), d],
               [0,0,0,1]])
    return T

 #Devuelve la matriz de transformacion 
def pos(joint,q):
    global TT
    #Parametros DH
    #a[m],d[m], alpha[rad]
    DH= np.array ([[0,  0.1273,  math.pi/2], 
                    [-0.612,  0, 0],
                    [-0.5723,  0,  0],
                    [0,  0.163941,  math.pi/2],
                    [0,  0.1157,  -math.pi/2],
                    [0,  0.0922,  0]])
    a=0
    d=0
    alpha=0
    thetas=0
    if(joint == 1):
         a= DH[0][0]
         d= DH[0][1]
         alpha= DH[0][2]
         thetas=q[0]
    elif(joint == 2):
         a= DH[1][0]
         d= DH[1][1]
         alpha= DH[1][2]
         thetas=q[1]
    elif(joint == 3):
         a= DH[2][0]
         d= DH[2][1]
         alpha= DH[2][2]
         thetas=q[2]
    elif(joint == 4):
         a= DH[3][0]
         d= DH[3][1]
         alpha= DH[3][2]
         thetas=q[3]
    elif(joint == 5):
         a= DH[4][0]
         d= DH[4][1]
         alpha= DH[4][2]
         thetas=q[4]
    elif(joint == 6):
         a= DH[5][0]
         d= DH[5][1]
         alpha= DH[5][2]
         thetas=q[5]
    #else:
         #print("joint fuera de rango")
         
    TT= Matriz__T(thetas,a,d,alpha)
    #T_lim = np.around(TT, decimals=4)

# Genera la matriz de Transformacion entre dos puntos, ej 0T6
def  T_entre(num1, num2,q):
    global Pose
    matrices = []# Gra
    
    for i in range(num1, num2 + 1):
        pos(i,q)
        matrices.append(TT) 
        
    #print("matriz",matrices)
    resultado = matrices[0]
    for i in range(1, len(matrices)):
        resultado = np.dot(resultado,matrices[i] )
        path = resultado [0:3, 3]
        Pose.append(path)
    return resultado

def posit(q):
    global Pose
    Pose = []
    # Aplica la funcion de arriba
    for j in range(0,5):
        T_entre(0, j + 1,q)
    return Pose

def deg2rotvec(deg,etapa, normalize=True):
    
    if(etapa  == "v"):
        #z_e, x_e, y_e=np.radians(deg),np.pi,0
        z_e, x_e, y_e = (1.580+np.radians(deg)),-1.49, 0.033
    elif(etapa == "peqv"):
        z_e, x_e, y_e = (2.369+np.radians(deg)),-1.497, 0.022
    elif(etapa == "p"):
        z_e, x_e, y_e = (1.494+np.radians(deg)),-1.508, 0.028
    elif(etapa == "peqp"):
        z_e, x_e, y_e = (2.280+np.radians(deg)),-1.507, 0.028
    
    # Assuming the angles are in radians.
    c1 = np.cos(z_e/2)
    s1 = np.sin(z_e/2)
    c2 = np.cos(x_e/2)
    s2 = np.sin(x_e/2)
    c3 = np.cos(y_e/2)
    s3 = np.sin(y_e/2)
    c1c2 = c1*c2
    s1s2 = s1*s2
    w = c1c2*c3 - s1s2*s3
    x = c1c2*s3 + s1s2*c3
    y = s1*c2*c3 + c1*s2*s3
    z = c1*s2*c3 - s1*c2*s3
    angle = 2 * np.arccos(w)
    if normalize:
        norm = x*x+y*y+z*z
        if norm < 0.001:
            # when all euler angles are zero angle =0 so
            # we can set axis to anything to avoid divide by zero
            x = 1
            y = 0
            z = 0
        else:
            norm = np.sqrt(norm)
            x /= norm
            y /= norm
            z /= norm
    rotvec=angle*np.array([z,x,y])
    return rotvec[0],rotvec[1],rotvec[2]

def AH( n,th,c):

  T_a = mat(np.identity(4), copy=False)
  T_a[0,3] = a[0,n-1]
  T_d = mat(np.identity(4), copy=False)
  T_d[2,3] = d[0,n-1]

  Rzt = mat([[cos(th[n-1,c]), -sin(th[n-1,c]), 0 ,0],
	         [sin(th[n-1,c]),  cos(th[n-1,c]), 0, 0],
	         [0,               0,              1, 0],
	         [0,               0,              0, 1]],copy=False)
      

  Rxa = mat([[1, 0,                 0,                  0],
			 [0, cos(alph[0,n-1]), -sin(alph[0,n-1]),   0],
			 [0, sin(alph[0,n-1]),  cos(alph[0,n-1]),   0],
			 [0, 0,                 0,                  1]],copy=False)

  A_i = T_d * Rzt * T_a * Rxa
	    

  return A_i

def HTrans(th,c ):  
  A_1=AH( 1,th,c  )
  A_2=AH( 2,th,c  )
  A_3=AH( 3,th,c  )
  A_4=AH( 4,th,c  )
  A_5=AH( 5,th,c  )
  A_6=AH( 6,th,c  )
      
  T_06=A_1*A_2*A_3*A_4*A_5*A_6

  return T_06

def invKine(desired_pos,qnear):# T60
  
  th = mat(np.zeros((6, 8)))
  P_05 = (desired_pos * mat([0,0, -d6, 1]).T-mat([0,0,0,1 ]).T)
  
  # **** theta1 ****
  
  psi = atan2(P_05[2-1,0], P_05[1-1,0])
  phi = acos(d4 /sqrt(P_05[2-1,0]*P_05[2-1,0] + P_05[1-1,0]*P_05[1-1,0]))
  #The two solutions for theta1 correspond to the shoulder
  #being either left or right
  th[0, 0:4] = pi/2 + psi + phi
  th[0, 4:8] = pi/2 + psi - phi
  th = th.real
  
  # **** theta5 ****
  
  cl = [0, 4]# wrist up or down
  for i in range(0,len(cl)):
      c = cl[i]
      T_10 = linalg.inv(AH(1,th,c))
      T_16 = T_10 * desired_pos
      th[4, c:c+2] = + acos((T_16[2,3]-d4)/d6);
      th[4, c+2:c+4] = - acos((T_16[2,3]-d4)/d6);

  th = th.real
  
  # **** theta6 ****
  # theta6 is not well-defined when sin(theta5) = 0 or when T16(1,3), T16(2,3) = 0.

  cl = [0, 2, 4, 6]
  for i in range(0,len(cl)):
      c = cl[i]
      T_10 = linalg.inv(AH(1,th,c))
      T_16 = linalg.inv( T_10 * desired_pos )
      th[5, c:c+2] = atan2((-T_16[1,2]/sin(th[4, c])),(T_16[0,2]/sin(th[4, c])))
    
  th = th.real

  # **** theta3 ****
  cl = [0, 2, 4, 6]
  for i in range(0,len(cl)):
      c = cl[i]
      T_10 = linalg.inv(AH(1,th,c))
      T_65 = AH( 6,th,c)
      T_54 = AH( 5,th,c)
      T_14 = ( T_10 * desired_pos) * linalg.inv(T_54 * T_65)
      P_13 = T_14 * mat([0, -d4, 0, 1]).T - mat([0,0,0,1]).T
      t3 = cmath.acos((linalg.norm(P_13)**2 - a2**2 - a3**2 )/(2 * a2 * a3)) # norm ?
      th[2, c] = t3.real
      th[2, c+1] = -t3.real

  # **** theta2 and theta 4 ****

  cl = [0, 1, 2, 3, 4, 5, 6, 7]
  for i in range(0,len(cl)):
      c = cl[i]
      T_10 = linalg.inv(AH( 1,th,c ))
      T_65 = linalg.inv(AH( 6,th,c))
      T_54 = linalg.inv(AH( 5,th,c))
      T_14 = (T_10 * desired_pos) * T_65 * T_54
      P_13 = T_14 * mat([0, -d4, 0, 1]).T - mat([0,0,0,1]).T
      
      # theta 2
      th[1, c] = -atan2(P_13[1], -P_13[0]) + asin(a3* sin(th[2,c])/linalg.norm(P_13))
      # theta 4
      T_32 = linalg.inv(AH( 3,th,c))
      T_21 = linalg.inv(AH( 2,th,c))
      T_34 = T_32 * T_21 * T_14
      th[3, c] = atan2(T_34[1,0], T_34[0,0])
	      
  th = th.real
  th = np.transpose(th)
  #print("th")
  #print(np.degrees(th))
  dif_min = np.inf  # Inicializamos una variable con un valor grande como infinito
  inds_min = -1 
  for i in range(len(th)):
    dif= np.sum(np.abs(th[i][0:3]-qnear[0:3]))
    if dif < dif_min:  # Si la diferencia actual es menor que la mínima encontrada hasta ahora
              dif_min = dif
              inds_min = i
  if inds_min >= 0:  # Comprobamos que se haya encontrado una solución válida
    sol = np.array(th[inds_min,:])
    
  return sol

def inversa(pos_deseada,qnear):
    
    theta1 = qnear[0]
    theta2 = qnear[1]
    theta3 = qnear[2]
    theta4 = qnear[3]
    theta5 = qnear[4]
    theta6 = qnear[5]
    
    x=pos_deseada[0]
    y=pos_deseada[1]
    z=pos_deseada[2]
    rx=pos_deseada[3]
    ry=pos_deseada[4]
    rz=pos_deseada[5]
    
    r = Rotation.from_rotvec(np.array([rx, ry, rz]))
    RR = r.as_matrix()
    
    desired_pos = np.array([[RR[0, 0], RR[0, 1], RR[0, 2], x],
                            [RR[1, 0], RR[1, 1], RR[1, 2], y],
                            [RR[2, 0], RR[2, 1], RR[2, 2], z],
                            [0, 0, 0, 1]])
    
    
    th = np.matrix([[theta1], [theta2], [theta3], [theta4], [theta5], [theta6]])
    q00= invKine(desired_pos,th)
    q0=np.squeeze(q00)
    # c = [0]
    #location = HTrans(q00.reshape((6,1)),c )
    
    return q0

#Funcion principal
def calcular_puntos (inf, sup):
    Partes= []
    inf = np.array(inf)*10 #en mm
    sup = np.array(sup)*10 #en mm
    vals=[0,0,0]
    vals[0]=sup[0]-inf[0]
    vals[1]=sup[1]-inf[1]
    vals[2]=sup[2]-inf[2]

    pos_ff= inf+(sup - inf)/2 
    pos_ff[2]= sup[2]
    pos_ff[0] = pallet[0]- pos_ff[0]
    pos_ff[1] = pallet[1]- pos_ff[1]
    pos_ff[2] = pallet[2]+ pos_ff[2]   
    
    pos_ff_m = pos_ff/1000 #metros
    
    if( (sup - inf)[0] > (sup - inf)[1]):
        ejeY = False
        rotx = True
        rx,ry,rz = deg2rotvec(90,"p")
        angulo_place=np.array([rx,ry,rz])         
           
            
    else:
        ejeY = True
        rotx = False
        rx,ry,rz = deg2rotvec(0,"p")
        angulo_place=np.array([rx,ry,rz]) 
    
    pos_ff_mo = np.concatenate((pos_ff_m,angulo_place))
    q_near4 = [-1.4781,-2.0534,-0.8714, -1.7853, 1.5624,0]
    my_q = inversa(pos_ff_mo,q_near4 )

    #qIK= rtde_c.getInverseKinematics(pos_ff_mo,q_near4)
    
   

    # Camara
    valx = 8.0 / 2.0
    valy = 9.0 / 2.0
    
#rge Eliécer Gaitán
#4.7
#(7,462)
#Teatro

    Camara = CajaRobot([-valx -10.0, -valy - 9, 16],[valx -10.0, valy - 9, 18], my_q)

    Partes.append(Parte(Camara,'caja'))
    
    poses = posit(my_q)
    
    
    
    # Cilindro grande del punto 2 a 3
    
    P2 = poses[1]
    #print(P2)
    P3 = poses[2]
    #print(P3)
   # Cilindros.append(Cilindro(0.13,P2,P3))

    # Cilindo pequeno 1 del punto 3 a 4
    P3= poses[2]
    P4 = poses[3]
    #print(P4)
    #Cilindros.append(Cilindro(0.11,P33,P4)

    # Cilindo pequeno 2 del punto 4 a 5
    P4 = poses[3]
    P5 = poses[4]
    #print(P5)
    #Cilindros.append(Cilindro(0.11,P1,P2))

    # Cilindo pequeno del punto 5 a 6
    P5 = poses[4]
    P6 = poses[5]
    #print(P6)
   # Cilindros.append(Cilindro(0.1,P1,P2))

    # Gripper 
    valGx = 12.0 / 2.0
    valGy = 30.0 / 2.0
    
    Gripper = CajaRobot([-valGx, -valGy, 0],[valGx, valGy, 19],my_q)

    # Caja
    
    tolCaja = 2
    valx = (float(vals[0]) - tolCaja) / 2.0
    valy = (float(vals[1]) - tolCaja) / 2.0

    CajaCargada  = CajaRobot([-valx, -valy, 19],[valx, valy, 19 + float(vals[2]) - 7], my_q)

    # Generacion de partes
    
    Partes.append(Parte(Gripper,'caja'))
    Partes.append(Parte(CajaCargada,'caja'))

    Puntos= (np.array([P2,P3,P4,P5,P6])-(pallet_array/1000))*100

    #print("Puntos",Puntos[4])
    #print(" ")
    #print("Camara",Partes[0].puntos)
    #print(" ")
    #print("Gripper",Partes[1].puntos)
    #print(" ")
    #print("Caja",Partes[2].puntos)
    #print(" ")
    return Puntos[3], Puntos[4], 10




####################################################
#Prueba del codigo
inf = [20,15,10] #en cm
sup = [50,60,50]  #en cm
calcular_puntos(inf, sup)