import numpy as np
from copy import copy
import rbdl
cos=np.cos; sin=np.sin; pi=np.pi

def dh(d, theta, a, alpha):
    """
    Calcular la matriz de transformacion homogenea asociada con los parametros
    de Denavit-Hartenberg.
    Los valores d, theta, a, alpha son escalares.
    """
    # Escriba aqui la matriz de transformacion homogenea en funcion de los valores de d, theta, a, alpha
    T = np.array([[cos(theta), -cos(alpha)*sin(theta), sin(alpha)*sin(theta), a*cos(theta)],
                  [sin(theta), cos(alpha)*cos(theta), -sin(alpha)*cos(theta), a*sin(theta)],
                  [0, sin(alpha), cos(alpha), d],
                  [0, 0, 0, 1]])
    return T



def fkine_kuka(q):
    """
    Calcular la cinematica directa del robot kuka dados sus valores articulares.
    q es un vector numpy de la forma [q1, q2, q3, q4, q5, q6]
    """
    # Longitudes (en metros)
    L1 = 0.34
    L2 = 0.4
    L3 = 0.4
    L4 = 0.126


    # d = distancia entre z-1 origen anterior
    # theta = rotación respecto de z-1
    # a = distancia entre x
    # alpha = rotación respecto a x

    # Matrices DH (completar), emplear la funcion dh con los parametros DH para cada articulacion

    T1 = dh(L1, q[0], 0,    -pi/2 )
    T2 = dh(0,  q[1], 0,    pi/2)
    T3 = dh(L2, q[2], 0,    -pi/2)
    T4 = dh(0,  q[3], 0,    pi/2)
    T5 = dh(L3, q[4], 0,    -pi/2)
    T6 = dh(0,  q[5], 0,    pi/2)
    T7 = dh(L4, q[6], 0,    0 )
    # Efector final con respecto a la base
    T = T1 @ T2 @ T3 @ T4 @ T5 @ T6 @ T7# @ T8
    return T

def jacobian(q, delta=0.001):
    """
    Jacobiano para el kuka
    """
    # Alocar espacio: matriz de ceros del tamaño adecuado (2x2 para XY del robot RR)
    J = np.zeros((3,7)) # (3,7)-------------
    # Posición en la configuración q
    Th = fkine_kuka(q)
    x = Th[0:3,3]  # NECESITO LA CINEMÁTICA DIRECTA

    # Iteraciones para las derivadas columna por columna
    for i in range(7): # (7)-----------------
        # Copiar la configuracion articular inicial (importante)
        dq = np.copy(q)

        # Incrementar la articulacion i-esima usando un delta: qi+delta
        dq[i] = dq[i] + delta
        # Posición luego del incremento (con qi+delta)
        dTh = fkine_kuka(dq)
        dx = dTh[0:3,3]

        # Columna i usando diferencias finitas
        columna_i = 1/delta * (dx-x)
        # Almacenamiento de la columna i
        J[:,i] = columna_i

    return J

def ikine_kuka(xdes, q0):
    """
    Cinemática inversa - Metodo de newton
    """
    epsilon  = 1e-4
    max_iter = 1000
    delta    = 0.001

    q  = copy(q0)
    for i in range(max_iter):
        J = jacobian(q,delta)   # Matriz Jacobiana
        Th = fkine_kuka(q)      # Matriz Actual
        xact = Th[0:3,3]        # Posición
        e = xdes-xact           # Error
        q = q + np.dot(np.linalg.pinv(J),e)
        #Condicion de termino
        if(np.linalg.norm(e)<epsilon):
            break
        pass
    return q

def ik_gradient_kuka(xdes, q0):
    """
    Calcular la cinematica inversa de UR5 numericamente a partir de la configuracion articular inicial de q0.
    Emplear el metodo gradiente
    """
    epsilon  = 1e-4
    max_iter = 1000
    delta    = 0.001
    alpha    = 0.5

    q  = copy(q0)
    for i in range(max_iter):
        # Main loop
        J = jacobian(q,delta)         # Matriz Jacobiana
        Td = fkine_kuka(q)             # Matriz Actual
        xact = Td[0:3,3]              # Posicion Actual
        e = xdes-xact                 # Error
        q = q + alpha*np.dot(J.T,e)   # Metodo de la gradiente
        #Condicion de termino
        if(np.linalg.norm(e)<epsilon):
            break
        pass
    return q


class Robot(object):
    def __init__(self, q0, dq0, ndof, dt):
        self.q = q0    # numpy array (ndof x 1)
        self.dq = dq0  # numpy array (ndof x 1)
        self.M = np.zeros([ndof, ndof])
        self.b = np.zeros(ndof)
        self.dt = dt
        self.robot = rbdl.loadModel('../urdf/lbr_iiwa7_r800.urdf')

    def send_command(self, tau):
        rbdl.CompositeRigidBodyAlgorithm(self.robot, self.q, self.M)
        rbdl.NonlinearEffects(self.robot, self.q, self.dq, self.b)
        ddq = np.linalg.inv(self.M).dot(tau-self.b)
        self.q = self.q + self.dt*self.dq
        self.dq = self.dq + self.dt*ddq

    def read_joint_positions(self):
        return self.q

    def read_joint_velocities(self):
        return self.dq