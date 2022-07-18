# Importación de librerías
from scipy.special import comb  # Combinatoria
import numpy as np  # Numpy
import sympy as sp  # Sympy
import threading  # Multi-hilos


def Bernstein(i, n, t):
    """
        Polinomio de Bernstein de grado n
    """
    return comb(n, i) * (t**(i)) * (1 - t)**(n - i)


def Bezier(puntos, t):
    """
        Dado un conjunto de contro de puntos,
        se retorna la curva de Bezier de esos puntos.

        Parámetros:
        puntos: Lista de puntos de control dada por
            [ [1,1], 
                [2,3], 
                [4,5], 
                ..
                [Xn, Yn] ]
        t: Lista de valores de t para los que se quiere evaluar la curva
            o rambién puede estar dada por una lista de variables simbólicas

        Retorno:
            Lista de puntos de la curva de Bezier
            ó función simbólica de la curva de Bezier
    """
    n = len(puntos)
    BernsteinPol = np.array([Bernstein(i, n-1, t) for i in range(0, n)])
    return np.dot(puntos.T, BernsteinPol)

# Puntos de control para la curva de Bezier
# que atraviese la ventana para líder 1 y 2.
lider1 = np.array([
    [-5, -75, 5],
    [-5, -60, 5],
    [-5, -45, 5],
    [-5, -35, 5],
    [-5, -28, 5],
    [-5, -25, 5],
    [-5, -20, 5],
    [-5, -17, 5],
    [-5, -15, 5],
    [-5, -12, 5],
    [-4, -10, 4],
    [-3, -8, 3],
    [-2, -6, 2],
    [-1, -3, 1],
    [-1, -1.5, 1],
    [-1, 0, 1],
    [-1, 1.5, 1],
    [-1, 3, 1],
    [-2, 6, 2],
    [-3, 8, 3],
    [-4, 10, 4],
    [-5, 12, 5],
    [-5, 15, 5],
    [-5, 17, 5],
    [-5, 20, 5],
    [-5, 25, 5],
    [-5, 28, 5],
])
lider2 = lider1.copy()
lider2[:, 0] = -lider2[:, 0]

# Variable simbólica para poder derivar
# doble vez y obtener las entrada de aceptación
x = sp.symbols('x')
dt = 0.01
tmax = 10
t = np.arange(0, tmax, dt)/tmax


""" 
Grafica la forma de la curva de Bezier para los dos líderes.

P = Bezier(lider1, t)
P2 = Bezier(lider2, t)

import matplotlib.pyplot as plt
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.plot3D(P[0, :], P[1, :], P[2, :], label="Lider 1")
ax.plot3D(P2[0, :], P2[1, :], P2[2, :], label="Lider 2")

dist = 2.5
t = np.linspace(0, 2*dist, 100)
o = np.ones(t.shape)
ax.plot3D(-dist*o, np.zeros(t.shape), t-dist, "black")
ax.plot3D(t-dist, np.zeros(t.shape), -dist*o, "black")

t = np.linspace(-2*dist, 0, 100)
ax.plot3D(t+dist, np.zeros(t.shape), dist*o, "black")
ax.plot3D(dist*o, np.zeros(t.shape), t+dist, "black")

plt.show()
exit() """

print("\nObteniendo curvas de Bezier")
P = Bezier(lider1, x)
P2 = Bezier(lider2, x)

print("Obteniendo primeras derivadas")
PP = [P[i].diff(x) for i in range(len(P))]
PP2 = [P2[i].diff(x) for i in range(len(P2))]

print("Obteniendo segundas derivadas")
PPP = [PP[i].diff(x) for i in range(len(PP))]
PPP2 = [PP2[i].diff(x) for i in range(len(PP2))]


def p1():
    # Evaluate P at t
    print("\nObteniendo evaluaciones de la curva")
    P_t = np.array([[P[i].subs(x, tt) for tt in t] for i in range(len(P))])
    P2_t = np.array([[P2[i].subs(x, tt) for tt in t] for i in range(len(P))])
    print("\nSe obtuvo la curva de Bezier")

    np.savetxt("./src/lider1_pos.dat",
               P_t.T, delimiter=" ", header=f"{P_t.T.shape[0]} {P_t.T.shape[1]}", comments='')
    np.savetxt("./src/lider2_pos.dat",
               P2_t.T, delimiter=" ", header=f"{P_t.T.shape[0]} {P_t.T.shape[1]}", comments='')
    print("Se ha completado curvas. :D")


def p2():
    # Evaluate PP at t
    print("Obteniendo evaluaciones de la primera derivada")
    PP_t = np.array([[PP[i].subs(x, tt) for tt in t] for i in range(len(PP))])
    PP2_t = np.array([[PP2[i].subs(x, tt) for tt in t]
                     for i in range(len(PP2))])
    print("Se obtuvo la primera derivada de la curva de Bezier")

    np.savetxt("./src/lider1_vel.dat",
               PP_t.T, delimiter=" ", header=f"{PP_t.T.shape[0]} {PP_t.T.shape[1]}", comments='')
    np.savetxt("./src/lider2_vel.dat",
               PP2_t.T, delimiter=" ", header=f"{PP_t.T.shape[0]} {PP_t.T.shape[1]}", comments='')
    print("\nSe ha completado primera derivada. :D")


def p3():
    # Evaluate PPP at t
    print("Obteniendo evaluaciones de la segunda derivada")
    PPP_t = np.array([[PPP[i].subs(x, tt)/100 for tt in t]
                     for i in range(len(PPP))])
    PPP2_t = np.array([[PPP2[i].subs(x, tt)/100 for tt in t]
                      for i in range(len(PPP2))])
    print("\nSe obtuvo la segunda derivada de la curva de Bezier")

    np.savetxt("./src/lider1_ace.dat",
               PPP_t.T, delimiter=" ", header=f"{PPP_t.T.shape[0]} {PPP_t.T.shape[1]}", comments='')
    np.savetxt("./src/lider2_ace.dat",
               PPP2_t.T, delimiter=" ", header=f"{PPP_t.T.shape[0]} {PPP_t.T.shape[1]}", comments='')
    print("Se ha completado segunda derivada. :D")


print("\nGuardando los valores de posición, velocidad y aceleración en archivos .dat")


# tp1 = threading.Thread(target=p1)
# tp1.start()
# tp2 = threading.Thread(target=p2)
# tp2.start()
tp3 = threading.Thread(target=p3)
tp3.start()
