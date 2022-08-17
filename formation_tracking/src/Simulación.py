from mpl_toolkits.mplot3d.axes3d import Axes3D
from matplotlib import animation
import matplotlib.pylab as plt
import numpy as np

# Colores y etiquetas de los drones
colores = ["red", "green", "blue", "orange", "purple", "black", "yellow", "pink"]
labels = ["Dron 1", "Dron 2", "Dron 3*", "Dron 4", "Dron 5", "Dron 6", "Dron 7*", "Dron 8"]
trayec = ["gray", "gray", "blue", "gray", "gray", "gray", "yellow", "gray"]



# VARIBALES PARA CONTROL DE GUARDADO
#----------------------------------------------------------------------------------------------------------------------

# Guardar en gif
save = True
# save = False

# Mostrar estadisticas
estad = True
# estad = False

# Mostrar animación
anim = True
# anim = False



#----------------------------------------------------------------------------------------------------------------------

def ortProj(x: np.array):
    """
    Función la cual calcula la matriz de proyección ortogonal de un vector
    
    Parámetros:
        x: np.array
    
    Retorno:
        Matriz de tipo np.array
    """
    m = len(x)
    x = x.reshape(m, 1)
    return np.eye(m) - np.dot(x, x.T) / np.linalg.norm(x)**2



# CONFIGURACIÓN INICIAL
# ----------------------------------------------------------------------------------------------------------------------


# Dimensión y número de drones
d = 3
n = 8

# Bearings cubo
A = np.array([1, 1, 1])
B = np.array([1, 1, -1])
C = np.array([1, -1, 1])
D = np.array([1, -1, -1])
E = np.array([-1, 1, 1])
F = np.array([-1, 1, -1])
G = np.array([-1, -1, 1])
H = np.array([-1, -1, -1])
All = np.array([A, B, C, D, E, F, G, H])

gijA = np.zeros((n, n, d))
for i in range(n):
    for j in range(n):
        gijA[i, j, :] = (All[j] - All[i])/np.linalg.norm(All[j] - All[i]
                                                         ) if np.linalg.norm(All[i] - All[j]) != 0 else np.zeros(d)

print(f"Bearings: {gijA}\n")



# PLOT FORMACIÓN Y BEARINGS SI: estad = True
# ----------------------------------------------------------------------------------------------------------------------

if estad:
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    plt.subplots_adjust(right=0.8)
    ax.set_title("Formación deseada y bearings")
    ax.set_box_aspect([1, 1, 1])
    for i in range(n):
        for j in range(n):
            ax.plot3D([All[i, 0], All[i, 0] + gijA[i, j, 0]], [All[i, 1], All[i, 1] + gijA[i, j, 1]],
                    [All[i, 2], All[i, 2] + gijA[i, j, 2]], color=colores[j], alpha=0.5)
        ax.plot3D([All[i, 0]], [All[i, 1]], [All[i, 2]],
                'o', color=colores[i], alpha=0.5, label=labels[i])
    ax.legend(bbox_to_anchor=(1.1, 0.5), loc="center left", borderaxespad=0)
    if save:
        plt.savefig("./src/out/formacion.png", dpi=300, bbox_inches='tight', transparent=True)



# CONDICIONES INICIALES
# ----------------------------------------------------------------------------------------------------------------------

lider_de_ac = np.loadtxt("./src/lider1_ace.dat", skiprows=1)
lider_iz_ac = np.loadtxt("./src/lider2_ace.dat", skiprows=1)

# # Posiciones iniciales aleatorias de los drones
# x = np.array([[np.random.uniform(-7, 7), np.random.uniform(150, 155),
#              np.random.uniform(10, 15)] for _ in range(n)])

# # Posiciones iniciales de los drones líderes de acuerdo a
# # a lo acordado por SimAceleración.py
# x[2] = [5, 155, 10]
# x[6] = [-5, 155, 10]

x = np.array([
    [-5.07392138, 151.93297595, 12.72742753],
    [3.02878723, 154.47274653, 14.66131689],
    [5, 155, 10],
    [-6.01750066, 150.62870301, 11.22141663],
    [2.91336507, 151.76084991, 10.34150496],
    [4.91667537, 152.53318023, 11.42084397],
    [-5, 155, 10],
    [1.98935014, 154.94513920, 14.54221830]
])

# Vector de velocidades iniciales cero
v = np.zeros((n, d))

# Arrays para guardar la posición y velocidades de los drones
arrx = [x]
arrv = [v]
print(f"Posiciones iniciales: {x} \n")
print(f"Velocidades iniciales: {v} \n")



# PLOTEO CONDICIONES INICIALES SI: estad = True
# ----------------------------------------------------------------------------------------------------------------------

if estad:
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    plt.subplots_adjust(right=0.8)
    ax.set_title("Condiciones iniciales")
    ax.set_box_aspect([1, 1, 1])
    for i in range(n):
        for j in range(n):
            ax.plot3D([x[i, 0], x[i, 0] + gijA[i, j, 0]], [x[i, 1], x[i, 1] + gijA[i, j, 1]],
                    [x[i, 2], x[i, 2] + gijA[i, j, 2]], color=colores[j], alpha=0.5)
        ax.plot3D([x[i, 0]], [x[i, 1]], [x[i, 2]],
                'o', color=colores[i], alpha=0.5, label=labels[i])
    ax.legend(bbox_to_anchor=(1.1, 0.5), loc="center left", borderaxespad=0)
    if save:
        plt.savefig("./src/out/condiciones_iniciales.png", dpi=300, bbox_inches='tight', transparent=True)



# CONFIGURACIÓN DE TIEMPO DE EJECUCIÓN Y dt
# ----------------------------------------------------------------------------------------------------------------------

tmax = 10
dt = .01
t = np.linspace(0, tmax, lider_iz_ac.shape[0])



# SIMULACIÓN
# ----------------------------------------------------------------------------------------------------------------------

# Ganancias del controlador
Kp = 3
Kv = 3

# Simulación con Euler
vnew = np.zeros((n, d))
u = [np.zeros((n, d))]
for num, tt in enumerate(t):
    for i in range(n):
        if i != 2 and i != 6:
            sumaP = np.zeros((d, d))
            for j in range(n):
                if i != j:
                    sumaP += ortProj(gijA[i, j, :])

            suma = np.zeros(d)
            for j in range(n):
                if i != j:
                    suma += np.dot(np.linalg.inv(sumaP), np.dot(ortProj(gijA[i, j, :]),
                                   (Kp*(x[i]-x[j]) + Kv*(v[i]-v[j])) - u[-1][j]))
            vnew[i, :] = -suma
        else:
            if i == 2:
                vnew[i, :] = lider_iz_ac[num]
            else:
                vnew[i, :] = lider_de_ac[num]
    
    v = v + dt * vnew
    x = x + dt * v

    u.append(vnew)
    arrx.append(x)
    arrv.append(v)

print(f"Posiciones finales: {x} \n")

# Conversión a numpy array para poder plotear
arrx = np.array(arrx)
arrv = np.array(arrv)



# PLOTEO DE VELOCIDADES SI: estad = True
# ----------------------------------------------------------------------------------------------------------------------
if estad:
    fig = plt.figure()
    ax = plt.axes()
    ax.set_title("Velocidades")
    plt.subplots_adjust(right=0.8)
    [plt.plot(t, arrv[1:, i, 0], color=colores[i],
            label = labels[i]) for i in range(n)]
    ax.legend(bbox_to_anchor=(1.02, 0.5), loc="center left", borderaxespad=0)
    if save:
        plt.savefig("./src/out/velocidades.png", dpi=300, bbox_inches='tight', transparent=True)



# ANIMACIÓN DE LAS POSICIONES
# ----------------------------------------------------------------------------------------------------------------------
def animate(num: int):
    # Configuración del plot
    ax.cla()
    ax.set_title(f"Tiempo: {num*dt:.2f}s")
    ax.set_xlim3d(np.min(arrx[num, :, 0])-1, np.max(arrx[num, :, 0])+1)
    ax.set_ylim3d(np.min(arrx[num, :, 1])-1, np.max(arrx[num, :, 1])+1)
    ax.set_zlim3d(np.min(arrx[num, :, 2])-1, np.max(arrx[num, :, 2])+1)
    ax.set_box_aspect([1, 1, 1])

    # Ventana
    dist = 2.5
    t = np.linspace(0, 2*dist, 100)
    o = np.ones(t.shape)
    ax.plot3D(-dist*o, np.zeros(t.shape), t-dist+5, "black")
    ax.plot3D(t-dist, np.zeros(t.shape), -dist*o+5, "black")

    t = np.linspace(-2*dist, 0, 100)
    ax.plot3D(t+dist, np.zeros(t.shape), dist*o+5, "black")
    ax.plot3D(dist*o, np.zeros(t.shape), t+dist+5, "black")

    # Recorrer todos los drones
    for i in range(n):
        for j in range(n):
            # Ploteo de los bearings
            ax.plot3D([arrx[num, i, 0], arrx[num, i, 0] + gijA[i, j, 0]], [arrx[num, i, 1], arrx[num, i, 1] +
                      gijA[i, j, 1]], [arrx[num, i, 2], arrx[num, i, 2] + gijA[i, j, 2]], color=colores[j], alpha=0.5)
        # Ploteo de los drones
        ax.plot3D([arrx[num, i, 0]], [arrx[num, i, 1]], [
                  arrx[num, i, 2]], 'o', color=colores[i], alpha=0.5, label=labels[i])
        # Ploteo de la trayectoria dada por los drones
        ax.plot3D(arrx[:num, i, 0], arrx[:num, i, 1],
                  arrx[:num, i, 2], '-', color=trayec[i], alpha=0.5)
        # Ploteo de los puntos iniciales
        ax.plot3D([arrx[0, i, 0]], [arrx[0, i, 1]], [
            arrx[0, i, 2]], 'o', color=trayec[i], alpha=0.5)
    ax.legend(bbox_to_anchor=(1.1, 0.5), loc="center left", borderaxespad=0)

    # """                                                                                                                                                    
    # Scaling is done from here...                                                                                                                           
    # """
    # x_scale = 1
    # y_scale = 3
    # z_scale = 1

    # scale=np.diag([x_scale, y_scale, z_scale, 1.0])
    # scale=scale*(1.0/scale.max())
    # scale[3,3]=1.0

    # def short_proj():
    #     return np.dot(Axes3D.get_proj(ax), scale)

    # ax.get_proj=short_proj
    # """                                                                                                                                                    
    # to here                                                                                                                                                
    # """

# Animación
if anim:
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    # ax.azim = -15
    # ax.elev = 15
    # ax.dist = 8
    ax.set_title("Simulación")
    plt.subplots_adjust(right=0.8)
    line_ani = animation.FuncAnimation(
        fig, animate, interval=25, frames=len(t),  repeat=True)



    # GUARDAR SIMULACIÓN EN ARCHIVO .gif
    # ----------------------------------------------------------------------------------------------------------------------

    if save:
        print("\nGuardando animación")
        f = "./src/out/formation.gif"
        writergif = animation.PillowWriter(fps=len(t)/(tmax+1))
        line_ani.save(f, writer=writergif)
        print("Se guardó.")

if estad or anim:
    plt.show()
