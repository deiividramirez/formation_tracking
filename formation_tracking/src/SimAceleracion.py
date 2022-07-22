# Importación de librerías
import matplotlib.pylab as plt
import numpy as np

# Se cargan los archivos de datos de los dos lideres
lider1 = (np.loadtxt("./src/lider1_ace.dat", skiprows=1))
lider2 = (np.loadtxt("./src/lider2_ace.dat", skiprows=1))
print(lider1.shape)
print(lider2.shape)

# Condiciones iniciales las cuales estarán en la simulación de Gazebo
v1 = [np.array([0, 0, 0])]
x1 = [np.array([-5, 155, 10])]
v2 = [np.array([0, 0, 0])]
x2 = [np.array([5, 155, 10])]

# dt para el método de Euler
dt = .01

# Simulación por medio de Euler
for i in range(len(lider1)):
    v1.append(v1[-1] + dt * lider1[i, :])
    x1.append(x1[-1] + dt * v1[i])

    v2.append(v2[-1] + dt * lider2[i, :])
    x2.append(x2[-1] + dt * v2[i])

# Conversión a numpy array
x1 = np.array(x1)
x2 = np.array(x2)

# plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot3D(x1[:, 0], x1[:, 1], x1[:, 2], 'c-', label='Lider 1')
ax.plot3D(x2[:, 0], x2[:, 1], x2[:, 2], 'm-', label='Lider 2')
ax.plot3D(x1[0, 0], x1[0, 1], x1[0, 2], 'c*')
ax.plot3D(x2[0, 0], x2[0, 1], x2[0, 2], 'm*')
ax.legend()

print(x1[0, :])
print(x2[0, :])

# VENTANA

dist = 2.5
t = np.linspace(0, 2*dist, 100)
o = np.ones(t.shape)
ax.plot3D(-dist*o, np.zeros(t.shape), t-dist+5, "black")
ax.plot3D(t-dist, np.zeros(t.shape), -dist*o+5, "black")

t = np.linspace(-2*dist, 0, 100)
ax.plot3D(t+dist, np.zeros(t.shape), dist*o+5, "black")
ax.plot3D(dist*o, np.zeros(t.shape), t+dist+5, "black")

ax.azim = -90
ax.elev = 40
# ax.dist = 10

plt.savefig("./src/out/trayectoria.png", dpi=300, bbox_inches='tight', transparent=True)
plt.show()
