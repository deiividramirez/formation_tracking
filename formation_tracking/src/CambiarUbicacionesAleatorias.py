# Importación de librerías
import numpy as np

def reemplazarLinea(filename, lineaNum, texto, espacios=3):
    """
        Función la cual reemplaza una línea específica de un archivo

        Parámetros:
            filename: str - Ubicación y nombre del archivo
            lineaNum: int - Número de línea a reemplazar
            texto: str - Texto a reemplazar
            espacios: int - Número de espacios a agregar al inicio la línea
    """
    archivo = open(filename, 'r').readlines()
    
    space = '\t' * espacios
    archivo[lineaNum-1] = f"{space} {texto} \n"

    reescribir = open(filename, 'w')
    reescribir.writelines(archivo)
    reescribir.close()

"""
    Ubicación del archivo "multiple_hummingbirds_example.launch" 
    el cual contiene la ubicación de los 8 drones en el espacio
    de simulación.
"""
path = "/home/leonardo/catkin_ws/src/rotors_simulator/rotors_gazebo/launch/multiple_hummingbirds_example.launch"

# Partes del archivo que se quieren reemplazar por ubicaciones aleatorias
partes = [26, 48, 70, 92, 114, 136, 158, 180]


# Nuevas ubicaciones aleatorias
alea = np.array([[np.random.uniform(-7, 7), np.random.uniform(150, 155),
             np.random.uniform(10, 15)] for _ in range(len(partes))])

# Correcion de ubicaciones para los dos drones líderes
alea[2] = [5, 155, 10]
alea[6] = [-5, 155, 10]

print(alea)

# Reemplazo de ubicaciones aleatorias en el archivo para cada dron
for num, i in enumerate(partes):
    line = f'<arg name="x" value="{alea[num][0]:.8f}" />'
    reemplazarLinea(path, i, line)
    line = f'<arg name="y" value="{alea[num][1]:.8f}" />'
    reemplazarLinea(path, i+1, line)
    line = f'<arg name="z" value="{alea[num][2]:.8f}" />'
    reemplazarLinea(path, i+2, line)
    line = f'<node name="waypoint_publisher" pkg="rotors_gazebo" type="waypoint_publisher" output="screen" args="{alea[num][0]:.8f} {alea[num][1]:.8f} {alea[num][2]:.8f} 0 {num+1}" />'
    reemplazarLinea(path, i+9, line, 2)
