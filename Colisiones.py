# -*- coding: utf-8 -*-
"""
Created on Mon Oct 30 22:15:56 2023

@author: Daniel Robledo
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from mpl_toolkits.mplot3d import Axes3D

def point_inside_box(point, B1, B2):
    return B1[0] <= point[0] <= B2[0] and B1[1] <= point[1] <= B2[1] and B1[2] <= point[2] <= B2[2]

def distance_point_to_box(point, B1, B2):
    if point_inside_box(point, B1, B2):
        return 0
    
    dx = max(0, max(B1[0] - point[0], point[0] - B2[0]))
    dy = max(0, max(B1[1] - point[1], point[1] - B2[1]))
    dz = max(0, max(B1[2] - point[2], point[2] - B2[2]))

    return np.sqrt(dx**2 + dy**2 + dz**2)

def parallel_lines(L1, L2, B1, B2, d):
    #d = L2 - L1
    #Entramos sabiendo que si hay un eje que es par
    axes = [0,1,2]
    ejes_paralelos = []
    #shortest_distances = []
    
    #print(d)
    # Check if the line is parallel to any of the specified axes
    for axis in axes:
        if d[axis] == 0:
            ejes_paralelos.append(axis)
            
    if len(ejes_paralelos) == 0:
        return [False, "No se sabe si hay interseccion"]
    elif len(ejes_paralelos) >= 2:
        #Son paralelas y se sabe que hay interseccion porque de no haberla, ya se hubiera descartado
        return[True, True]
    else:
        P1 = np.delete(L1,ejes_paralelos[0])
        P2 = np.delete(L2,ejes_paralelos[0])
        R1_1 = np.delete(B1,ejes_paralelos[0])
        R1_2 = np.delete(B2,ejes_paralelos[0])
        return [True, interseccion_2D(P1, P2, R1_1, R1_2)]
        
        
def interseccion_2D(P1, P2, R1_1, R1_2):
    # Calculate the endpoints of the other diagonal of the rectangle
    x1, y1 = R1_1
    x2, y2 = R1_2
    R2_1 = np.array([x2, y1])
    R2_2 = np.array([x1, y2])
    #print(R1_1)
    #print(R1_2)
    #print(R2_1)
    #print(R2_2)

    # Check if the line segment P1-P2 intersects with either diagonal
    diag1 = interseccion_segmentos(P1, P2, R1_1, R1_2)
    diag2 = interseccion_segmentos(P1, P2, R2_1, R2_2)
    #print(diag1, diag2)
    if (diag1 or diag2):
        return True
    else:
        return False


def interseccion_segmentos(P1, P2, Q1, Q2):
    # Check if two line segments defined by (P1, P2) and (Q1, Q2) intersect
    Px1, Py1 = P1
    Px2, Py2 = P2
    Bx1, By1 = Q1
    Bx2, By2 = Q2
    
    #Definir la pendiente
    Pm = (Py2-Py1)/(Px2-Px1)
    Bm = (By2-By1)/(Bx2-Bx1)
    #print(Pm, Bm)
    #Paralelas
    if Bm == Pm:
        return False
    
    #Definir intercepto
    Pb = Py1 - Pm*Px1
    Bb = By1 - Bm*Bx1
    
    #Encontrar X y Y interseccion
    X_int = (Bb-Pb)/(Pm-Bm)
    Y_int = Pm*X_int + Pb
    #print(X_int, Y_int)
    #print(Y_int, Pm, X_int, Pb)
    #Hay interseccion?
    Px_min = min(Px1, Px2)
    Px_max = max(Px1, Px2)
    Bx_min = min(Bx1, Bx2)
    Bx_max = max(Bx1, Bx2)
    Py_min = min(Py1, Py2)
    Py_max = max(Py1, Py2)
    By_min = min(By1, By2)
    By_max = max(By1, By2)
    if (Px_min <= X_int <= Px_max) and (Py_min <= Y_int <= Py_max) and (Bx_min <= X_int <= Bx_max) and (By_min <= Y_int <= By_max):
        return True
    else:
        return False
    
    
    
    
    


# =============================================================================
# def diagonal_intersection(L1, L2, B1, B2):
#     # Calculate the direction vector of the line segment
#     d = (L2-L1)
# 
#     # For each axis (x, y, z), calculate the t values
#     t_values = []
#     condition = []
#     for i in range(3):
#         if d[i] == 0:
#             # The line is parallel to the plane on this axis
#             t = float('inf')
#         else:
#             t1 = (B1[i] - L1[i]) / d[i]
#             t2 = (B2[i] - L1[i]) / d[i]
#             t_values.extend([t1, t2])
#             if 0<= t1 <= 1 or 0 <= t2 <= 1:
#                 condition.append(1)
#             else:
#                 condition.append(0)
# 
#     # Check if there is at least one valid t value for each axis
#     
# 
#     # If there are valid t values for all axes, there is an intersection
#     #return len(valid_t_values) == 6
#     print(condition)
#     print(t_values)
#     return sum(condition) >=2
# =============================================================================

def muy_lejos(L1, L2, B1, B2):
    for i in range(3):
        #Alguna de los ejes no tiene chance de cruzar la caja
        if L2[i] < B1[i] or B2[i] < L1[i]:
            #print(str(L2[i]) + " < " + str(B1[i]) + " o " + str(B2[i]) + " < " + str(L1[i]) + " Eje:" + str(i))
            return True #Muy lejos
    return False

def line_plane_intersection(L1, L2, B1, B2,d):
    t_values = []
    condition = []
    for i in range(3):
        print(d)
        print(B1, L1)
        print(B2, L1)
        #Alguna de los ejes no tiene chance de cruzar la caja
        #if L1[i] <= L2[i] < B1[i] or B2[i] < L1[i] <= L2[i]:
        #    return False #No hay colisión
        if d[i] == 0:
            # The line is parallel to the plane on this axis
            #Estos casos se deben manejar por separado
            #t = float('inf')
            ##
            return "Paralelas. Revisar logica"
        else:
            t1 = (B1[i] - L1[i]) / d[i]
            print(t1)
            t2 = (B2[i] - L1[i]) / d[i]
            print(t2)
            t_values.extend([t1, t2])
            if 0<= t1 <= 1 or 0 <= t2 <= 1:
                condition.append(1)
            else:
                condition.append(0)

    # Check if there is at least one valid t value for each axis
    

    # If there are valid t values for all axes, there is an intersection
    #return len(valid_t_values) == 6
    #print(t_values)
    return sum(condition) >=2




def plot_box_and_line(L1, L2, B1, B2): #L1 y L2 como listas
    fig = plt.figure(figsize=(10, 10))
    
    ax1 = fig.add_subplot(221, projection='3d')
    ax2 = fig.add_subplot(222, projection='3d')
    ax3 = fig.add_subplot(223, projection='3d')
    ax4 = fig.add_subplot(224, projection='3d')

    # Points for the box vertices
    box_points = np.array([[B1[0], B1[1], B1[2]],
                           [B1[0], B1[1], B2[2]],
                           [B1[0], B2[1], B1[2]],
                           [B1[0], B2[1], B2[2]],
                           [B2[0], B1[1], B1[2]],
                           [B2[0], B1[1], B2[2]],
                           [B2[0], B2[1], B1[2]],
                           [B2[0], B2[1], B2[2]]])

    # Define the box faces
    faces = [[box_points[0], box_points[1], box_points[5], box_points[4]],
             [box_points[7], box_points[6], box_points[2], box_points[3]],
             [box_points[0], box_points[1], box_points[3], box_points[2]],
             [box_points[7], box_points[6], box_points[4], box_points[5]],
             [box_points[7], box_points[3], box_points[1], box_points[5]],
             [box_points[0], box_points[4], box_points[6], box_points[2]]]

    # Define viewpoints
    elevations = [20, 20, 60, 60]
    azimuths = [30, -30, 0, 90]

    for ax, elev, azim in zip([ax1, ax2, ax3, ax4], elevations, azimuths):
        # Plot box with transparent blue color
        ax.add_collection3d(Poly3DCollection(faces, facecolors='blue', linewidths=1, edgecolors='b', alpha=0.25))
        
        # Calculate the scale factor for the line segment
        #scale_factor = max(B2[0] - B1[0], B2[1] - B1[1], B2[2] - B1[2])

        # Scale and plot the line segment
        for i in range(len(L1)):
            l1 = L1[i]
            l2 = L2[i]
            ax.plot([l1[0], l2[0]], [l1[1], l2[1]], [l1[2], l2[2]], 'r-')

        
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title(f'Viewpoint (elev={elev}, azim={azim})')
        ax.view_init(elev=elev, azim=azim)

    plt.tight_layout()
    plt.show()


def shortest_distance(L1, L2, B1, B2):
    # If either endpoint is inside or on the surface of the box, return 0
    if point_inside_box(L1, B1, B2) or point_inside_box(L2, B1, B2):
        #print("Dentro de la caja")
        return True #Hay colision
    if muy_lejos(L1, L2, B1, B2):
        #print("Muy lejos")
        return False #No hay chance de colision
    
# =============================================================================
#     distances = []
#     # 1. Check endpoints of the line segment
#     distances.append(distance_point_to_box(L1, B1, B2))
#     distances.append(distance_point_to_box(L2, B1, B2))
# =============================================================================

    d = L2-L1
    # 2. Check intersections with box planes
    #Parallel[0] <- Son paralelas?
    #Parallel[1] <- Hay colision?
    parallel = parallel_lines(L1, L2, B1, B2, d)
    if parallel[0]:
        #print("Paralelas. Da respuesta")
        return parallel[1] 
    #distances.append(parallel)
    #print("Parallel ="+str(parallel))
    #if parallel == 0:
    #    return parallel
    # 3. Check other cases
    #distances.append(diagonal_intersection(L1, L2, B1, B2))
    #print("Interseccion")
    x = line_plane_intersection(L1, L2, B1, B2, d)
    #axes = [0,1,2]    
    #distances.append(line_plane_intersection(L1, L2, B1, B2, d))
    
    #if len(distances) == 0:
    #    return "No hay distancia"
    #return distances
    return x
  
def crear_lineas(P5, P6, diametro):
    radio = 0.5*diametro
    radio_array = np.array([radio, radio, radio])
    transformaciones = []
    
    transformaciones.append(np.array( [1,1,1]))
    transformaciones.append(np.array( [1,1,0]))
    transformaciones.append(np.array( [1,1,-1]))
    transformaciones.append(np.array( [1,0,1]))
    transformaciones.append(np.array( [1,0,0]))
    transformaciones.append(np.array( [1,0,-1]))
    transformaciones.append(np.array( [1,-1,1]))
    transformaciones.append(np.array( [1,-1,0]))
    transformaciones.append(np.array( [1,-1,-1]))
    transformaciones.append(np.array( [0,1,1]))
    transformaciones.append(np.array( [0,1,0]))
    transformaciones.append(np.array( [0,1,-1]))
    transformaciones.append(np.array( [0,0,1]))
    transformaciones.append(np.array( [0,0,-1]))
    transformaciones.append(np.array( [0,-1,1]))
    transformaciones.append(np.array( [0,-1,0]))
    transformaciones.append(np.array( [0,-1,-1]))
    transformaciones.append(np.array( [-1,1,1]))
    transformaciones.append(np.array( [-1,1,0]))
    transformaciones.append(np.array( [-1,1,-1]))
    transformaciones.append(np.array( [-1,0,1]))
    transformaciones.append(np.array( [-1,0,0]))
    transformaciones.append(np.array( [-1,0,-1]))
    transformaciones.append(np.array( [-1,-1,1]))
    transformaciones.append(np.array( [-1,-1,0]))
    transformaciones.append(np.array( [-1,-1,-1]))
    
    
    
    L1 = []
    L2 = []
    for t in transformaciones:
        l1 = t*radio_array+P5
        l2 = t*radio_array+P6
        L1.append(l1)
        L2.append(l2)
    return L1, L2

def verificar_colisiones(L1, L2, B1, B2): #L1 y L2 son listas con los tríos de puntos que componen cada línea de la malla
    rango = len(L1)
    x = False
    for i in range(rango):
        l1 = L1[i]
        l2 = L2[i]
        x = shortest_distance(l1, l2, B1, B2)
        if x:
            return x
    return x
    


def plot_cylinder(ax, x1, y1, z1, x2, y2, z2, radius):
    # Create a cylinder between two points
    v = np.array([x2 - x1, y2 - y1, z2 - z1])
    mag = np.linalg.norm(v)
    v = v / mag  # normalize vector

    # Create a 3D plot
    phi = np.linspace(0, 2 * np.pi, 100)
    x_cylinder = x1 + radius * np.cos(phi)
    y_cylinder = y1 + radius * np.sin(phi)
    z_cylinder = z1 + mag * np.linspace(0, 1, 100)

    ax.plot(x_cylinder, y_cylinder, z_cylinder, color='b')

# Example usage
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

x1, y1, z1 = 1, 2, 3  # First endpoint
x2, y2, z2 = 4, 5, 6  # Second endpoint
radius = 1

plot_cylinder(ax, x1, y1, z1, x2, y2, z2, radius)

# You can customize the plot as needed
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

plt.show()



# Example
#caja
B1 = np.array([1, 1, 1])
B2 = np.array([2, 2, 2])
#P5 = np.array([0,0,5])
#P6 = np.array([2,2,3])
#diametro = 0.5
#L1, L2 = crear_lineas(P5, P6, diametro)
#plot_box_and_line(L1, L2, B1, B2)

#Pasa por esquina de la caja - espera 0
L1 = np.array([0.9, 1.9, 0.9])
L2 = np.array([1.1, 2.1, 1.1])
#plot_box_and_line(L1, L2, B1, B2)
distance = shortest_distance(L1, L2, B1, B2)
print("Shortest distance T3 - 0 esquina:", distance)


# =============================================================================
# #Cruza la caja - espera 0
# L1 = np.array([1, 1, 1])
# L2 = np.array([2, 2, 2])
# #plot_box_and_line(L1, L2, B1, B2)
# distance1 = shortest_distance(L1, L2, B1, B2)
# print("Shortest distance T1 - 0 diagonal:", distance1)
# 
# #paralela a la caja - espera 1.5
# L1 = np.array([0, 3.5, 1])
# L2 = np.array([4, 3.5, 3.5])
# #plot_box_and_line(L1, L2, B1, B2)
# distance = shortest_distance(L1, L2, B1, B2)
# print("Shortest distance T2 - 1.5 paralela:", distance)
# 
# #Pasa por esquina de la caja - espera 0
# L1 = np.array([0.9, 1.9, 0.9])
# L2 = np.array([1.1, 2.1, 1.1])
# #plot_box_and_line(L1, L2, B1, B2)
# distance = shortest_distance(L1, L2, B1, B2)
# print("Shortest distance T3 - 0 esquina:", distance)
# 
# #Dentro de la caja - espera 0
# L1 = np.array([1.6, 1.6, 1.6])
# L2 = np.array([1.7, 1.7, 1.7])
# #plot_box_and_line(L1, L2, B1, B2)
# distance = shortest_distance(L1, L2, B1, B2)
# print("Shortest distance T4 - 0 dentro:", distance)
# 
# #diagonal a la caja. La cruza sin tocar un borde - espera 0
# L1 = np.array([1.1, 1.1, 0.2])
# L2 = np.array([1.5, 2.1, 3.2])
# #plot_box_and_line(L1, L2, B1, B2)
# distance = shortest_distance(L1, L2, B1, B2)
# print("Shortest distance T5 - 0 cruza la caja:", distance)
# =============================================================================
# =============================================================================
# #sobre una cara - espera 1
# L1 = np.array([1.2, 1.2, 4])
# L2 = np.array([2.8, 2.8, 6])
# #plot_box_and_line(L1, L2, B1, B2)
# distance = shortest_distance(L1, L2, B1, B2)
# print("Shortest distance T6 - 2 sobre cara:", distance)
# =============================================================================


#diagonal, near miss - espera false
L1 = np.array([-10, 1.3, 1.7])
L2 = np.array([1, 1.5, 2.8])
#plot_box_and_line(L1, L2, B1, B2)
distance = shortest_distance(L1, L2, B1, B2)
print("Shortest distance T6 - False near miss:", distance)

# =============================================================================
# #Paralela 1 eje con interseccion - espera True
# L1 = np.array([0, 1.5, 1.5])
# L2 = np.array([2, 1.5, 3.5])
# #plot_box_and_line(L1, L2, B1, B2)
# distance = shortest_distance(L1, L2, B1, B2)
# print("Shortest distance T7 - true paralela:", distance)
# =============================================================================

# =============================================================================
# #Paralela 1 eje sin interseccion - espera False
# L1 = np.array([0, 1.5, 2.5])
# L2 = np.array([2, 1.5, 4.5])
# #plot_box_and_line(L1, L2, B1, B2)
# distance = shortest_distance(L1, L2, B1, B2)
# print("Shortest distance T8 - falso paralela:", distance)
# 
# =============================================================================
