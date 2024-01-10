# -*- coding: utf-8 -*-
"""
Created on Wed Aug 23 18:58:30 2023

@author: Daniel Robledo
"""

# NOTA: Toda distancia está en cm a menos que se indique lo contrario

import csv
import datetime
import os
import Visualización as vis
from copy import deepcopy
from Coordenadas_robot import *
from Colisiones import *


class Rotaciones:
    
    def __init__(self, R):
        self.x = R[0]
        self.y = R[1]
        self.z = R[2]
        self.area = self.x * self.y
        self.volumen = self.area * self.z
        self.R = R


class TipoCaja:
    #Rotaciones_Posibles = [] #Se llena con las rotaciones posibles. Es tipo CajaRotada
    #R = [] 
    #L = Length, W = Width, H = Height
    #L corresponde a x, W a y y H a z
    #Rx, Ry y Rz dictan las rotaciones posibles de la caja. Son enteros entre 0 y 3
    def __init__(self, ID_Tipo_Caja, L, W, H,Rx,Ry,Rz): 
        self.ID_Tipo = ID_Tipo_Caja
        self.R = [Rx, Ry, Rz]
        self.Rotaciones_Posibles = []
        self.volumen= L * W * H
        self.L = L
        self.W = W
        self.H = H
        
        #Hacer rotaciones posibles
        for i in range(3):
            r = self.R[i]
            #Aqui R_temp pasa a tener las medidas en cm, las que se envian a Rotaciones
            R_temp = [L, W, H]
            if r > 0:
                z_temp = H
                R_temp[2] = r
                R_temp[i] = z_temp
            if r == 1 or r == 3:
                
                self.Rotaciones_Posibles.append(Rotaciones(R_temp))
            if r > 1:
                y_temp = R_temp[1]
                R_temp[1] = R_temp[0]
                R_temp[0] = y_temp
                self.Rotaciones_Posibles.append(Rotaciones(R_temp))
            
        
class Caja:
    
    def __init__(self, TipoCaja: TipoCaja, ID_caja):
        self.ID_caja = ID_caja
        self.TipoCaja = TipoCaja
        # se establece el valor como False mientras se empaca la caja
        # se declaran las variables que se usarán para empacar la caja
        self.Rotacion_Asignada = False
        self.x1 = 0
        self.y1= 0
        self.z1= 0
        self.x2= 0
        self.y2= 0
        self.z2= 0
        
        
# =============================================================================
# class CajaEmpacada:
#     
#     #Las coordenadas corresponden a la coordenada de la esquina más cercana al punto (0, 0, 0)
#     #Caja es la caja que se empaca
#     #Rotacion_asignada es la rotacion de la caja que se escoge entre las posibilidades dadas por el tipo de caja
#     def __init__(self, Caja: Caja , x1, y1, z1):
#         self.x1 = x1
#         self.y1 = y1
#         self.z1 = z1
#         self.x2 = x1 + Caja.Rotacion_Asignada.x
#         self.y2 = y1 + Caja.Rotacion_Asignada.y
#         self.z2 = z1 + Caja.Rotacion_Asignada.z
#         self.Caja = Caja
# =============================================================================
        
        
class EspacioMaximal:
    
    def __init__(self, x1, y1, z1, x2, y2, z2):
        #self.ID = ID
        self.x1 = x1
        self.y1 = y1
        self.z1 = z1
        self.x2 = x2
        self.y2 = y2
        self.z2 = z2
        #dx, dy ,dz
        #self.area_plana = ( self.x2 - self.x1) * (self.y2 - self.y1)
        self.p1 = [x1, y1, z1]
        self.p2 = [x2, y2, z2]
        self.best_fit = 1000
        
    def EsValido(self):
        return (self.x2 > self.x1) and (self.y2 > self.y1) and (self.z2 > self.z1)

        
        
class TipoPallet:
    
    def __init__(self, ID, x_max, y_max, z_max):
        self.ID = ID
        self.x = x_max
        self.y = y_max
        self.z = z_max
        
        
# =============================================================================
# NOTAS
#class Pallet:
#     #Cajas = [] #Almacena las cajas empacadas en el Pallet. Tipo CajaRotada
#     #Espacios = [] #Almacena los espacios disponibles tipo EspacioMaximal
#     
#     def __init__(self, ID_Pallet, TipoPallet: TipoPallet):
#         #se define únicamente el punto más alejado del origen porque el origen es una esquina del pallet
#         self.ID_Pallet = ID_Pallet
#         self.TipoPallet = TipoPallet
#         Espacio_inicial = EspacioMaximal(0, 0, 0, TipoPallet.x, TipoPallet.y, TipoPallet.z)
#         self.Espacios = [Espacio_inicial]
#         #Espacios se diferencia de Espacios_posibles porque El primero es todo espacio maximal disponible en el pallet
#         #El segundo es todo espacio maximal posible para la caja en cuestión.
#         self.Espacios_posibles = []
#         self.cajas_empacadas =[]
#         
#         
#     def crear_espacios_posibles(self, caja: Caja):
#         self.Espacios_posibles = []
#         Hay_Espacio = False
#         for espacio in self.Espacios:
#             cabe = False
#             area_espacio = espacio.area_plana
#             altura_espacio = espacio.z2 - espacio.z1
#             for rotacion in caja.Rotaciones_Posibles:
#                 area_caja = rotacion.area
#                 altura_caja = rotacion.z
#                 if area_caja < area_espacio:
#                     if altura_caja < altura_espacio:
#                         cabe = True
#             if cabe:
#                 self.Espacios_posibles.append(espacio)
#         if len(self.Espacios_posibles) > 0:
#             Hay_Espacio = True
#         return Hay_Espacio
#             
#     def cabe_caja_en_espacio(self, caja: Caja, espacio: EspacioMaximal):
#         pass
# =============================================================================
 
def dar_best_fit(espacio):
    return espacio.best_fit
       
class Pallet:

    def __init__(self, ID_Pallet, TipoPallet: TipoPallet):
        self.ID_Pallet = ID_Pallet
        self.TipoPallet = TipoPallet
        Espacio_inicial = EspacioMaximal(0, 0, 0, TipoPallet.x, TipoPallet.y, TipoPallet.z)
        self.Espacios = [Espacio_inicial]
        self.Espacios_posibles = []
        self.cajas_empacadas = []
        
        # Initialize logger
        self.logger = PalletLogger(self.ID_Pallet, (TipoPallet.x, TipoPallet.y, TipoPallet.z))
    #1
    def crear_espacios_posibles(self, caja: Caja):
        self.Espacios_posibles = []
        for espacio in self.Espacios:
            for rotacion in caja.TipoCaja.Rotaciones_Posibles:
                tamano_x = espacio.x2 - espacio.x1
                tamano_y = espacio.y2 - espacio.y1
                tamano_z = espacio.z2 - espacio.z1
                if (tamano_x >= rotacion.x and
                        tamano_y >= rotacion.y and
                        tamano_z >= rotacion.z):
                    #best fit se define como el margen en X y Y + el nivel en el que está la caja 
                    espacio.best_fit = (tamano_x - rotacion.x) + (tamano_y - rotacion.y) + espacio.z1
                    self.Espacios_posibles.append(espacio)
                    break
        #if len(self.Espacios_posibles)>0:
            #esp = sorted(self.Espacios_posibles,key=dar_best_fit)
            #self.Espacios_posibles.sort(key=dar_best_fit)
        #print(len(esp))
        #print(len(self.Espacios_posibles))
        #self.imprimir_espacios("Posibles", self.Espacios_posibles)
        return bool(self.Espacios_posibles)
    
    
    
    #2
    def place_box_in_space(self, caja: Caja, visualizar, revisar_colisiones):
        for espacio in self.Espacios_posibles:
            for rotacion in caja.TipoCaja.Rotaciones_Posibles:
                if (espacio.x2 - espacio.x1 >= rotacion.x and
                    espacio.y2 - espacio.y1 >= rotacion.y and
                    espacio.z2 - espacio.z1 >= rotacion.z):
    
                    # Asignar coordenadas de la caja en base a la rotación
                    caja_temp = deepcopy(caja)
                    caja_temp.x1 = espacio.x1
                    caja_temp.y1 = espacio.y1
                    caja_temp.z1 = espacio.z1
                    caja_temp.x2 = espacio.x1 + rotacion.x
                    caja_temp.y2 = espacio.y1 + rotacion.y
                    caja_temp.z2 = espacio.z1 + rotacion.z
    
                    # Verificar que la caja no sea flotante
                    if self.caja_no_flotante(caja_temp.x1, caja_temp.y1, caja_temp.z1, caja_temp.x2, caja_temp.y2, caja_temp.z2):
                        x = False
                        if revisar_colisiones:
                            B1 = [caja_temp.x1, caja_temp.y1, caja_temp.z1]
                            B2 = [caja_temp.x2, caja_temp.y2, caja_temp.z2]
                            P5, P6, diametro = calcular_puntos(B1, B2)
                            L1, L2 = crear_lineas(P5, P6, diametro)
                            x = verificar_colisiones(L1, L2, B1, B2)
                        if not x:
                            # Actualizar la información original de la caja
                            caja.x1 = caja_temp.x1
                            caja.y1 = caja_temp.y1
                            caja.z1 = caja_temp.z1
                            caja.x2 = caja_temp.x2
                            caja.y2 = caja_temp.y2
                            caja.z2 = caja_temp.z2
                            caja.Rotacion_Asignada = rotacion
    
                            # Empacar caja
                            self.cajas_empacadas.append(caja)
                            # Registrar la adición de la caja
                            self.logger.log_box(caja.ID_caja, caja.TipoCaja.ID_Tipo, caja.x1, caja.y1, caja.z1, caja.x2, caja.y2, caja.z2, visualizar)
                            return espacio
        #print(len(self.Espacios_posibles))
        return False  # La caja no pudo ser colocada en ningún espacio


    def are_spaces_adjacent(self, space1: EspacioMaximal, space2: EspacioMaximal):
        #same_height = space1.z1 == space2.z1
        x_adjacent = (space1.x2 == space2.x1 and (space1.y1 < space2.y2 and space1.y2 > space2.y1)) or \
                     (space1.x1 == space2.x2 and (space1.y1 < space2.y2 and space1.y2 > space2.y1))
        y_adjacent = (space1.y2 == space2.y1 and (space1.x1 < space2.x2 and space1.x2 > space2.x1)) or \
                     (space1.y1 == space2.y2 and (space1.x1 < space2.x2 and space1.x2 > space2.x1))
        return 'x' if x_adjacent else ('y' if y_adjacent else False)

    def merge_spaces(self, space1: EspacioMaximal, space2: EspacioMaximal, axis):
        if axis == "x":
            x1 = min(space1.x1, space2.x1)
            x2 = max(space1.x2, space2.x2)
            y1 = max(space1.y1, space2.y1)
            y2 = min(space1.y2, space2.y2)
        elif axis == "y":
            x1 = max(space1.x1, space2.x1)
            x2 = min(space1.x2, space2.x2)
            y1 = min(space1.y1, space2.y1)
            y2 = max(space1.y2, space2.y2)

        z1 = space1.z1
        z2 = space1.z2
        return EspacioMaximal(x1, y1, z1, x2, y2, z2)

    def merge_adjacent_spaces(self):
        ans = False
        i = 0
        while i < len(self.Espacios):
            space1 = self.Espacios[i]
            j = i + 1
            while j < len(self.Espacios):
                space2 = self.Espacios[j]
                axis = self.are_spaces_adjacent(space1, space2)
                if axis:
                    #Indicador de que hubo cambios
                    ans = True
                    new_space = self.merge_spaces(space1, space2, axis)
                    self.Espacios.append(new_space)
                    self.logger.log_space("Merged", space1.x1, space1.y1, space1.z1, space1.x2, space1.y2, space1.z2)
                    self.logger.log_space("Merged", space2.x1, space2.y1, space2.z1, space2.x2, space2.y2, space2.z2)
                    self.logger.log_space("Created", new_space.x1, new_space.y1, new_space.z1, new_space.x2, new_space.y2, new_space.z2)
                    break
                j += 1
            i += 1
        return ans

    def is_space_contained(self, spaceA: EspacioMaximal, spaceB: EspacioMaximal) -> bool:
        x_condition = spaceA.x1 >= spaceB.x1 and spaceA.x2 <= spaceB.x2
        y_condition = spaceA.y1 >= spaceB.y1 and spaceA.y2 <= spaceB.y2
        z_condition = spaceA.z1 >= spaceB.z1 and spaceA.z2 <= spaceB.z2
        return x_condition and y_condition and z_condition

    def revisar_espacios_redundantes(self):
        spaces_to_delete = set()
        for i, spaceA in enumerate(self.Espacios):
            for j, spaceB in enumerate(self.Espacios):
                if i != j and self.is_space_contained(spaceA, spaceB):
                    spaces_to_delete.add(spaceA)
        self.Espacios = [space for space in self.Espacios if space not in spaces_to_delete]
        # Log spaces that are being deleted
        for space in spaces_to_delete:
            self.logger.log_space("Deleted", space.x1, space.y1, space.z1, space.x2, space.y2, space.z2)

    def actualizar_espacios_disponibles(self):
        loop_counter = 0
        max_loops = 10
        while self.merge_adjacent_spaces() and loop_counter < max_loops:
            self.revisar_espacios_redundantes()
            loop_counter += 1
        self.imprimir_espacios("Actualizados", self.Espacios)
            
    def total_used_volume(self) -> float:
        total_volume = 0
        for caja in self.cajas_empacadas:
            total_volume += caja.Rotacion_Asignada.x * caja.Rotacion_Asignada.y * caja.Rotacion_Asignada.z
        return total_volume
    
    def utilization(self) -> float:
        used_volume = self.total_used_volume()
        max_volume = self.TipoPallet.x * self.TipoPallet.y * self.TipoPallet.z
        return (used_volume / max_volume) * 100
    #3
    def create_new_spaces(self, used_espacio: EspacioMaximal, caja: Caja):
        new_spaces = []
    
        # Space to the right of the box
        if caja.x2 < used_espacio.x2:
            new_space_right = EspacioMaximal(caja.x2, caja.y1, used_espacio.z1, used_espacio.x2, used_espacio.y2, used_espacio.z2)
            new_spaces.append(new_space_right)
        
        # Space to the left of the box
        if caja.x1 > used_espacio.x1:
            new_space_left = EspacioMaximal(used_espacio.x1, caja.y1, used_espacio.z1, caja.x1, used_espacio.y2, used_espacio.z2)
            new_spaces.append(new_space_left)
        
        # Space in front of the box
        if caja.y2 < used_espacio.y2:
            new_space_front = EspacioMaximal(caja.x1, caja.y2, used_espacio.z1, used_espacio.x2, used_espacio.y2, used_espacio.z2)
            new_spaces.append(new_space_front)
        
        # Space behind the box
        if caja.y1 > used_espacio.y1:
            new_space_behind = EspacioMaximal(caja.x1, used_espacio.y1, used_espacio.z1, used_espacio.x2, caja.y1, used_espacio.z2)
            new_spaces.append(new_space_behind)
    
        # Space above the box
        if caja.z2 < used_espacio.z2:
            new_space_above = EspacioMaximal(caja.x1, caja.y1, caja.z2, used_espacio.x2, used_espacio.y2, used_espacio.z2)
            new_spaces.append(new_space_above)
        
        # Adjust spaces for each packed box
        for new_space in new_spaces:
            #print(str(new_space.x1) + " " + str(new_space.y1) +" "+ str(new_space.z1) + " " +str(new_space.x2)+" "+str(new_space.y2)+" "+str(new_space.z2))
            for caja_empacada in self.cajas_empacadas:
                self.ajustar_espacio_por_caja(new_space, caja_empacada)
                
        # Remove invalid spaces
        new_spaces = [space for space in new_spaces if space.EsValido()]
        
        for espacio in self.Espacios:
            self.ajustar_espacio_por_caja(espacio, caja)
    
        # Remove the used space and add the new spaces
        self.Espacios.remove(used_espacio)
        self.Espacios.extend(new_spaces)
        self.imprimir_espacios("creados", self.Espacios)

    def ajustar_espacio_por_caja(self, espacio, caja):
        
    
        superpone_x = espacio.x1 < caja.x2 and espacio.x2 > caja.x1
        superpone_y = espacio.y1 < caja.y2 and espacio.y2 > caja.y1
        superpone_z = espacio.z1 < caja.z2 and espacio.z2 > caja.z1
    
        if superpone_x and superpone_y and superpone_z:
            if espacio.x2 > caja.x2:
                espacio.x1 = max(espacio.x1, caja.x2)
            else:
                espacio.x2 = min(espacio.x2, caja.x1)
    
            if espacio.y2 > caja.y2:
                espacio.y1 = max(espacio.y1, caja.y2)
            else:
                espacio.y2 = min(espacio.y2, caja.y1)
    
            if espacio.z2 > caja.z2:
                espacio.z1 = max(espacio.z1, caja.z2)
            else:
                espacio.z2 = min(espacio.z2, caja.z1)
    
        return espacio

    def caja_no_flotante(self, x1, y1, z1, x2, y2, z2):
        threshold = 1
        area_total = (x2 - x1) * (y2 - y1)
        area_ocupada = 0
        if z1 == 0:
            return True
        for caja in self.cajas_empacadas:
            # Si la caja está justo debajo del espacio dado
            if caja.z2 == z1:
                # Encuentra la superposición en x
                overlap_x = max(0, min(x2, caja.x2) - max(x1, caja.x1))
                # Encuentra la superposición en y
                overlap_y = max(0, min(y2, caja.y2) - max(y1, caja.y1))
                # Aumenta el área ocupada
                area_ocupada += overlap_x * overlap_y

        porcentaje_ocupado = area_ocupada / area_total

        return porcentaje_ocupado >= threshold
        #La función primero calcula el área total del espacio proporcionado. Luego, recorre todas las cajas ya empaquetadas y verifica si alguna de ellas está justo debajo del espacio dado. Si es así, calcula la superposición en x e y entre esa caja y el espacio dado y aumenta el area_ocupada en consecuencia. Finalmente, se compara el área ocupada con el área total para determinar si se cumple el umbral.

    def existe_colision( B1, B2 ): #B1 y B2 son los puntos finales donde la caja se empaca
        pass



    def imprimir_espacios(self, mensaje, espacios):
        pass
# =============================================================================
#         print(mensaje)
#         for i in espacios:
#             ans = ""
#             for j in i.p1:
#                 ans = ans + str(j) + " "
#             for j in i.p2:
#                 ans = ans + str(j) + " "
#             
#             print(ans)
# 
# =============================================================================
            
            


class PalletLogger:

    def __init__(self, pallet_id, max_dims):
        self.pallet_id = pallet_id
        self.folder_name = "PalletLogs"
        self.max_dims = max_dims
        
        # Check if folder exists, if not, create it
        if not os.path.exists(self.folder_name):
            os.makedirs(self.folder_name)
        
        # Now, the logs will be saved inside the folder 'PalletLogs'
        self.box_file_name = os.path.join(self.folder_name, f"Cajas Pallet {self.pallet_id} - {datetime.datetime.now().strftime('%Y-%m-%d %H-%M-%S')}.csv")
        self.space_file_name = os.path.join(self.folder_name, f"Espacios Pallet {self.pallet_id} - {datetime.datetime.now().strftime('%Y-%m-%d %H-%M-%S')}.csv")

        
        # Write headers for CSV files
        with open(self.box_file_name, "w") as box_file:
            box_file.write("Box ID,Box Type,x1,y1,z1,x2,y2,z2\n")
        
        with open(self.space_file_name, "w") as space_file:
            space_file.write("Action,x1,y1,z1,x2,y2,z2\n")

    def log_box(self, box_id, box_type, x1, y1, z1, x2, y2, z2, visualizar):
        with open(self.box_file_name, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([box_id, box_type, x1, y1, z1, x2, y2, z2])
        #print(box_id, box_type, x1, y1, z1, x2, y2, z2)
        filename = self.box_file_name
        if visualizar:
            vis.visualize_2d(filename, self.max_dims)
            vis.visualize_xz(filename, self.max_dims)
            vis.visualize_yz(filename, self.max_dims)
            vis.visualize_3d(filename, self.max_dims)
        
    def log_space(self, action, x1, y1, z1, x2, y2, z2):
        with open(self.space_file_name, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([action, x1, y1, z1, x2, y2, z2])

    def log_box_packaged(self):
        self.log_space("Box Packaged", "", "", "", "", "", "")
        
    def log_pallet_full(self):
        self.log_space("Pallet Full", "", "", "", "", "", "")
