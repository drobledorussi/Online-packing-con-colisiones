# -*- coding: utf-8 -*-
"""
Created on Sun Aug 27 14:53:36 2023

@author: Daniel Robledo
"""

from Clases_Packing import TipoCaja, Caja, EspacioMaximal, TipoPallet, Pallet, PalletLogger
from Inicio_simulacion import *
import random
import csv
import time

#Lista con los tipos de caja
tipos, tipos_dict = crear_tipos_cajas()

#Se usará un sólo tipo de pallet, por lo que es una única variable
tipo_pallet = crear_tipos_pallet()

#Lista con las cajas que se van a usar
file = open("Instancias.txt", "r")
data = list(csv.reader(file, delimiter=","))
file.close()

start = time.time()

cajas = leer_cajas(data, tipos_dict)


pallets = []
pallets.append(Pallet("Pallet 1", tipo_pallet))
num_pallet = 0
pallet = pallets[num_pallet]

for caja in cajas:
    Hay_espacio = pallet.crear_espacios_posibles(caja)
    #Como la caja si cabe, se pasa a empacarla. Los 2 input boolean dicen si se debe visualizar y verificar colisiones, en ese orden
    espacio = pallet.place_box_in_space(caja, True, True)
    
    #Como la caja no cabe en el pallet actual, se crea un nuevo pallet
    if not espacio:
        #print(pallet.ID_Pallet + " lleno")
        #print("Utilización: " + str(round(pallet.utilization())) + "%")
        #print(pallet.ID_Pallet + "," + str(round(pallet.utilization())) + "%")
        #break
        num_pallet += 1
        pallets.append(Pallet("Pallet " + str(num_pallet+1), tipo_pallet))
        pallet = pallets[num_pallet]
    else:

        
        #Se crea los espacios nuevos a partir del espacio usado
        pallet.create_new_spaces(espacio, caja)
        
        #Se actualizan los espacios disponibles en el pallet 
        #(buscar espacios que se pueden unir y borrar los redundantes)
        pallet.actualizar_espacios_disponibles()
    

end = time.time()
print(end-start)     
#for pallet in pallets:
#    print(pallet.total_used_volume())
#    print(pallet.utilization())
    
#print(volumen)
    

#print(len(pallets)+1)





