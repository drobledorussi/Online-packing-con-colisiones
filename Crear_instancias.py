# -*- coding: utf-8 -*-
"""
Created on Wed Aug 23 19:24:13 2023

@author: Daniel Robledo
"""

from Clases_Packing import TipoCaja, Caja, EspacioMaximal, TipoPallet, Pallet, PalletLogger
from Inicio_simulacion import *
import random
import csv


#Lista con los tipos de caja
tipos, tipos_dict = crear_tipos_cajas()

#Se usará un sólo tipo de pallet, por lo que es una única variable
tipo_pallet = crear_tipos_pallet()

#Lista con las cajas que se van a usar
cajas = crear_cajas(500, tipos)

for caja in cajas:
    with open("cajas.txt", 'a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([caja.ID_caja, str(caja.TipoCaja.ID_Tipo), caja.TipoCaja.L, caja.TipoCaja.W, caja.TipoCaja.H, caja.TipoCaja.R[0], caja.TipoCaja.R[1], caja.TipoCaja.R[2]])

