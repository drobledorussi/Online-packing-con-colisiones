# -*- coding: utf-8 -*-
"""
Created on Tue Nov  14 01:31:21 2023

@author: Daniel Robledo
"""
# NOTA: Toda distancia está en cm a menos que se indique lo contrario

from Clases_Packing import TipoCaja, Caja, EspacioMaximal, TipoPallet, Pallet, PalletLogger
import random

def crear_tipos_cajas():
    #Tipos de cajas obtenidos del catálogo de Packaging.my https://www.packaging.my/store/p107/packaging-box-k-series.html
    tipos = []
    #Cajas de 25cm de alto
    K001 = TipoCaja("K001", 20, 20, 25, 0, 0, 3)
    K002 = TipoCaja("K002", 25, 25, 25, 0, 0, 3)
    K003 = TipoCaja("K003", 30, 30, 25, 0, 0, 3)
    K004 = TipoCaja("K004", 35, 30, 25, 0, 0, 3)
    K005 = TipoCaja("K005", 40, 25, 25, 0, 0, 3)
    K006 = TipoCaja("K006", 40, 35, 25, 0, 0, 3)
    #Cajas de 30cm de alto
    K007 = TipoCaja("K007", 25, 25, 30, 0, 0, 3)
    K008 = TipoCaja("K008", 30, 30, 30, 0, 0, 3)
    K009 = TipoCaja("K009", 35, 35, 30, 0, 0, 3)
    K010 = TipoCaja("K010", 40, 40, 30, 0, 0, 3)
    #Cajas de 40cm de alto
    K011 = TipoCaja("K011", 35, 17, 40, 0, 0, 3)
    
    tipos = [K001, K002, K003, K004, K005, K006]
    tipos_dict = {"K001" : K001, "K002": K002, "K003":K003, "K004":K004, "K005":K005,"K006" :K006}
    
    return tipos, tipos_dict

def crear_cajas(num, tipos: list):
    cajas = []
    vol = 0
    for i in range(num):
        tipo = random.choice(tipos)
        caja = Caja(tipo, "Caja "+ str(i+1))
        cajas.append(caja)
        vol += tipo.volumen
    return cajas

def crear_tipos_pallet():
    tipos_pallets = []
    #Los estandares se obtienen de esta página https://www.eurosender.com/en/packaging-materials/pallets-types
    #Para la altura máxima se asume 105cm
    #La altura se deduce de esta página https://ntslogistics.com/knowledge_base/pallet-shipping-dimensions/#:~:text=The%20standard%20packing%20height%20for,still%20hold%20a%20single%20stack.
    #en la sección "How many pallets fit in a truck?" Dice que se recomienda una altura de 48in, o 121.92cm, que al restar la altura estándar de un pallet de 14.5cm deja un espacio libre de 107.42. En este caso se aproxima a 105cm
    Eur1 = TipoPallet("Eur 1", 80, 120, 105)
    Eur2 = TipoPallet("Eur 2", 100, 120, 105) #Tambien es el estandar de USA
    Eur6 = TipoPallet("Eur 6", 60, 80, 105)
    Australia = TipoPallet("Aus", 116.5, 116.5, 105)
    T42x42 = TipoPallet("42x42", 106.7, 106.7, 105)
    T110x110 = TipoPallet("110x110", 110, 110, 105)
    Prueba = TipoPallet("Prueba", 60, 60, 60)
    
    #tipos_pallets = [Eur1, Eur2, Eur6, Australia, T110x110, T42x42]
    tipos_pallets = Eur2
    return tipos_pallets

def leer_cajas(lista, tipos_dict):
    cajas = []
    for fila in lista:
        ID = fila[0]
        tipo = tipos_dict[fila[1]]
        caja = Caja(tipo, ID)
        cajas.append(caja)
    return cajas