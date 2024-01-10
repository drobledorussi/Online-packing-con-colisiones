# -*- coding: utf-8 -*-
"""
Created on Tue Oct 31 01:38:07 2023

@author: Daniel Robledo
"""

import random
import numpy as np
import time
from Colisiones import shortest_distance

# Number of pairs you want to generate
num_lineas = 1000000
num_pairs = num_lineas*3  # Change this to the desired number of pairs

# Generate and print random pairs
pairs = []
for _ in range(num_pairs):
    first_number = random.uniform(0, 4.99)
    second_number = random.uniform(first_number + 0.01, 5)
    pairs.append([first_number, second_number])

# Create NumPy arrays and group them into sets of two
Lineas = []
for i in range(0, len(pairs), 3):
    pairx = pairs[i]
    pairy = pairs[i + 1]
    pairz = pairs[i + 2]
    L1 = np.array([pairx[0], pairy[0], pairz[0]])
    L2 = np.array([pairx[1], pairy[1], pairz[1]])
    Lineas.append([L1, L2])
    
B1 = np.array([1, 1, 1])
B2 = np.array([6, 6, 6])

start = time.time()
for linea in Lineas:
    L1, L2 = linea
    colision = shortest_distance(L1, L2, B1, B2)
    
end = time.time()
execution_time = end - start
time_per_line = execution_time / num_lineas
print(f"Tested on {num_lineas} random lines")
print(f"Total execution time: {execution_time:.8f} seconds")
print(f"Execution time per line: {time_per_line: .8f} seconds" )
    